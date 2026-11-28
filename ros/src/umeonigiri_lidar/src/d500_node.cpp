#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "umeonigiri_lidar/scan_processor.hpp"

namespace
{

constexpr std::array<uint8_t, 256> BuildCrc8Table() noexcept
{
  std::array<uint8_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    uint8_t crc = static_cast<uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                         : static_cast<uint8_t>(crc << 1);
    }
    table[static_cast<size_t>(i)] = crc;
  }
  return table;
}

inline constexpr std::array<uint8_t, 256> kCrc8Table = BuildCrc8Table();

}  // namespace

class D500Node : public rclcpp::Node
{
public:
  D500Node() : Node("d500_node")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = this->declare_parameter<int>("baudrate", 230400);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar_link");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/scan");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");

    range_min_ = this->declare_parameter<double>("range_min", 0.12);
    range_max_ = this->declare_parameter<double>("range_max", 12.0);
    scan_frequency_ = this->declare_parameter<double>("scan_frequency", 10.0);

    clockwise_ = this->declare_parameter<bool>("clockwise", true);
    angle_offset_deg_ = this->declare_parameter<double>("angle_offset_deg", 0.0);
    bins_ = this->declare_parameter<int>("bins", 720);

    filter_cfg_.speckle_filter_enabled =
      this->declare_parameter<bool>("speckle_filter_enabled", true);
    filter_cfg_.speckle_max_neighbor_diff_m = static_cast<float>(
      this->declare_parameter<double>("speckle_max_neighbor_diff", 0.25));
    filter_cfg_.deskew_enabled =
      this->declare_parameter<bool>("deskew_enabled", true);
    filter_cfg_.deskew_min_wz_rad_s =
      this->declare_parameter<double>("deskew_min_wz", 0.05);
    filter_cfg_.deskew_sign =
      this->declare_parameter<double>("deskew_sign", 1.0);
    filter_cfg_.min_intensity =
      this->declare_parameter<int>("min_intensity", 15);
    filter_cfg_.tilt_compensation_enabled =
      this->declare_parameter<bool>("tilt_compensation_enabled", false);
    filter_cfg_.max_tilt_rad =
      this->declare_parameter<double>("max_tilt_rad", 0.25);

    if (bins_ < 90) {
      bins_ = 360;
    }

    bins_per_deg_ = static_cast<double>(bins_) / 360.0;
    angle_increment_ = (2.0 * M_PI) / static_cast<double>(bins_);

    ranges_.assign(bins_, std::numeric_limits<float>::infinity());
    intensities_.assign(bins_, 0.0f);
    speckle_scratch_.resize(bins_);
    rx_buffer_.reserve(4096);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

    if (filter_cfg_.tilt_compensation_enabled && !imu_topic_.empty()) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          const auto & q = msg->orientation;
          const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
          const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
          const double sinp = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
          latest_roll_.store(std::atan2(sinr, cosr), std::memory_order_relaxed);
          latest_pitch_.store(std::asin(sinp), std::memory_order_relaxed);
        });
    }

    if (filter_cfg_.deskew_enabled && !odom_topic_.empty()) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_wz_.store(msg->twist.twist.angular.z, std::memory_order_relaxed);
          latest_odom_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
        });
    }

    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "LDLiDAR D500 decoder started on %s at %d baud (speckle=%s deskew=%s min_int=%d range_min=%.2fm)",
      port_.c_str(),
      baudrate_,
      filter_cfg_.speckle_filter_enabled ? "on" : "off",
      filter_cfg_.deskew_enabled ? "on" : "off",
      filter_cfg_.min_intensity,
      range_min_
    );

    // 500Hz タイマーポーリングをやめ、poll() ブロッキング待機スレッドで受信する。
    rx_stop_.store(false, std::memory_order_relaxed);
    rx_thread_ = std::thread([this] { pollLoop(); });
  }

  ~D500Node()
  {
    rx_stop_.store(true, std::memory_order_relaxed);
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  static constexpr uint8_t HEADER = 0x54;
  static constexpr uint8_t VER_LEN = 0x2C;
  static constexpr int POINTS_PER_PACKET = 12;
  static constexpr size_t PACKET_SIZE = 47;

  static uint16_t readU16LE(const uint8_t* data, size_t index) noexcept
  {
    return static_cast<uint16_t>(data[index]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[index + 1]) << 8);
  }

  static uint8_t crc8(const uint8_t* data, size_t length) noexcept
  {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
      crc = kCrc8Table[crc ^ data[i]];
    }
    return crc;
  }

  double currentScanTimeSec(uint16_t speed) const noexcept
  {
    double scan_time_s = 1.0 / scan_frequency_;
    if (speed > 0) {
      const double measured_s = 360.0 / static_cast<double>(speed);
      if (measured_s >= 0.02 && measured_s <= 2.0) {
        scan_time_s = measured_s;
      }
    }
    return scan_time_s;
  }

  double activeAngularVel() const
  {
    if (!filter_cfg_.deskew_enabled) {
      return 0.0;
    }
    const int64_t stamp_ns = latest_odom_ns_.load(std::memory_order_relaxed);
    if (stamp_ns <= 0) {
      return 0.0;
    }
    const int64_t age_ns = this->now().nanoseconds() - stamp_ns;
    if (age_ns < 0 || age_ns > 300000000LL) {
      return 0.0;
    }
    return latest_wz_.load(std::memory_order_relaxed);
  }

  bool openSerial()
  {
    serial_fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
      return false;
    }

    termios tty{};

    if (tcgetattr(serial_fd_, &tty) != 0) {
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B230400;

    if (baudrate_ != 230400) {
      RCLCPP_WARN(
        this->get_logger(),
        "Only 230400 is explicitly configured in this node. Using B230400 anyway."
      );
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      return false;
    }

    tcflush(serial_fd_, TCIFLUSH);

    return true;
  }

  void compactRxBufferIfNeeded() noexcept
  {
    if (rx_head_ == rx_buffer_.size()) {
      rx_buffer_.clear();
      rx_head_ = 0;
    } else if (rx_head_ >= 512 && rx_head_ * 2 >= rx_buffer_.size()) {
      const size_t rem = rx_buffer_.size() - rx_head_;
      std::memmove(rx_buffer_.data(), rx_buffer_.data() + rx_head_, rem);
      rx_buffer_.resize(rem);
      rx_head_ = 0;
    }
  }

  void pollLoop()
  {
    pollfd pfd{};
    pfd.fd = serial_fd_;
    pfd.events = POLLIN;
    while (!rx_stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      const int ret = poll(&pfd, 1, 50);
      if (ret > 0 && (pfd.revents & POLLIN) != 0) {
        readLoop();
      }
    }
  }

  void readLoop()
  {
    if (serial_fd_ < 0) {
      return;
    }

    uint8_t buffer[1024];
    bool received = false;
    for (;;) {
      const ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
      if (n > 0) {
        rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + n);
        received = true;
        if (static_cast<size_t>(n) < sizeof(buffer)) {
          break;
        }
      } else {
        break;
      }
    }

    if (received) {
      parsePackets();
    }
  }

  void parsePackets()
  {
    while (rx_buffer_.size() - rx_head_ >= PACKET_SIZE) {
      const uint8_t* ptr = rx_buffer_.data() + rx_head_;
      const size_t avail = rx_buffer_.size() - rx_head_;

      size_t offset = 0;
      while (offset + 1 < avail && !(ptr[offset] == HEADER && ptr[offset + 1] == VER_LEN)) {
        ++offset;
      }
      rx_head_ += offset;

      if (rx_buffer_.size() - rx_head_ < PACKET_SIZE) {
        break;
      }

      const uint8_t* packet = rx_buffer_.data() + rx_head_;
      if (packet[0] != HEADER || packet[1] != VER_LEN) {
        ++rx_head_;
        continue;
      }

      const uint8_t calculated_crc = crc8(packet, PACKET_SIZE - 1);
      if (calculated_crc != packet[PACKET_SIZE - 1]) {
        ++rx_head_;
        continue;
      }

      parseFrame(packet);
      rx_head_ += PACKET_SIZE;
    }

    compactRxBufferIfNeeded();
  }

  void parseFrame(const uint8_t* packet)
  {
    const uint16_t speed = readU16LE(packet, 2);
    const uint16_t start_angle_raw = readU16LE(packet, 4);
    const uint16_t end_angle_raw = readU16LE(packet, 42);

    const double start_angle_deg = static_cast<double>(start_angle_raw) * 0.01;
    const double end_angle_deg = static_cast<double>(end_angle_raw) * 0.01;

    double diff = end_angle_deg - start_angle_deg;
    if (diff < 0.0) {
      diff += 360.0;
    }

    const double step = diff / static_cast<double>(POINTS_PER_PACKET - 1);
    const double scan_time_s = currentScanTimeSec(speed);
    const double wz = activeAngularVel();

    for (int i = 0; i < POINTS_PER_PACKET; ++i) {
      const size_t offset = 6 + static_cast<size_t>(i) * 3;
      const uint16_t distance_mm = readU16LE(packet, offset);
      const uint8_t intensity = packet[offset + 2];

      double angle_deg = start_angle_deg + step * static_cast<double>(i);
      if (angle_deg >= 360.0) {
        angle_deg -= 360.0;
      }

      if (has_last_angle_ && last_point_angle_deg_ > 270.0 && angle_deg < 90.0) {
        publishScan(speed);
        resetScan();
      }

      const double frac = angle_deg / 360.0;
      const double deskew_deg = umeonigiri_lidar::ComputeDeskewAngleOffsetDeg(
        frac, scan_time_s, wz, filter_cfg_.deskew_min_wz_rad_s, filter_cfg_.deskew_sign);
      addPoint(angle_deg, deskew_deg, distance_mm, intensity);
      last_point_angle_deg_ = angle_deg;
      has_last_angle_ = true;
    }
  }

  void addPoint(double angle_deg, double deskew_deg, uint16_t distance_mm, uint8_t intensity)
  {
    if (distance_mm == 0) {
      return;
    }
    if (filter_cfg_.min_intensity > 0 && static_cast<int>(intensity) < filter_cfg_.min_intensity) {
      return;
    }

    double range_m = static_cast<double>(distance_mm) * 0.001;
    if (filter_cfg_.tilt_compensation_enabled) {
      const double beam_rad = (angle_deg + angle_offset_deg_) * (M_PI / 180.0);
      const float proj = umeonigiri_lidar::ProjectRangeToHorizontal(
        static_cast<float>(range_m), beam_rad,
        latest_roll_.load(std::memory_order_relaxed),
        latest_pitch_.load(std::memory_order_relaxed),
        filter_cfg_.max_tilt_rad);
      if (!std::isfinite(proj)) {
        return;
      }
      range_m = static_cast<double>(proj);
    }
    if (range_m < range_min_ || range_m > range_max_) {
      return;
    }

    if (clockwise_) {
      angle_deg = 360.0 - angle_deg;
    }

    double shifted_deg = std::fmod(angle_deg + angle_offset_deg_ + deskew_deg + 180.0, 360.0);
    if (shifted_deg < 0.0) {
      shifted_deg += 360.0;
    }

    const int index = static_cast<int>(std::lround(shifted_deg * bins_per_deg_)) % bins_;

    if (!std::isfinite(ranges_[index]) || range_m < ranges_[index]) {
      ranges_[index] = static_cast<float>(range_m);
      intensities_[index] = static_cast<float>(intensity);
    }
  }

  void publishScan(uint16_t speed)
  {
    if (filter_cfg_.speckle_filter_enabled) {
      umeonigiri_lidar::ApplySpeckleFilter(
        ranges_, filter_cfg_.speckle_max_neighbor_diff_m, speckle_scratch_);
    }

    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = frame_id_;

    scan.angle_min = static_cast<float>(-M_PI);
    scan.angle_max = static_cast<float>(-M_PI + angle_increment_ * static_cast<double>(bins_ - 1));
    scan.angle_increment = static_cast<float>(angle_increment_);

    const double scan_time_s = currentScanTimeSec(speed);
    scan.scan_time = static_cast<float>(scan_time_s);
    scan.time_increment = static_cast<float>(scan_time_s / static_cast<double>(bins_));

    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    scan.ranges = ranges_;
    scan.intensities = intensities_;

    scan_pub_->publish(scan);

    ++publish_counter_;
    if (publish_counter_ % 50 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Published /scan | speed_raw=%u | bins=%d",
        speed,
        bins_
      );
    }
  }

  void resetScan()
  {
    std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());
    std::fill(intensities_.begin(), intensities_.end(), 0.0f);
  }

private:
  std::string port_;
  std::string frame_id_;
  std::string topic_name_;
  std::string odom_topic_;
  std::string imu_topic_;

  int baudrate_;
  int serial_fd_{-1};
  int bins_;
  double bins_per_deg_{2.0};
  double angle_increment_{0.0};

  double range_min_;
  double range_max_;
  double scan_frequency_;
  double angle_offset_deg_;

  bool clockwise_;
  umeonigiri_lidar::ScanFilterConfig filter_cfg_{};

  std::atomic<double> latest_wz_{0.0};
  std::atomic<double> latest_roll_{0.0};
  std::atomic<double> latest_pitch_{0.0};
  std::atomic<int64_t> latest_odom_ns_{0};
  std::atomic<bool> rx_stop_{false};
  std::thread rx_thread_;

  bool has_last_angle_{false};
  double last_point_angle_deg_{0.0};
  int publish_counter_{0};

  size_t rx_head_{0};
  std::vector<uint8_t> rx_buffer_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;
  std::vector<float> speckle_scratch_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D500Node>());
  rclcpp::shutdown();
  return 0;
}
