#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

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

    range_min_ = this->declare_parameter<double>("range_min", 0.03);
    range_max_ = this->declare_parameter<double>("range_max", 12.0);
    scan_frequency_ = this->declare_parameter<double>("scan_frequency", 10.0);

    clockwise_ = this->declare_parameter<bool>("clockwise", true);
    angle_offset_deg_ = this->declare_parameter<double>("angle_offset_deg", 0.0);
    bins_ = this->declare_parameter<int>("bins", 720);

    if (bins_ < 90) {
      bins_ = 360;
    }

    bins_per_deg_ = static_cast<double>(bins_) / 360.0;
    angle_increment_ = (2.0 * M_PI) / static_cast<double>(bins_);

    ranges_.assign(bins_, std::numeric_limits<float>::infinity());
    intensities_.assign(bins_, 0.0f);
    rx_buffer_.reserve(4096);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "LDLiDAR D500 decoder started on %s at %d baud",
      port_.c_str(),
      baudrate_
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&D500Node::readLoop, this)
    );
  }

  ~D500Node()
  {
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
    // erase(begin) によるパケットごとの O(N) memmove を避け、rx_head_ を進めて遅延圧縮する。
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

    for (int i = 0; i < POINTS_PER_PACKET; ++i) {
      const size_t offset = 6 + static_cast<size_t>(i) * 3;
      const uint16_t distance_mm = readU16LE(packet, offset);
      const uint8_t intensity = packet[offset + 2];

      double angle_deg = start_angle_deg + step * static_cast<double>(i);
      if (angle_deg >= 360.0) {
        angle_deg -= 360.0;
      }

      // 0° を跨ぐパケットの前後で点が混ざらないよう、点単位のラップでスキャンを区切る。
      if (has_last_angle_ && last_point_angle_deg_ > 270.0 && angle_deg < 90.0) {
        publishScan(speed);
        resetScan();
      }

      addPoint(angle_deg, distance_mm, intensity);
      last_point_angle_deg_ = angle_deg;
      has_last_angle_ = true;
    }
  }

  void addPoint(double angle_deg, uint16_t distance_mm, uint8_t intensity)
  {
    if (distance_mm == 0) {
      return;
    }

    const double range_m = static_cast<double>(distance_mm) * 0.001;
    if (range_m < range_min_ || range_m > range_max_) {
      return;
    }

    if (clockwise_) {
      angle_deg = 360.0 - angle_deg;
    }

    // LaserScan の angle_min=-pi に対応させるため +180° シフトして [0, 360) に正規化する。
    double shifted_deg = std::fmod(angle_deg + angle_offset_deg_ + 180.0, 360.0);
    if (shifted_deg < 0.0) {
      shifted_deg += 360.0;
    }

    // 切り捨てだと全ビームが平均 0.5 ビン偏るため、最近傍丸め + 循環モジュロで割り当てる。
    const int index = static_cast<int>(std::lround(shifted_deg * bins_per_deg_)) % bins_;

    if (!std::isfinite(ranges_[index]) || range_m < ranges_[index]) {
      ranges_[index] = static_cast<float>(range_m);
      intensities_[index] = static_cast<float>(intensity);
    }
  }

  void publishScan(uint16_t speed)
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = frame_id_;

    scan.angle_min = static_cast<float>(-M_PI);
    // bins_ 個の配列に対し angle_min + (bins_ - 1) * angle_increment が厳密な最終ビン角度。
    scan.angle_max = static_cast<float>(-M_PI + angle_increment_ * static_cast<double>(bins_ - 1));
    scan.angle_increment = static_cast<float>(angle_increment_);

    double scan_time_s = 1.0 / scan_frequency_;
    if (speed > 0) {
      const double measured_s = 360.0 / static_cast<double>(speed);
      if (measured_s >= 0.02 && measured_s <= 2.0) {
        scan_time_s = measured_s;
      }
    }
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

  bool has_last_angle_{false};
  double last_point_angle_deg_{0.0};
  int publish_counter_{0};

  size_t rx_head_{0};
  std::vector<uint8_t> rx_buffer_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D500Node>());
  rclcpp::shutdown();
  return 0;
}
