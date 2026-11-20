// FastSAM-x を NPU で回し、注釈付き画像を ROS トピックへ流す最小ノード。
//
// 参照実装: cqu20160901/FastSAM_rknn_Cplusplus のデモ（postprocess.cc をそのまま使う）。
// デモは 1 枚の画像を処理して終わるので、ここでは「カメラ → 推論 → 配信」のループにする。
//
// 使い方:
//   fastsam_stream <model.rknn> [/dev/video0] [topic]
#include <sys/time.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "postprocess.h"
#include "rknn_api.h"

namespace {

constexpr int kCaptureWidth = 640;
constexpr int kCaptureHeight = 480;

double NowMs() {
  struct timeval tv {};
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

}  // namespace

class FastSamNode : public rclcpp::Node {
 public:
  FastSamNode(const std::string& model_path, const std::string& device,
              const std::string& topic)
      : rclcpp::Node("fastsam_x"), device_(device) {
    // Image ディスプレイは最新フレームだけ要る。深くすると古い絵が混ざる。
    pub_ = create_publisher<sensor_msgs::msg::Image>(topic, 1);
    if (!LoadModel(model_path)) {
      throw std::runtime_error("RKNN モデルを読めません: " + model_path);
    }
    cap_.open(device_, cv::CAP_V4L2);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, kCaptureWidth);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, kCaptureHeight);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!cap_.isOpened()) {
      throw std::runtime_error("カメラを開けません: " + device_);
    }
    input_.index = 0;
    input_.type = RKNN_TENSOR_UINT8;
    input_.size = input_w_ * input_h_ * 3;
    input_.fmt = RKNN_TENSOR_NHWC;
    input_.pass_through = 0;
    outputs_ = std::vector<rknn_output>(n_output_);
    RCLCPP_INFO(get_logger(), "モデル入力 %dx%d, 出力 %d 本, カメラ %s",
                input_w_, input_h_, n_output_, device_.c_str());
  }

  ~FastSamNode() override {
    // 出力バッファは毎フレーム release 済み。ここで二重に解放すると落ちる。
    rknn_destroy(ctx_);
    cap_.release();
  }

  void Spin() {
    while (rclcpp::ok()) {
      cv::Mat bgr;
      if (!cap_.read(bgr) || bgr.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "フレーム取得に失敗");
        continue;
      }
      Annotate(bgr);
    }
  }

 private:
  bool LoadModel(const std::string& path) {
    FILE* fp = fopen(path.c_str(), "rb");
    if (fp == nullptr) {
      RCLCPP_ERROR(get_logger(), "open failed: %s", path.c_str());
      return false;
    }
    fseek(fp, 0, SEEK_END);
    const long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    std::vector<char> data(static_cast<size_t>(size));
    if (fread(data.data(), 1, data.size(), fp) != data.size()) {
      fclose(fp);
      return false;
    }
    fclose(fp);

    if (rknn_init(&ctx_, data.data(), data.size(), 0, nullptr) != 0) {
      return false;
    }
    rknn_input_output_num io_num {};
    if (rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != 0) {
      return false;
    }
    n_input_ = static_cast<int>(io_num.n_input);
    n_output_ = static_cast<int>(io_num.n_output);
    if (n_input_ != 1) {
      return false;
    }

    rknn_tensor_attr in_attr {};
    in_attr.index = 0;
    if (rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr)) != 0) {
      return false;
    }
    if (in_attr.fmt == RKNN_TENSOR_NCHW) {
      input_h_ = static_cast<int>(in_attr.dims[2]);
      input_w_ = static_cast<int>(in_attr.dims[3]);
    } else {
      input_h_ = static_cast<int>(in_attr.dims[1]);
      input_w_ = static_cast<int>(in_attr.dims[2]);
    }

    out_scales_.assign(static_cast<size_t>(n_output_), 0.0f);
    out_zps_.assign(static_cast<size_t>(n_output_), 0);
    for (int i = 0; i < n_output_; ++i) {
      rknn_tensor_attr attr {};
      attr.index = i;
      if (rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) != 0) {
        return false;
      }
      out_scales_[i] = attr.scale;
      out_zps_[i] = attr.zp;
    }
    return true;
  }

  void Annotate(const cv::Mat& bgr) {
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    if (rgb.cols != input_w_ || rgb.rows != input_h_) {
      cv::resize(rgb, resized, cv::Size(input_w_, input_h_));
    } else {
      resized = rgb;
    }
    input_.buf = resized.data;

    const double t0 = NowMs();
    rknn_inputs_set(ctx_, n_input_, &input_);
    rknn_run(ctx_, nullptr);
    rknn_outputs_get(ctx_, n_output_, outputs_.data(), nullptr);
    const double t1 = NowMs();

    // postprocess.cc は int8 の生ポインタ配列を要求する
    std::vector<int8_t*> blobs(static_cast<size_t>(n_output_));
    for (int i = 0; i < n_output_; ++i) {
      blobs[i] = static_cast<int8_t*>(outputs_[i].buf);
    }

    std::vector<float> rects;
    cv::Mat seg_mask = cv::Mat::zeros(88, 160, CV_8UC3);
    GetResultRectFastSAM post;
    post.GetConvDetectionResult(blobs.data(), out_zps_, out_scales_, rects, seg_mask);
    for (int i = 0; i < n_output_; ++i) {
      rknn_outputs_release(ctx_, 1, &outputs_[i]);
    }
    const double t2 = NowMs();

    cv::Mat mask_resized;
    cv::resize(seg_mask, mask_resized, cv::Size(bgr.cols, bgr.rows), 0, 0, cv::INTER_LINEAR);
    cv::Mat canvas;
    cv::addWeighted(bgr, 0.45, mask_resized, 0.55, 0, canvas);

    const int objects = static_cast<int>(rects.size() / 6);
    for (int i = 0; i + 5 < static_cast<int>(rects.size()); i += 6) {
      const float conf = rects[i + 1];
      const int xmin = static_cast<int>(rects[i + 2] * bgr.cols + 0.5f);
      const int ymin = static_cast<int>(rects[i + 3] * bgr.rows + 0.5f);
      const int xmax = static_cast<int>(rects[i + 4] * bgr.cols + 0.5f);
      const int ymax = static_cast<int>(rects[i + 5] * bgr.rows + 0.5f);
      const cv::Point center((xmin + xmax) / 2, (ymin + ymax) / 2);
      char label[32];
      snprintf(label, sizeof(label), "%.2f", conf);
      cv::putText(canvas, label, cv::Point(xmin, std::max(12, ymin - 4)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
      cv::circle(canvas, center, 4, cv::Scalar(0, 255, 255), -1);
    }
    cv::putText(canvas,
                cv::format("%d obj  infer %.0fms  post %.0fms", objects, t1 - t0, t2 - t1),
                cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);

    Publish(canvas);

    ++frame_;
    if (frame_ % 20 == 0) {
      RCLCPP_INFO(get_logger(), "%d obj  infer %.1f ms  post %.1f ms",
                  objects, t1 - t0, t2 - t1);
    }
  }

  void Publish(const cv::Mat& bgr) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera_link";
    msg.height = static_cast<uint32_t>(bgr.rows);
    msg.width = static_cast<uint32_t>(bgr.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = 0;
    msg.step = static_cast<uint32_t>(bgr.cols * bgr.channels());
    const size_t bytes = static_cast<size_t>(msg.step) * msg.height;
    msg.data.resize(bytes);
    if (bgr.isContinuous()) {
      memcpy(msg.data.data(), bgr.data, bytes);
    } else {
      for (int y = 0; y < bgr.rows; ++y) {
        memcpy(msg.data.data() + static_cast<size_t>(y) * msg.step,
               bgr.ptr(y), msg.step);
      }
    }
    pub_->publish(msg);
  }

  rknn_context ctx_ {};
  int n_input_ = 0;
  int n_output_ = 0;
  int input_w_ = 0;
  int input_h_ = 0;
  rknn_input input_ {};
  std::vector<rknn_output> outputs_;
  std::vector<float> out_scales_;
  std::vector<int32_t> out_zps_;
  std::string device_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  int frame_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::string model = (argc > 1) ? argv[1] : "model/RK3588/FastSAM-x_640x352.rknn";
  const std::string device = (argc > 2) ? argv[2] : "/dev/video0";
  const std::string topic = (argc > 3) ? argv[3] : "/fastsam_image";
  try {
    auto node = std::make_shared<FastSamNode>(model, device, topic);
    node->Spin();
  } catch (const std::exception& e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
