#ifndef VIDEO_VIEWER
#define VIDEO_VIEWER

#include "precompile.hpp"

namespace video_view {

class VideoViewer : public rclcpp::Node {

public:
  explicit VideoViewer(const rclcpp::NodeOptions &options);

  virtual ~VideoViewer() {}

protected:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img);

private:
  // subscribers
  image_transport::Subscriber sub_;

  // parameters
  std::string param_rotate_;
  std::string topic_;
};

} // namespace video_view

#endif // VIDEO_VIEWER
