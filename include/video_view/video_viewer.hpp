#ifndef VIDEO_VIEWER
#define VIDEO_VIEWER

#include <rcutils/logging_macros.h>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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
