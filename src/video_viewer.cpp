#include "video_view/video_viewer.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace video_view {

const rmw_qos_profile_t qos_string_to_qos(std::string str) {
#if !defined(DASHING) && !defined(ELOQUENT)
  if (str == "UNKNOWN")
    return rmw_qos_profile_unknown;
#endif
  if (str == "SYSTEM_DEFAULT")
    return rmw_qos_profile_system_default;
  if (str == "DEFAULT")
    return rmw_qos_profile_default;
  if (str == "PARAMETER_EVENTS")
    return rmw_qos_profile_parameter_events;
  if (str == "SERVICES_DEFAULT")
    return rmw_qos_profile_services_default;
  if (str == "PARAMETERS")
    return rmw_qos_profile_parameters;
  if (str == "SENSOR_DATA")
    return rmw_qos_profile_sensor_data;
  throw std::runtime_error("Unknown QoS string " + str);
}

VideoViewer::VideoViewer(const rclcpp::NodeOptions &options)
    : Node("video_viewer", options) {

  this->declare_parameter<std::string>("image_topic", "image");
  topic_ = this->get_parameter("image_topic").as_string();

  this->declare_parameter<std::string>("image_transport", "compressed");
  std::string transport = this->get_parameter("image_transport").as_string();

  this->declare_parameter<std::string>("rotate", "rotate_0");
  param_rotate_ = this->get_parameter("rotate").as_string();

  this->declare_parameter<std::string>("qos", "SENSOR_DATA");

  std::string qos_string = this->get_parameter("qos").as_string();
  auto qos = qos_string_to_qos(qos_string);

  sub_ = image_transport::create_subscription(
      this, topic_,
      std::bind(&VideoViewer::imageCallback, this, std::placeholders::_1),
      transport, qos);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Subscribing to topic: " << sub_.getTopic());
  RCLCPP_INFO_STREAM(this->get_logger(), "Using QoS:  " << qos_string);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Using Transport: " << sub_.getTransport());
}

void VideoViewer::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &img) {

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (param_rotate_ == "rotate_0") {
    cv::imshow(topic_, cv_ptr->image);
  } else if (param_rotate_ == "rotate_90") {
    cv::Mat rot_image;
    cv::rotate(cv_ptr->image, rot_image, cv::ROTATE_90_CLOCKWISE);
    cv::imshow(topic_, rot_image);

  } else if (param_rotate_ == "rotate_180") {
    cv::Mat rot_image;
    cv::rotate(cv_ptr->image, rot_image, cv::ROTATE_180);
    cv::imshow(topic_, rot_image);

  } else if (param_rotate_ == "rotate_270") {
    cv::Mat rot_image;
    cv::rotate(cv_ptr->image, rot_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imshow(topic_, rot_image);
  }

  char key = cv::waitKey(1);
  if (key == 27 || key == 'q') { // if ESC or q
    rclcpp::shutdown();
  }
}

} // namespace video_view

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(video_view::VideoViewer)
