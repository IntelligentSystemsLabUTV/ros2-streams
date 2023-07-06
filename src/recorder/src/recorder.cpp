/**
 * Recorder node implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 6, 2023
 */

#include <algorithm>

#include <recorder/recorder.hpp>

namespace ROS2Streams
{

/**
 * @brief Node constructor.
 */
Recorder::Recorder(const rclcpp::NodeOptions & opts)
: NodeBase("recorder", opts, true)
{
  init_parameters();

  // Create subscriptions for all topics
  for (auto & topic : topics_) {
    subs_.push_back(
      std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this,
          topic,
          std::bind(
            &Recorder::image_callback,
            this,
            std::placeholders::_1),
          transport_,
          DUAQoS::get_image_qos(10).get_rmw_qos_profile())));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to topic '%s' with transport '%s'",
      topic.c_str(),
      transport_.c_str());
  }

  RCLCPP_INFO(get_logger(), "Node initialized");
}

/**
 * @brief Node destructor.
 */
Recorder::~Recorder()
{
  for (auto & sub : subs_) {
    sub->shutdown();
    sub.reset();
  }
}

/**
 * @brief Callback that saves the image to a file.
 */
void Recorder::image_callback(const Image::ConstSharedPtr & msg)
{
  // Get image to the OpenCV space
  cv::Mat orig_frame(
    msg->height,
    msg->width,
    CV_8UC3,
    (void *)(msg->data.data()));

  // Post-process image
  cv::Mat frame;
  if (grayscale_) {
    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::cvtColor(orig_frame, frame, cv::COLOR_BGR2GRAY);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(orig_frame, frame, cv::COLOR_RGB2GRAY);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unsupported image encoding: '%s'",
        msg->encoding.c_str());
    }
  } else {
    frame = orig_frame;
  }

  // Remove all '/' from the frame ID and replace them with '_'
  std::string camera_frame_id = msg->header.frame_id;
  std::replace(camera_frame_id.begin(), camera_frame_id.end(), '/', '_');

  // Save image to file
  std::string filename =
    output_path_ + "/" +
    camera_frame_id + "_" +
    std::to_string(msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec) +
    ".jpg";
  cv::imwrite(filename, frame);
}

} // namespace ROS2Streams
