/**
 * Recorder node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 6, 2023
 */

#ifndef ROS2_STREAMS__RECORDER_HPP
#define ROS2_STREAMS__RECORDER_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/image.hpp>

using namespace sensor_msgs::msg;

namespace ROS2Streams
{

/**
 * Node that saves images to a file.
 */
class Recorder : public DUANode::NodeBase
{
public:
  Recorder(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~Recorder();

private:
  /* Node initialization routines. */
  void init_parameters();

  /* Node callbacks. */
  void image_callback(const Image::ConstSharedPtr & msg);

  /* Node parameters. */
  bool grayscale_ = false;
  std::string output_path_ = "";
  std::string transport_ = "raw";
  std::vector<std::string> topics_{};

  /* Internal state variables. */
  std::vector<std::shared_ptr<image_transport::Subscriber>> subs_{};
};

} // namespace ROS2Streams

#endif // ROS2_STREAMS__RECORDER_HPP
