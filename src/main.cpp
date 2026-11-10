#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_landmark_detection/landmark_detector_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_landmark_detection::LandmarkDetectorServer>();
  // Second-phase init: shared_from_this() works now that a shared_ptr owns
  // the node, and the plugins are handed it.
  node->configure();

  // Multi-threaded: a detector decodes and searches frames on its own
  // callback group while the service answers on the server's, so a 30 Hz
  // image stream never delays a DetectLandmark answer.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
