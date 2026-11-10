#ifndef SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__ARUCO_DETECTOR_HPP_
#define SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__ARUCO_DETECTOR_HPP_

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "syncai_landmark_detection/landmark_detector.hpp"
#include "syncai_landmark_detection/plugins/aruco_detector/marker_geometry.hpp"
#include "syncai_landmark_detection/plugins/aruco_detector/solution_selector.hpp"

namespace syncai_landmark_detection
{

/**
 * @class ArucoDetector
 * @brief ArUco markers in syncai_camera_feeder's image_raw/compressed, with
 * its latched camera_info: the detection SyncAI-Robot-Charger's detector_node
 * did, without its stream handling (README, "The ArUco plugin").
 *
 * Every frame while active: decoded straight to grey, searched around where
 * the group's markers were last seen (the whole frame when that misses), then
 * per landmark the outline picked (aruco::pickMarker), the corners
 * undistorted with camera_info, both PnP solutions, the choice and the gate
 * (aruco::SolutionSelector), and the result kept as a Sighting in the
 * camera's frame at the image's capture stamp.
 */
class ArucoDetector : public LandmarkDetector
{
public:
  ArucoDetector() = default;
  ~ArucoDetector() override;

  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;
  void addLandmark(const std::string & landmark_id) override;
  void activate() override;
  void deactivate() override;
  SensorStatus sensorStatus() const override;
  std::optional<Sighting> latest(const std::string & landmark_id) const override;

private:
  struct Landmark
  {
    int marker_id{0};
    double marker_size_m{0.0};
    aruco::SolutionSelector selector;
    std::optional<cv::Rect> last_box;  // the marker's bounding box in the last frame
    std::optional<Sighting> latest;
    uint16_t consecutive{0};
  };

  // Landmarks sharing a dictionary and polarity are found by one
  // cv::aruco::detectMarkers call.
  struct Group
  {
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    std::vector<std::string> landmarks;
  };

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  /** @brief onImage's work; onImage only turns what it throws into frame_error_. */
  void processImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);
  std::optional<cv::Rect> groupRoi(const Group & group, const cv::Size & frame) const;
  cv::Rect roiAround(const cv::Rect & box, const cv::Size & frame) const;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ArucoDetector")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Parameters (initialize).
  std::string image_topic_;
  std::string camera_info_topic_;
  double frame_timeout_{1.0};
  int corner_refine_win_px_{2};
  bool roi_enabled_{true};
  double roi_margin_{0.75};
  int roi_min_size_px_{160};
  aruco::SelectorParams selector_params_;

  mutable std::mutex mutex_;  // everything below
  std::map<std::string, Landmark> landmarks_;
  std::map<std::string, Group> groups_;  // key: dictionary + polarity
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  bool active_{false};
  // Bumped by activate(): a frame searched across it belongs to the previous
  // activation and is dropped, not merged into the cleared state.
  uint64_t generation_{0};
  rclcpp::Time activated_at_;
  std::optional<aruco::CameraModel> camera_model_;
  std::string camera_model_error_;             // why camera_info cannot be used; empty when it can
  std::optional<rclcpp::Time> last_frame_at_;  // node clock, on arrival
  std::string frame_error_;  // why the last frame could not be used; empty when it could
};

}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__PLUGINS__ARUCO_DETECTOR__ARUCO_DETECTOR_HPP_
