#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;

class AprilTagPoseDetector : public rclcpp::Node
{
public:
  AprilTagPoseDetector() : Node("apriltag_pose_detector")
  {
    image_topic_       = declare_parameter<std::string>("image_topic", "/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");

    marker_size_m_ = declare_parameter<double>("marker_size_m", 0.055);
    marker_id_     = declare_parameter<int>("marker_id", 239);

    // Словарь: по умолчанию ArUco DICT_4X4_250 (=2)
    dictionary_id_ = declare_parameter<int>("dictionary_id", (int)cv::aruco::DICT_4X4_250);

    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    frame_override_      = declare_parameter<std::string>("sensor_frame_override", "");

    detector_params_ = cv::aruco::DetectorParameters::create();

    // pubs
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      image_topic_ + std::string("_apriltag_pose"), rclcpp::QoS(10).reliable());

    if (publish_debug_image_) {
      debug_pub_ = image_transport::create_publisher(
        this, image_topic_ + std::string("_apriltag_detection"));
    }

    // subs
    image_sub_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&AprilTagPoseDetector::imageCallback, this, std::placeholders::_1),
      "raw",
      rclcpp::QoS(10).reliable().get_rmw_qos_profile());

    camera_info_sub_ = create_subscription<CameraInfo>(
      camera_info_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(&AprilTagPoseDetector::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Started. image=%s camera_info=%s dict=%d id=%d size=%.4f (expects rgb8)",
      image_topic_.c_str(), camera_info_topic_.c_str(),
      dictionary_id_, marker_id_, marker_size_m_);
  }

private:
  void cameraInfoCallback(const CameraInfo::ConstSharedPtr msg)
  {
    camera_info_ = msg;

    K_ = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      K_.at<double>(i / 3, i % 3) = msg->k[i];
    }

    D_ = cv::Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < 5 && i < (int)msg->d.size(); ++i) {
      D_.at<double>(0, i) = msg->d[i];
    }

//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
//   "CameraInfo received. frame_id=%s  K=[%.1f %.1f %.1f; %.1f %.1f %.1f; %.1f %.1f %.1f]  D=[%.4f %.4f %.4f %.4f %.4f]  d_size=%zu",
//   msg->header.frame_id.c_str(),
//   msg->k[0], msg->k[1], msg->k[2],
//   msg->k[3], msg->k[4], msg->k[5],
//   msg->k[6], msg->k[7], msg->k[8],
//   (msg->d.size() > 0 ? msg->d[0] : 0.0),
//   (msg->d.size() > 1 ? msg->d[1] : 0.0),
//   (msg->d.size() > 2 ? msg->d[2] : 0.0),
//   (msg->d.size() > 3 ? msg->d[3] : 0.0),
//   (msg->d.size() > 4 ? msg->d[4] : 0.0),
//   msg->d.size()
// );

    calib_ready_ = true;
  }

  cv::Ptr<cv::aruco::Dictionary> dict() const
    {
    return cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id_));
    }

  static geometry_msgs::msg::Quaternion rvecToQuat(const cv::Vec3d &rvec)
  {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    tf2::Matrix3x3 m(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );

    tf2::Quaternion q;
    m.getRotation(q);

    geometry_msgs::msg::Quaternion out;
    out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
    return out;
  }

  void publishDebugBgr(const Image::ConstSharedPtr &msg, const cv::Mat &bgr)
  {
    if (!publish_debug_image_) return;

    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = bgr;
    debug_pub_.publish(out.toImageMsg());
  }

  void imageCallback(const Image::ConstSharedPtr msg)
  {
    if (!calib_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /camera_info...");
      return;
    }

    if (msg->encoding != sensor_msgs::image_encodings::RGB8) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Expected rgb8, got: %s", msg->encoding.c_str());
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }

    // RGB -> GRAY
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);

    // detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(gray, dict(), corners, ids, detector_params_, rejected);

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
    //   "detected=%zu rejected=%zu dict=%d",
    //   ids.size(), rejected.size(), dictionary_id_);

    // debug base image
    cv::Mat dbg;
    cv::cvtColor(gray, dbg, cv::COLOR_GRAY2BGR);

    if (ids.empty()) {
      if (!rejected.empty()) {
        cv::aruco::drawDetectedMarkers(dbg, rejected);
      }
      publishDebugBgr(msg, dbg);
      return;
    }

    cv::aruco::drawDetectedMarkers(dbg, corners, ids);

    // pose for all
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, (float)marker_size_m_, K_, D_, rvecs, tvecs);

    // find chosen id
    int chosen = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == marker_id_) { chosen = (int)i; break; }
    }

    // draw axes for all markers
    for (size_t i = 0; i < ids.size(); ++i) {
      cv::aruco::drawAxis(dbg, K_, D_, rvecs[i], tvecs[i], (float)(marker_size_m_ * 0.5));
    }

    // publish pose only if our marker exists
    if (chosen >= 0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = msg->header.stamp;

      if (!frame_override_.empty()) pose.header.frame_id = frame_override_;
      else if (camera_info_) pose.header.frame_id = camera_info_->header.frame_id;
      else pose.header.frame_id = msg->header.frame_id;

      const auto &t = tvecs[chosen];
      const auto &r = rvecs[chosen];

      pose.pose.position.x = t[0]/10;
      pose.pose.position.y = t[1]/10;
      pose.pose.position.z = t[2]/10;
      pose.pose.orientation = rvecToQuat(r);

      pose_pub_->publish(pose);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Marker id=%d not found in this frame (found %zu markers)", marker_id_, ids.size());
    }

    publishDebugBgr(msg, dbg);
  }

private:
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string frame_override_;

    double marker_size_m_{0.055};
    int marker_id_{239};
    int dictionary_id_{(int)cv::aruco::DICT_4X4_250};
    bool publish_debug_image_{true};

    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    image_transport::Publisher debug_pub_;

    CameraInfo::ConstSharedPtr camera_info_;
    bool calib_ready_{false};
    cv::Mat K_, D_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagPoseDetector>());
  rclcpp::shutdown();
  return 0;
}