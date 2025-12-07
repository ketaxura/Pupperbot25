#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class CameraViewerCpp : public rclcpp::Node
{
public:
  CameraViewerCpp()
  : Node("camera_viewer_cpp")
  {
    using std::placeholders::_1;

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/oak/rgb/image_raw", 10,
      std::bind(&CameraViewerCpp::imageCallback, this, _1));

    tags_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
          "/detections",   // <-- REAL TOPIC
          10,
          std::bind(&CameraViewerCpp::tagsCallback, this, _1));


    RCLCPP_INFO(this->get_logger(), "C++ Camera viewer with AprilTag overlay started.");
  }

private:
  void tagsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_detections_ = *msg;  // copy
    }
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received AprilTagDetectionArray with %zu detections",
      msg->detections.size());
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try
    {
      // Use whatever encoding is in the message; convert to BGR if needed later
      frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      if (msg->encoding == "rgb8")
      {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Copy detections under lock
    apriltag_msgs::msg::AprilTagDetectionArray detections_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      detections_copy = last_detections_;
    }

    const int img_w = static_cast<int>(frame.cols);
    const int img_h = static_cast<int>(frame.rows);

    // No detections -> just show image
    if (detections_copy.detections.empty())
    {
      cv::imshow("OAK RGB + AprilTags (C++)", frame);
      cv::waitKey(1);
      return;
    }

    // Draw detections
    for (const auto &det : detections_copy.detections)
    {
      // Debug print (throttled)
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Tag id=%d centre=(%.1f, %.1f) first_corner=(%.1f, %.1f)",
        det.id, det.centre.x, det.centre.y,
        det.corners[0].x, det.corners[0].y);

      // Draw corners as a quad
      if (det.corners.size() == 4)
      {
        std::vector<cv::Point> pts;
        pts.reserve(4);

        for (const auto &c : det.corners)
        {
          int x = static_cast<int>(c.x);
          int y = static_cast<int>(c.y);

          // Sanity check: must be inside image
          if (x < 0 || x >= img_w || y < 0 || y >= img_h)
          {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "Corner (%.1f, %.1f) for tag %d is outside image (%dx%d)",
              c.x, c.y, det.id, img_w, img_h);
          }

          pts.emplace_back(x, y);
        }

        for (int i = 0; i < 4; ++i)
        {
          cv::line(
            frame,
            pts[i],
            pts[(i + 1) % 4],
            cv::Scalar(0, 0, 255),   // bright red
            3,                        // thicker line
            cv::LINE_AA);
        }
      }

      // Draw centre
      int cx = static_cast<int>(det.centre.x);
      int cy = static_cast<int>(det.centre.y);
      cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);

      // Tag ID
      std::string text = "ID: " + std::to_string(det.id);
      cv::putText(frame, text,
                  cv::Point(cx + 8, cy - 8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow("OAK RGB + AprilTags (C++)", frame);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tags_sub_;

  apriltag_msgs::msg::AprilTagDetectionArray last_detections_;
  std::mutex mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraViewerCpp>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
