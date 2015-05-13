// Ros include
#include <ros/ros.h>
// TF includes
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
// Eigen
#include <Eigen/Geometry>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// ROS <-> OpenCV
#include <cv_bridge/cv_bridge.h>
// Image encodings
#include <sensor_msgs/image_encodings.h>

// Camera topic names
#if defined(USE_XYZRGB_ID)
const static std::string left_xyz_topic = "/camera_left/points_xyzrgb";
#elif defined(USE_XYZI_ID)
const static std::string left_xyz_topic = "/camera_left/points_xyzconfidence";
#endif
const static std::string left_image_topic = "/camera_left/color";
const static std::string left_index_image_topic = "/camera_left/depth_indices";
// Frame names
const static std::string shelf_frame = "/shelf";
const static std::string left_camera_frame = "/camera_left_depth_optical_frame";

class Calibrator {
public:
    Calibrator(ros::NodeHandle& nh) :
        left_image_sub(nh.subscribe(left_image_topic, 1, &Calibrator::left_image_cb, this))
    {}

protected:
    ros::Subscriber left_image_sub;
    tf::TransformListener listener;

    //sensor_msgs::ImageConstPtr lastImage_left;
    cv_bridge::CvImagePtr lastImage_left;

    void left_image_cb(const sensor_msgs::ImageConstPtr& image) {
        lastImage_left = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

        std::vector<cv::Point2f> corners;
        bool result = cv::findChessboardCorners(lastImage_left->image, cv::Size(5, 4), corners);
        cv::drawChessboardCorners(lastImage_left->image, cv::Size(5, 4), corners, result);
        cv::imshow("calibration", lastImage_left->image);
        cv::waitKey(1);
    }

public:
    void calibrate() {
        // Do calibration-y things here
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration_node");
    ros::NodeHandle nh;

    Calibrator calibrator(nh);

    ros::spin();

}
