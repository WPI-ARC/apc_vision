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

        std::vector<cv::Point2f> detected_points;
        std::vector<cv::Point3f> pattern_points;
        std::vector<std::vector<float> > tvecs;
        std::vector<std::vector<float> > rvecs;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        float square_width = 0.0245;
        for(int j = 0; j < 4; ++j) {
            for(int i = 0; i < 5; ++i) {
                pattern_points.push_back(cv::Point3f(i*square_width, j*square_width, 0.0));
            }
        }

        std::vector<std::vector<cv::Point2f> > im_points;
        im_points.push_back(detected_points);
        std::vector<std::vector<cv::Point3f> > obj_points;
        obj_points.push_back(pattern_points);

        bool result = cv::findChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points);
        cv::calibrateCamera(im_points, obj_points, lastImage_left->image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

        cv::drawChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points, result);
        cv::imshow("calibration", lastImage_left->image);

        cv::FileStorage fs("camera_calib.yaml", cv::FileStorage::WRITE);

        fs << "cameraMatrix" << cameraMatrix;
        fs << "distCoeffs" << distCoeffs;
        fs << "rvecs" << rvecs;
        fs << "tvecs" << tvecs;
        fs.release();

        cv::waitKey(0);
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
