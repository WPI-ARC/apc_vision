// Ros include
#include <ros/ros.h>
// TF includes
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
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
const static std::string base_frame = "/base_link";
const static std::string tool_frame = "/arm_left_link_tool0";

class Calibrator {
public:
    Calibrator(ros::NodeHandle& nh) :
        left_image_sub(nh.subscribe(left_image_topic, 1, &Calibrator::left_image_cb, this))
    {}

protected:
    ros::Subscriber left_image_sub;
    boost::mutex image_mutex;
    tf::TransformListener listener;

    std::vector<Eigen::Affine3d> transforms;
    std::vector<std::vector<cv::Point2f> > image_points;

    //sensor_msgs::ImageConstPtr lastImage_left;
    cv_bridge::CvImagePtr lastImage_left;

    void left_image_cb(const sensor_msgs::ImageConstPtr& image) {
        boost::mutex::scoped_lock lock(image_mutex);
        lastImage_left = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }

public:
    void take_sample() {
        // Lock the image
        boost::mutex::scoped_lock lock(image_mutex);

        ros::Time stamp = ros::Time::now();
        std::vector<cv::Point2f> detected_points;
        tf::StampedTransform transform;
        Eigen::Affine3d tool_base_tf;

        // Wait for tool->base transform (2 sec)
        bool success = listener.waitForTransform(base_frame, tool_frame, stamp, ros::Duration(2.0));
        // If transform isn't found in that time, give up
        if(!success) {
            ROS_ERROR("Couldn't look up camera to shelf transform!");
            return;
        }
        // Otherwise, get the transform
        listener.lookupTransform(base_frame, tool_frame, stamp, transform);
        // Get an eigen transform from the tf one
        tf::transformTFToEigen(transform, tool_base_tf);

        if(cv::findChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points)) {
            transforms.push_back(tool_base_tf);
            image_points.push_back(detected_points);

            cv::drawChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points, true);
            cv::imshow("calibration", lastImage_left->image);
        } else {
            ROS_WARN("Could not detect entire checkerboard in this image. Skipping");
        }

        cv::waitKey(0);
    }

    void calibrate() {
        std::vector<cv::Point3f> pattern_points;
        float square_width = 0.0245;
        for(int j = 0; j < 4; ++j) {
            for(int i = 0; i < 5; ++i) {
                pattern_points.push_back(cv::Point3f(i*square_width, j*square_width, 0.0));
            }
        }

        /*
        for(int i = 0; i < poses.size(); ++i) {
            goto_pose(poses[i]);
            take_sample();
        }
        */

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        std::vector<cv::Mat_<double> > rotations;
        std::vector<cv::Mat_<double> > translations;
        std::vector<Eigen::Affine3d> transforms;

        cv::calibrateCamera(pattern_points, image_points, lastImage_left->image.size(), cameraMatrix, distCoeffs, rotations, translations);

        for(int i = 0; i < rotations.size(); ++i) {
            cv::Rodrigues(rotations[i], rotations[i]);
            Eigen::Translation3d translation(translations[i].at<double>(0), translations[i].at<double>(0), translations[i].at<double>(0));
            Eigen::Map<Eigen::Matrix3d> rotation((double*)rotations[i].data);
            Eigen::Affine3d transform = rotation*translation;
            transforms.push_back(transform);
        }


        cv::FileStorage fs("camera_calib.yaml", cv::FileStorage::WRITE);
        fs << "cameraMatrix" << cameraMatrix;
        fs << "distCoeffs" << distCoeffs;
        fs << "rvecs" << rotations;
        fs << "tvecs" << translations;
        fs.release();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(1);

    Calibrator calibrator(nh);
    calibrator.calibrate();

    spinner.stop();
}
