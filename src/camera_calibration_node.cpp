// Ros include
#include <ros/ros.h>
// TF includes
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// ROS <-> OpenCV
#include <cv_bridge/cv_bridge.h>
// Image encodings
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <XmlRpcValue.h>
#include <moveit/move_group_interface/move_group.h>

#include <iostream>

#include <motoman_moveit/convert_trajectory_server.h>



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
    Calibrator(ros::NodeHandle& nh, std::string filename, std::vector<geometry_msgs::Pose> poses, Eigen::Affine3d color_depth_tf) :
        filename(filename), poses(poses), arm("arm_left_torso"), color_depth_tf(color_depth_tf),
        left_image_sub(nh.subscribe(left_image_topic, 1, &Calibrator::left_image_cb, this)),
        camera_pose_pub(nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 1))
    {
        arm.setPlannerId("RRTConnectkConfigDefault");
        arm.setPlanningTime(5.0);
    }
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    moveit::planning_interface::MoveGroup arm;
    std::string filename;
    ros::Subscriber left_image_sub;
    boost::mutex image_mutex;
    tf::TransformListener listener;
    ros::Publisher camera_pose_pub;

    Eigen::Affine3d color_depth_tf;
    std::vector<geometry_msgs::Pose> poses;
    std::vector<Eigen::Affine3d> tool_base_tfs;
    std::vector<std::vector<cv::Point2f> > image_points;

    //sensor_msgs::ImageConstPtr lastImage_left;
    cv_bridge::CvImagePtr lastImage_left;

    void left_image_cb(const sensor_msgs::ImageConstPtr& image) {
        boost::mutex::scoped_lock lock(image_mutex);
        lastImage_left = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }

public:
    void goto_pose(geometry_msgs::Pose pose) {
        moveit::planning_interface::MoveGroup::Plan plan;
        arm.setPoseTarget(pose);

        if(!arm.plan(plan)) {
            ROS_ERROR("Failed to find plan!");
            return;
        }

        motoman_moveit::convert_trajectory_server move;
        move.request.jointTraj = plan.trajectory_.joint_trajectory;
        ros::service::call("convert_trajectory_service", move);

        if(!move.response.success) {
            ROS_ERROR("Failed to execute plan!");
        }
    }

    void take_sample() {
        {
            // Lock the image
            boost::mutex::scoped_lock lock(image_mutex);

            ros::Time stamp = ros::Time::now();
            std::vector<cv::Point2f> detected_points;
            tf::StampedTransform transform;
            Eigen::Affine3d tool_base_tf;

            // Wait for tool->base transform (2 sec)
            bool success = listener.waitForTransform(tool_frame, base_frame, stamp, ros::Duration(2.0));
            // If transform isn't found in that time, give up
            if(!success) {
                ROS_ERROR("Couldn't look up camera to shelf transform!");
                return;
        }
        // Otherwise, get the transform
        listener.lookupTransform(tool_frame, base_frame, stamp, transform);
        // Get an eigen transform from the tf one
        tf::transformTFToEigen(transform, tool_base_tf);

        if(cv::findChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points)) {
            cv::circle(lastImage_left->image, detected_points[0], 10, cv::Scalar(0, 255, 0));
            tool_base_tfs.push_back(tool_base_tf);
            image_points.push_back(detected_points);

            cv::drawChessboardCorners(lastImage_left->image, cv::Size(5, 4), detected_points, true);
            cv::imshow("calibration", lastImage_left->image);
        } else {
            ROS_WARN("Could not detect entire checkerboard in this image. Skipping");
            cv::imshow("calibration", lastImage_left->image);
        }
        }

        cv::waitKey(0);
    }

    void calibrate() {
        std::vector<cv::Point3f> points;
        std::vector<std::vector<cv::Point3f> > pattern_points;
        float square_width = 0.0245;
        for(int j = 0; j < 4; ++j) {
            for(int i = 0; i < 5; ++i) {
                points.push_back(cv::Point3f(i*square_width, -j*square_width, 0.0));
            }
        }

        for(int i = 0; i < poses.size(); ++i) {
            goto_pose(poses[i]);
            take_sample();
        }

        for(int i = 0; i < image_points.size(); ++i) {
            pattern_points.push_back(points);
        }

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        //std::vector<cv::Mat_<double> > rotations;
        //std::vector<cv::Mat_<double> > translations;
        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;
        std::vector<Eigen::Affine3d> cam_checker_tfs;

        cv::calibrateCamera(pattern_points, image_points, lastImage_left->image.size(), cameraMatrix, distCoeffs, rotations, translations);

        if(rotations.size() != translations.size()) {
            ROS_ERROR("Number of rotations and translations differ");
            return;
        }

        if(rotations.size() <= 0) {
            ROS_ERROR("No samples taken.");
            return;
        }

        Eigen::Affine3d base_checker_tf(Eigen::Translation3d(0.276+0.0245, 0.589-0.0245, 0.004));

        int numPoints = translations.size();

        Eigen::Matrix3Xd pointsTool;
        Eigen::Matrix3Xd pointsCam;

        pointsTool.resize(3, numPoints);
        pointsCam.resize(3, numPoints);

        Eigen::Vector3d tool_points_centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d cam_points_centroid = Eigen::Vector3d::Zero();

        for(int i = 0; i < numPoints; ++i) {
            // Turn Opencv Rotation vector into a rotation matrix
            cv::Mat_<double> rotMat;
            cv::Rodrigues(rotations[i], rotMat);
            // OpenCV matrices are row major (Eigen defaults to column major)
            Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::RowMajor> > rotation((double*)rotMat.data);
            // Turn OpenCV translation vector into a translation matrix
            Eigen::Translation3d translation(
                    translations[i].at<double>(0),
                    translations[i].at<double>(1),
                    translations[i].at<double>(2));
            // We want the inverse, since opencv gives us the trasform from the camera to the checkerboard
            // and we really want the transform from the checkerboard to the camera
            Eigen::Affine3d checker_cam_tf = (translation*rotation).inverse();
            cam_checker_tfs.push_back(checker_cam_tf);
            // Compute the transform from the tool frame to the camera frame
            Eigen::Affine3d tool_camera_tf = tool_base_tfs[i]*base_checker_tf*checker_cam_tf;
            std::cout << tool_camera_tf.matrix() << std::endl << std::endl;

            pointsTool.col(i) = tool_base_tfs[i]*base_checker_tf*Eigen::Vector3d(0.0, 0.0, 0.0);
            pointsCam.col(i)  = checker_cam_tf.inverse()*Eigen::Vector3d(0.0, 0.0, 0.0);

            tool_points_centroid += pointsTool.col(i);
            cam_points_centroid += pointsCam.col(i);
        }

        tool_points_centroid /= numPoints;
        cam_points_centroid /= numPoints;

        for(int i = 0; i < numPoints; ++i) {
            pointsTool.col(i) -= tool_points_centroid;
            pointsCam.col(i) -= cam_points_centroid;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(pointsCam*pointsTool.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        double d = (svd.matrixV() * svd.matrixU().transpose()).determinant() > 0.0 ? 1.0 : -1.0;
        I(2,2) = d;

        Eigen::Matrix3d rotation = svd.matrixV() * I * svd.matrixU().transpose();

        Eigen::Vector3d color_translation(tool_points_centroid - rotation*cam_points_centroid);
        Eigen::Quaterniond color_rotation(rotation);
        Eigen::Affine3d bestTransform = Eigen::Translation3d(color_translation) * rotation;
        Eigen::Affine3d tool_camera_tf = bestTransform * color_depth_tf;
        std::cout << "\nColor Depth\n" << color_depth_tf.matrix() << std::endl;
        std::cout << "\nBest Transform\n" << bestTransform.matrix() << std::endl;
        std::cout << "\nCorrected\n" << tool_camera_tf.matrix() << std::endl;

        Eigen::Vector3d depth_translation(tool_camera_tf.translation());
        Eigen::Quaterniond depth_rotation(tool_camera_tf.rotation());
        //bestTransform.rotation() = rotation;
        //bestTransform.translation() = cam_points_centroid - rotation*tool_points_centroid;
        
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //fs << "cameraMatrix" << cameraMatrix;
        //fs << "distCoeffs" << distCoeffs;
        fs << "depth_optical_frame" << "{"
               << "position" <<"{"
                   << "x" << depth_translation.x()
                   << "y" << depth_translation.y()
                   << "z" << depth_translation.z()
               << "}"
               << "orientation" << "{"
                   << "x" << depth_rotation.x()
                   << "y" << depth_rotation.y()
                   << "z" << depth_rotation.z()
                   << "w" << depth_rotation.w()
               << "}"
           << "}";
        fs << "color_optical_frame" << "{"
               << "position" <<"{"
                   << "x" << color_translation.x()
                   << "y" << color_translation.y()
                   << "z" << color_translation.z()
               << "}"
               << "orientation" << "{"
                   << "x" << color_rotation.x()
                   << "y" << color_rotation.y()
                   << "z" << color_rotation.z()
                   << "w" << color_rotation.w()
               << "}"
           << "}";
        fs.release();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration_node");
    ros::NodeHandle nh("~");

    std::string filename;
    XmlRpc::XmlRpcValue pose_config;
    XmlRpc::XmlRpcValue color_offset;
    Eigen::Affine3d color_depth_tf;
    std::vector<geometry_msgs::Pose> poses;
    nh.getParam("camera_calib", filename);
    nh.getParam("calibration_poses", pose_config);
    nh.getParam("offset", color_offset);

    // Read in list of poses
    ROS_ASSERT(pose_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < pose_config.size(); ++i) {
        XmlRpc::XmlRpcValue pose = pose_config[i];
        ROS_ASSERT(pose.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue orientation = pose["orientation"];
        XmlRpc::XmlRpcValue position = pose["position"];
        ROS_ASSERT(orientation.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(position.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue tx = position["x"];
        ROS_ASSERT(tx.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue ty = position["y"];
        ROS_ASSERT(ty.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue tz = position["z"];
        ROS_ASSERT(tz.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue rx = orientation["x"];
        ROS_ASSERT(rx.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue ry = orientation["y"];
        ROS_ASSERT(ry.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue rz = orientation["z"];
        ROS_ASSERT(rz.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue rw = orientation["w"];
        ROS_ASSERT(rw.getType() == XmlRpc::XmlRpcValue::TypeDouble);

        geometry_msgs::Pose p;
        p.position.x = static_cast<double>(tx);
        p.position.y = static_cast<double>(ty);
        p.position.z = static_cast<double>(tz);
        p.orientation.x = static_cast<double>(rx);
        p.orientation.y = static_cast<double>(ry);
        p.orientation.z = static_cast<double>(rz);
        p.orientation.w = static_cast<double>(rw);
        poses.push_back(p);
    }

    ROS_ASSERT(color_offset.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue rows = color_offset["rows"];
    XmlRpc::XmlRpcValue cols = color_offset["cols"];
    ROS_ASSERT(rows.getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(cols.getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(static_cast<int>(rows) == 4);
    ROS_ASSERT(static_cast<int>(cols) == 4);
    XmlRpc::XmlRpcValue data = color_offset["data"];
    ROS_ASSERT(data.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(data.size() == 16);
    for(int i = 0; i < data.size(); ++i) {
        XmlRpc::XmlRpcValue val = data[i];
        ROS_ASSERT(val.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        color_depth_tf(i/4, i%4) = static_cast<double>(val);
    }

    std::cout << "Color -> Detph tf:\n" << color_depth_tf.matrix() << std::endl;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    Calibrator calibrator(nh, filename, poses, color_depth_tf);
    sleep(1);
    calibrator.calibrate();

    spinner.stop();
}
