#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/filesystem.hpp>

#include <limits>

#include <iostream>
#include <utility>
#include <vector>
#include <algorithm>    // std::sort
#include <list>

#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>

#include "apc_vision/SampleVision.h"
#include "apc_vision/Sample.h"

using namespace apc_vision;

bool close_to(float x, float y, float tolerance_lower, float tolerance_upper) {
    return (x >= y-tolerance_lower) && (x <= y+tolerance_upper);
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::PointUV> UVCloud;

const static std::string left_xyz_topic = "/camera_left/points_xyzrgb";
const static std::string left_uv_topic = "/camera_left/points_uv";
const static std::string left_image_topic = "/camera_left/color";
const static std::string out_topic = "/object_segmentation/points_xyz";
const static std::string pose_topic = "/object_segmentation/pose";
const static std::string marker_topic = "/object_segmentation/bounding_boxes";
const static std::string limits_topic = "/object_segmentation/bin_limits";

const static std::string shelf_frame = "/shelf";
const static std::string left_camera_frame = "/camera_left_depth_optical_frame";
const static std::string base_frame = "/base_link";

class ScannerNode {
public:
    ScannerNode(ros::NodeHandle& nh) :
        pointcloud_sub_left(nh, left_xyz_topic, 1),
        uv_sub_left(nh, left_uv_topic, 1),
        left_image_sub(nh.subscribe(left_image_topic, 1, &ScannerNode::left_image_cb, this)),
        left_sync(pointcloud_sub_left, uv_sub_left, 10),
        sample_server(nh.advertiseService("sample_vision", &ScannerNode::sample_cb, this)),
        limits_pub(nh.advertise<visualization_msgs::Marker>(limits_topic, 1))
    {
        left_sync.registerCallback(boost::bind(&ScannerNode::process_left, this, _1, _2));
    }

protected:
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_left;
    message_filters::Subscriber<sensor_msgs::PointCloud2> uv_sub_left;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> left_sync;
    ros::Subscriber left_image_sub;

    ros::Publisher limits_pub;
    ros::ServiceServer sample_server;
    tf::TransformListener listener;

    sensor_msgs::ImageConstPtr lastImage_left;
    sensor_msgs::PointCloud2::ConstPtr lastPC_left;
    sensor_msgs::PointCloud2::ConstPtr lastUVC_left;

    std::map<std::string, std::vector<Sample> > samples;

    void left_image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastImage_left = img;
    }

    void process_left(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, 
                 const sensor_msgs::PointCloud2::ConstPtr& uv_msg) {
        lastPC_left = pc_msg;
        lastUVC_left = uv_msg;
    }

    bool sample_cb(SampleVision::Request& request, SampleVision::Response& response) {
        if (request.command == "reset") {
            samples.clear();
            return true;
        }

        sensor_msgs::PointCloud2::ConstPtr lastPC = lastPC_left, lastUVC= lastUVC_left;
        if (!lastPC.get() or !lastUVC.get()) {
          ROS_ERROR("Null pointer with lastPC or lastUVC is the camera enabled");
            return false;
        }

        pcl::PCLPointCloud2 uv_pc2, pc_pc2;
        pcl_conversions::toPCL(*lastPC, pc_pc2);
        pcl_conversions::toPCL(*lastUVC, uv_pc2);

        PointCloud::Ptr out(new PointCloud);
        PointCloud::Ptr pubcloud(new PointCloud);
        pubcloud->header.frame_id=shelf_frame;
        UVCloud::Ptr uvcloud(new UVCloud);

        pcl::fromPCLPointCloud2(pc_pc2, *out);
        pcl::fromPCLPointCloud2(uv_pc2, *uvcloud);

        // --------------------------------
        // Transform cloud to shelf-space
        // --------------------------------
        tf::StampedTransform transform;
        tf::StampedTransform base_transform;
        Eigen::Affine3d affine;

        // Wait up to 2 seconds for transform.
        ros::Time stamp = lastPC->header.stamp;
        bool success = listener.waitForTransform(shelf_frame, left_camera_frame, stamp, ros::Duration(2.0));
        // If transform isn't found in that time, give up
        if (!success) {
          ROS_ERROR("Couldn't lookup transform!");
          return false;
        }
        // Otherwise, get the transform
        listener.lookupTransform(shelf_frame, left_camera_frame, stamp, transform);
        // Get an eigen transform from the tf one
        tf::transformTFToEigen(transform, affine);
        // Transform the pointcloud
        pcl::transformPointCloud(*out, *out, affine);

        Sample sample;
        // TODO: sample.transform = ;
        // TODO: sample.image = ;
        pcl::PCLPointCloud2 pointcloud;
        pcl::toPCLPointCloud2(*out, pointcloud);
        pcl_conversions::fromPCL(pointcloud, sample.pointcloud);
        // TODO: sample.index = ;
        samples[request.command].push_back(sample);

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scanner_node");
    ros::NodeHandle nh;

    ScannerNode processor(nh);

    ros::spin();
}
