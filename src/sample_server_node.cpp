// Ros include
#include <ros/ros.h>
// TF includes
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
// PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// Message Filtering Includes
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Eigen
#include <Eigen/Geometry>
// Service Description Includes
#include "apc_msgs/Sample.h"
#include "apc_vision/TakeSample.h"
#include "apc_vision/GetSamples.h"
// Util
#include "util.h"

using namespace apc_msgs;
using namespace apc_vision;

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

class SampleServer {
public:
    SampleServer(ros::NodeHandle& nh) :
        pointcloud_sub_left(nh, left_xyz_topic, 1),
        left_image_sub(nh.subscribe(left_image_topic, 1, &SampleServer::left_image_cb, this)),
        left_index_image_sub(nh, left_index_image_topic, 1),
        left_sync(pointcloud_sub_left, left_index_image_sub, 10),
        take_sample_server(nh.advertiseService("take_sample", &SampleServer::take_sample_cb, this)),
        get_samples_server(nh.advertiseService("get_samples", &SampleServer::get_samples_cb, this))
    {
        left_sync.registerCallback(boost::bind(&SampleServer::process_left, this, _1, _2));

    }

protected:
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_left;
    message_filters::Subscriber<sensor_msgs::Image> left_index_image_sub;

    message_filters::TimeSynchronizer<
        sensor_msgs::PointCloud2, sensor_msgs::Image
    > left_sync;

    ros::Subscriber left_image_sub;

    ros::ServiceServer take_sample_server;
    ros::ServiceServer get_samples_server;

    tf::TransformListener listener;

    sensor_msgs::PointCloud2::ConstPtr lastPC_left;
    sensor_msgs::ImageConstPtr lastIndexImage_left;
    sensor_msgs::ImageConstPtr lastImage_left;

    std::map<std::string, std::vector<Sample> > samples;

    void left_image_cb(const sensor_msgs::ImageConstPtr& image) {
        lastImage_left = image;
    }

    void process_left(
        const sensor_msgs::PointCloud2::ConstPtr& pc_msg,
        const sensor_msgs::ImageConstPtr& index_image)
    {
        lastPC_left = pc_msg;
        lastIndexImage_left = index_image;
    }


    bool take_sample_cb(TakeSample::Request& request, TakeSample::Response& response) {
        sensor_msgs::PointCloud2::ConstPtr lastPC = lastPC_left;
        sensor_msgs::ImageConstPtr lastImage = lastImage_left;
        sensor_msgs::ImageConstPtr lastIndexImage = lastIndexImage_left;
        std::string camera_frame = left_camera_frame;

        if(request.command == "reset") {
            samples[request.bin].clear();
            response.status = TakeSample::Response::SUCCESS;
        } else if(request.command == "sample") {
            if(!lastPC.get()) {
                response.status = TakeSample::Response::NO_DATA;
                ROS_ERROR("No pointclouds have been recieved!");
                return true;
            }

            PointCloud::Ptr out(new PointCloud);
            pc2_to_pcl(*lastPC, *out);

            ros::Time stamp = lastPC->header.stamp;

            // --------------------------------
            // Transform cloud to shelf-space
            // --------------------------------
            tf::StampedTransform transform;
            tf::StampedTransform base_transform;
            Eigen::Affine3d affine;

            // Wait up to 2 seconds for transform.
            bool success = listener.waitForTransform(shelf_frame, camera_frame, stamp, ros::Duration(2.0));
            // If transform isn't found in that time, give up
            if(!success) {
                response.status = TakeSample::Response::NO_TRANSFORM;
                ROS_ERROR("Couldn't look up camera to shelf transform!");
                return true;
            }
            // Otherwise, get the transform
            listener.lookupTransform(shelf_frame, camera_frame, stamp, transform);
            // Get an eigen transform from the tf one
            tf::transformTFToEigen(transform, affine);
            // Transform the pointcloud
            pcl::transformPointCloud(*out, *out, affine);

            Sample sample;
            pcl_to_pc2(*out, sample.cloud);
            // sample.transform = ...?;
            sample.rgb = *lastImage;
            sample.indices = *lastIndexImage;

            samples[request.bin].push_back(sample);
            response.status = TakeSample::Response::SUCCESS;
        } else {
            response.status = TakeSample::Response::INVALID_COMMAND;
        }

        return true;
    }

    bool get_samples_cb(GetSamples::Request& request, GetSamples::Response& response) {
        response.samples = samples[request.bin];
        response.status = GetSamples::Response::SUCCESS;

        if(response.samples.size() == 0) {
            ROS_WARN("No samples have been taken for bin `%s'", request.bin.c_str());
        }

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sample_server_node");
    ros::NodeHandle nh;

    SampleServer server(nh);

    ros::spin();
}
