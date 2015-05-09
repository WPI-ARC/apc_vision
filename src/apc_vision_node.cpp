#define PCL_NO_PRECOMPILE
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

#include "PointXYZRGB_ID.h"
#include "ObjectIDs.h"

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

#include "ObjectRecognizer.h"
#include "apc_vision/ProcessVision.h"
#include "apc_vision/SampleVision.h"

using namespace apc_vision;

bool close_to(float x, float y, float tolerance_lower, float tolerance_upper) {
    return (x >= y-tolerance_lower) && (x <= y+tolerance_upper);
}

typedef PointXYZRGB_ID PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::PointUV> UVCloud;

const static std::string right_xyz_topic = "/camera_right/points_xyzrgb";
const static std::string right_uv_topic = "/camera_right/points_uv";
const static std::string right_image_topic = "/camera_right/color";
const static std::string right_index_image_topic = "/camera_right/depth_indices";
const static std::string left_xyz_topic = "/camera_left/points_xyzrgb";
const static std::string left_uv_topic = "/camera_left/points_uv";
const static std::string left_image_topic = "/camera_left/color";
const static std::string left_index_image_topic = "/camera_left/depth_indices";
const static std::string out_topic = "/object_segmentation/points_xyz";
const static std::string pose_topic = "/object_segmentation/pose";
const static std::string marker_topic = "/object_segmentation/bounding_boxes";
const static std::string limits_topic = "/object_segmentation/bin_limits";

const static std::string shelf_frame = "/shelf";
const static std::string right_camera_frame = "/camera_right_depth_optical_frame";
const static std::string left_camera_frame = "/camera_left_depth_optical_frame";
const static std::string base_frame = "/base_link";

struct Sample {
    PointCloud::Ptr cloud;
    sensor_msgs::ImageConstPtr image;
    sensor_msgs::ImageConstPtr indices;
};

struct ObjInfo {
    ObjInfo() {}
    ObjInfo(std::vector<std::string> calibFiles, std::vector<float> dimensions) : calibFiles(calibFiles), dimensions(dimensions) {}

    std::vector<std::string> calibFiles;
    std::vector<float>       dimensions;
};

struct Config {
    typedef std::map<std::string, ObjInfo> CalibMap;
    CalibMap calib;

    typedef std::map<std::string, std::vector<float> > BinLimitsMap;
    BinLimitsMap bin_limits;
};

struct PointCluster {
    Eigen::Matrix3f rotation;
    PointT position;
    float score;
    PointCloud::Ptr out;
};

bool bestScoreComparison(const PointCluster &a, const PointCluster &b)
{
    return a.score<b.score;
}

class VisionProcessor {
public:
    VisionProcessor(ros::NodeHandle& nh, std::string obj, Config config) :
        pointcloud_sub_left(nh, left_xyz_topic, 1),
        pointcloud_sub_right(nh, right_xyz_topic, 1),
        uv_sub_left(nh, left_uv_topic, 1),
        uv_sub_right(nh, right_uv_topic, 1),
        left_image_sub(nh.subscribe(left_image_topic, 1, &VisionProcessor::left_image_cb, this)),
        right_image_sub(nh.subscribe(right_image_topic, 1, &VisionProcessor::right_image_cb, this)),
        left_index_image_sub(nh, left_index_image_topic, 1),
        right_index_image_sub(nh, right_index_image_topic, 1),
        left_sync(pointcloud_sub_left, uv_sub_left, left_index_image_sub, 10),
        right_sync(pointcloud_sub_right, uv_sub_right, right_index_image_sub, 10),
        pointcloud_pub(nh.advertise<PointCloud>(out_topic, 1)),
        pose_pub(nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1)),
        sample_server(nh.advertiseService("sample_vision", &VisionProcessor::sample_cb, this)),
        process_server(nh.advertiseService("process_vision", &VisionProcessor::process_cb, this)),
        config(config),
        marker_pub(nh.advertise<visualization_msgs::Marker>(marker_topic, 1)),
        limits_pub(nh.advertise<visualization_msgs::Marker>(limits_topic, 1))
    {
        left_sync.registerCallback(boost::bind(&VisionProcessor::process_left, this, _1, _2, _3));
        right_sync.registerCallback(boost::bind(&VisionProcessor::process_right, this, _1, _2, _3));

        for(Config::CalibMap::iterator it = config.calib.begin(); it != config.calib.end(); it++) {
            objDetectors[it->first] = ObjectRecognizer(it->second.calibFiles);
        }
    }

protected:
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_left;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_right;

    message_filters::Subscriber<sensor_msgs::PointCloud2> uv_sub_left;
    message_filters::Subscriber<sensor_msgs::PointCloud2> uv_sub_right;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> left_index_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> right_index_image_sub;

    message_filters::TimeSynchronizer<
        sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image
    > left_sync;

    message_filters::TimeSynchronizer<
        sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image
    > right_sync;

    ros::Publisher pointcloud_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_pub;
    ros::Publisher limits_pub;
    ros::ServiceServer sample_server;
    ros::ServiceServer process_server;

    tf::TransformListener listener;

    sensor_msgs::ImageConstPtr lastImage_left;
    sensor_msgs::ImageConstPtr lastImage_right;
    sensor_msgs::ImageConstPtr lastIndexImage_left;
    sensor_msgs::ImageConstPtr lastIndexImage_right;

    Config config;
    std::map<std::string, ObjectRecognizer> objDetectors;

    sensor_msgs::PointCloud2::ConstPtr lastPC_left;
    sensor_msgs::PointCloud2::ConstPtr lastUVC_left;
    sensor_msgs::PointCloud2::ConstPtr lastPC_right;
    sensor_msgs::PointCloud2::ConstPtr lastUVC_right;

    std::vector<Sample> samples;

    void left_image_cb(const sensor_msgs::ImageConstPtr& image) {
        lastImage_left = image;
    }

    void right_image_cb(const sensor_msgs::ImageConstPtr& image) {
        lastImage_right = image;
    }

    void process_left(
        const sensor_msgs::PointCloud2::ConstPtr& pc_msg, 
        const sensor_msgs::PointCloud2::ConstPtr& uv_msg, 
        const sensor_msgs::ImageConstPtr& index_image) 
    {
        lastPC_left = pc_msg;
        lastUVC_left = uv_msg;
        lastIndexImage_left = index_image;
    }

    void process_right(
        const sensor_msgs::PointCloud2::ConstPtr& pc_msg,
        const sensor_msgs::PointCloud2::ConstPtr& uv_msg,
        const sensor_msgs::ImageConstPtr& index_image) 
    {
        lastPC_right = pc_msg;
        lastUVC_right = uv_msg;
        lastIndexImage_right = index_image;
    }

    bool filter(PointCloud::Ptr cloud, std::string bin) {
        // ------------------------------
        // Filter out bin
        // ------------------------------
        pcl::PassThrough<PointT> filter;

        if(config.bin_limits.find(bin) == config.bin_limits.end()) {
            ROS_ERROR("Could not find bin limits for bin `%s'", bin.c_str());
            return false;
        }
        std::vector<float>& limits = config.bin_limits[bin];

        visualization_msgs::Marker marker;
        marker.header.frame_id = shelf_frame;
        marker.header.stamp = ros::Time(0);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (limits[0]+limits[1])/2;
        marker.pose.position.y = (limits[2]+limits[3])/2;
        marker.pose.position.z = (limits[4]+limits[5])/2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = limits[1]-limits[0];
        marker.scale.y = limits[3]-limits[2];
        marker.scale.z = limits[5]-limits[4];
        marker.color.r = 40.0;
        marker.color.g = 230.0;
        marker.color.b = 40.0;
        marker.color.a = 0.7;
        limits_pub.publish(marker);

        ROS_INFO("Bin '%s' limits: x: [%f, %f], y: [%f, %f], z: [%f, %f]\n", bin.c_str(), limits[0], limits[1], limits[2], limits[3], limits[4], limits[5]);

        ROS_INFO("%zd points before filtering\n", cloud->points.size());

        filter.setInputCloud(cloud);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(limits[0], limits[1]);
        //filter.filter(*indices);
        filter.filter(*cloud);

        ROS_INFO("%zd points left after filtering x\n", cloud->points.size());

        filter.setInputCloud(cloud);
        //filter.setIndices(indices);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(limits[2], limits[3]);
        //filter.filter(*indices);
        filter.filter(*cloud);
        ROS_INFO("%zd points left after filtering y\n", cloud->points.size());

        filter.setInputCloud(cloud);
        //filter.setIndices(indices);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(limits[4], limits[5]);
        //filter.filter(*indices);
        filter.filter(*cloud);
        ROS_INFO("%zd points left after filtering z\n", cloud->points.size());

        // ------------------------------------
        // Filter out statistical outliers
        // ------------------------------------
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (cloud);
        //sor.setIndices(indices);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        //sor.filter (*indices);
        sor.filter(*cloud);
        ROS_INFO("%zd points left after SOR filtering\n", cloud->points.size());

        if(!cloud->points.size()) {
            ROS_ERROR("Empty filtered point cloud!");
            return false;
        }

        return true;
    }

    bool sample_cb(SampleVision::Request& request, SampleVision::Response& response) {
        response.success = false;

        sensor_msgs::PointCloud2::ConstPtr lastPC;
        sensor_msgs::PointCloud2::ConstPtr lastUVC;
        sensor_msgs::ImageConstPtr lastImage;
        sensor_msgs::ImageConstPtr lastIndexImage;
        std::string camera_frame;

        if(request.camera == SampleVision::Request::LEFT) {
            lastPC = lastPC_left;
            lastUVC = lastUVC_left;
            lastImage = lastImage_left;
            lastIndexImage = lastIndexImage_left;
            camera_frame = left_camera_frame;
        } else {
            lastPC = lastPC_right;
            lastUVC = lastUVC_right;
            lastImage = lastImage_right;
            lastIndexImage = lastIndexImage_right;
            camera_frame = right_camera_frame;
        }

        if(request.command != "reset") {
            if(!lastPC.get()) {
                ROS_ERROR("No pointclouds have been recieved!");
                return true;
            }
            if(!lastUVC.get()) {
                ROS_ERROR("No UV pointclouds have been recieved!");
                return true;
            }

            pcl::PCLPointCloud2 uv_pc2, pc_pc2;

            ros::Time stamp = lastPC->header.stamp;

            pcl_conversions::toPCL(*lastPC,pc_pc2);
            pcl_conversions::toPCL(*lastUVC,uv_pc2);

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
            bool success = listener.waitForTransform(shelf_frame, camera_frame, stamp, ros::Duration(2.0));
            // If transform isn't found in that time, give up
            if(!success) {
                ROS_ERROR("Couldn't look up camera to shelf transform!");
                return true;
            }
            // Otherwise, get the transform
            listener.lookupTransform(shelf_frame, camera_frame, stamp, transform);
            // Get an eigen transform from the tf one
            tf::transformTFToEigen(transform, affine);
            // Transform the pointcloud
            pcl::transformPointCloud(*out, *out, affine);

            /*
            Eigen::Vector3d camera_heading = affine * Eigen::Vector3d(0.0, 0.0, 1.0).homogeneous();
            Eigen::Vector3d camera_origin = affine * Eigen::Vector3d(0.0, 0.0, 0.0).homogeneous();
            camera_heading = camera_heading - camera_origin;
            */

            Sample sample;
            sample.cloud = out;
            sample.image = lastImage;
            sample.indices = lastIndexImage;

            samples.push_back(sample);

            response.success = true;

            return true;
        } else {
            response.success = true;
            samples.clear();
            return true;
        }
    }

    bool showMarkers(std::list<PointCluster>::iterator result, std::vector<float> dims)
    {
        Eigen::Quaternionf quaternion(result->rotation);
        visualization_msgs::Marker marker;
        marker.header.frame_id = shelf_frame;
        marker.header.stamp = ros::Time(0);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = result->position.x;
        marker.pose.position.y = result->position.y;
        marker.pose.position.z = result->position.z;
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        marker.scale.x = dims[0];
        marker.scale.y = dims[1];
        marker.scale.z = dims[2];
        marker.color.r = 40.0;
        marker.color.g = 230.0;
        marker.color.b = 40.0;
        marker.color.a = 0.7;
        marker_pub.publish(marker);
    }

    bool process_cb(ProcessVision::Request& request, ProcessVision::Response& response) {
        response.valid = false;

        std::string object = request.target.name;
        std::vector<float> dims = config.calib[object].dimensions;


        // Combine sampled pointclouds.
        PointCloud::Ptr out(new PointCloud);
        std::list<PointCluster> clusters;
        std::list<PointCluster>::iterator it=clusters.begin();
        for(int i = 0 ; i < samples.size(); i++, it++) {
            if(!samples[i].cloud->points.size()) {
                ROS_WARN("Empty pointcloud from sample %d\n", i);
                continue;
            }

            // --------------------------
            // Feature Matching
            // --------------------------
            std::map<std::string, ObjectRecognizer>::iterator feature_matcher = objDetectors.find(request.target.name);
            if(feature_matcher == objDetectors.end()) {
                ROS_INFO("No feature matcher for object `%s'", request.target.name.c_str());
            } else {
                ROS_DEBUG("Converting rgb image to OpenCV for object `%s'", request.target.name.c_str());
                cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(samples[i].image, sensor_msgs::image_encodings::BGR8);
                ROS_DEBUG("Converting index image to OpenCV for object `%s'", request.target.name.c_str());
                cv_bridge::CvImagePtr ind_ptr = cv_bridge::toCvCopy(samples[i].indices, sensor_msgs::image_encodings::TYPE_32SC1);
                ROS_DEBUG("Feature matching for object `%s'", request.target.name.c_str());
                std::vector<std::vector<cv::Point2f> > obj_bounds;
                float score = feature_matcher->second.detect(img_ptr->image, obj_bounds);
                ROS_INFO("Score %f for object `%s'", score, request.target.name.c_str());

                ROS_DEBUG("Image size (%d,%d), indices size (%d,%d)", img_ptr->image.size().width, img_ptr->image.size().height, ind_ptr->image.size().width, ind_ptr->image.size().height);
                ROS_DEBUG("Object matches found: %zd", obj_bounds.size());

                for(int obj = 0; obj < obj_bounds.size(); ++obj) {
                    int min_x=INT_MAX, max_x=0, min_y=INT_MAX, max_y=0;
                    for(int p = 0; p < obj_bounds[obj].size(); ++p) {
                        min_x = std::min(min_x, (int)obj_bounds[obj][p].x);
                        max_x = std::max(max_x, (int)obj_bounds[obj][p].x);
                        min_y = std::min(min_y, (int)obj_bounds[obj][p].y);
                        max_y = std::max(max_y, (int)obj_bounds[obj][p].y);
                    }

                    min_x = (min_x * ind_ptr->image.size().width) / img_ptr->image.size().width;
                    max_x = (max_x * ind_ptr->image.size().width) / img_ptr->image.size().width;
                    min_y = (min_y * ind_ptr->image.size().height) / img_ptr->image.size().height;
                    max_y = (max_y * ind_ptr->image.size().height) / img_ptr->image.size().height;

                    min_x = std::min(std::max(min_x, 0), ind_ptr->image.size().width-1);
                    max_x = std::min(std::max(max_x, 0), ind_ptr->image.size().width-1);
                    min_y = std::min(std::max(min_y, 0), ind_ptr->image.size().height-1);
                    max_y = std::min(std::max(max_y, 0), ind_ptr->image.size().height-1);

                    for(int y = min_y; y <= max_y; ++y) {
                        for(int x = min_x; x <= max_x; ++x) {
                            // if point in quadrilateral TODO: Fix this check
                            if(true) {
                                int32_t index = ind_ptr->image.at<int32_t>(y,x);
                                if(index >= 0 && index < samples[i].cloud->points.size()) {
                                    samples[i].cloud->points[index].id = string_to_id(request.target.name);
                                    samples[i].cloud->points[index].r = 0;
                                    samples[i].cloud->points[index].g = 0;
                                    samples[i].cloud->points[index].b = 255;
                                }
                            }
                        }
                    }
                }
            }

            // Use icp to match clouds
            if(i > 0) {
                pcl::IterativeClosestPoint<PointT, PointT> icp;
                icp.setMaximumIterations(50);
                icp.setInputSource(samples[i].cloud);
                icp.setInputTarget(samples[0].cloud);
                icp.align(*samples[i].cloud);

                if(icp.hasConverged()) {
                    ROS_INFO("ICP converged\n");
                }
            }

            *out += *samples[i].cloud;
        }

        if(!out->points.size()) {
            ROS_ERROR("Empty combined pointcloud\n");
            return true;
        }

        if(!filter(out, request.bin)) {
            ROS_ERROR("Empty filtered pointcloud");
            return true;
        }

        // -----------------
        // Clustering
        // -----------------
        clusters.push_back(extractClusters(out, object));

        if(config.calib.find(object) == config.calib.end()) {
            ROS_ERROR("Could not find calibration info for object `%s'", object.c_str());
            return true;
        }

        if(config.calib[object].dimensions.size() == 0) {
            ROS_ERROR("Object dimensions nonexistent for object `%s'", object.c_str());
            return true;
        }

        // Get best cluster
        std::list<PointCluster>::iterator result = std::min_element(clusters.begin(), clusters.end(), bestScoreComparison);
        if(clusters.end()==result)
        {
            ROS_ERROR("Could not find best cluster");
            return true;
        }
        else
        {
            ros::Time stamp = ros::Time::now();//lastPC->header.stamp;
            response.pose.header.stamp = stamp;
            response.pose.header.frame_id = shelf_frame;
            Eigen::Quaternionf quaternion(result->rotation);
            response.pose.pose.position.x = result->position.x;
            response.pose.pose.position.y = result->position.y;
            response.pose.pose.position.z = result->position.z;
            response.pose.pose.orientation.x = quaternion.x();
            response.pose.pose.orientation.y = quaternion.y();
            response.pose.pose.orientation.z = quaternion.z();
            response.pose.pose.orientation.w = quaternion.w();
            
            bool success = listener.waitForTransform(shelf_frame, base_frame, stamp, ros::Duration(2.0));
            // If transform isn't found in that time, give up
            if(!success) {
                ROS_ERROR("Couldn't lookup transform from shelf to base_link!");
                return true;
            }
            listener.transformPose(base_frame, response.pose, response.pose);

            response.size.x = dims[0];
            response.size.y = dims[1];
            response.size.z = dims[2];

            pcl::PCLPointCloud2 pclpc2;
            pcl::toPCLPointCloud2(*result->out, pclpc2);
            pcl_conversions::fromPCL(pclpc2,response.object_points);
            response.object_points.header.stamp = ros::Time::now();
            response.object_points.header.frame_id = shelf_frame;

            pose_pub.publish(response.pose);
            showMarkers(result,dims);
        }
        pointcloud_pub.publish(out);

        samples.clear();
        //pointcloud_pub.publish(pubcloud);
        return true;
    }

    PointCluster extractClusters(PointCloud::Ptr cloud, std::string object)
    {
        // ------------------------------
        // Cluster extraction
        // ------------------------------

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.01); // 1cm
        ec.setMinClusterSize (100);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);


        PointCluster cluster;
        float best_score = 1.0;

        int i = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            // Create a new pointcloud with just the points of this segment
            PointCloud::Ptr segment(new PointCloud);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                segment->points.push_back(cloud->points[*pit]);
            }

            PointT position;
            Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
            float boundingBoxProbability = scoreBoundingBox(object, segment, position, rotation);
            float featureMatchProbability = scoreFeatureMatch(object, segment, cloud);

            float score = boundingBoxProbability * featureMatchProbability;
            ROS_INFO("Segment[%d] probabilities for object `%s'; box: %f, feature: %f, total: %f", i, object.c_str(), boundingBoxProbability, featureMatchProbability, score);

            if(score < best_score) {
                best_score = score;
                cluster.score = score;
                cluster.rotation = rotation;
                cluster.position = position;
                cluster.out = segment;
            }

            i++;
        }

        return cluster;
    }

    float scoreFeatureMatch(
        std::string object,
        PointCloud::Ptr segment,
        PointCloud::Ptr cloud)
    {
        int num_total_points = 0, num_segment_points = 0;
        uint64_t obj_id = string_to_id(object);

        for(int i = 0; i < segment->points.size(); ++i) {
            if(segment->points[i].id == obj_id) ++num_segment_points;
        }

        for(int i = 0; i < cloud->points.size(); ++i) {
            if(cloud->points[i].id == obj_id) ++num_total_points;
        }

        return (float)num_segment_points / (float)num_total_points;
    }

    float scoreBoundingBox(
        // Object name
        std::string object,
        // Pointcloud segment
        PointCloud::Ptr segment,
        PointT& position,
        Eigen::Matrix3f& rotation)
    {
        // Estimate OBB
        pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
        feature_extractor.setInputCloud(segment);
        feature_extractor.compute();

        PointT minPoint, maxPoint;

        feature_extractor.getOBB(minPoint, maxPoint, position, rotation);

        std::vector<float> sensed_dims;
        sensed_dims.push_back(maxPoint.x - minPoint.x);
        sensed_dims.push_back(maxPoint.y - minPoint.y);
        sensed_dims.push_back(maxPoint.z - minPoint.z);

        Eigen::Vector3f x(1.0, 0.0, 0.0);
        Eigen::Vector3f y(0.0, 1.0, 0.0);
        Eigen::Vector3f z(0.0, 0.0, 1.0);
        x = rotation*x;
        y = rotation*y;
        z = rotation*z;

        float best_score = 1.0;
        Eigen::Matrix3f best_rotation = Eigen::Matrix3f::Identity();
        Eigen::Vector3f offset(0.0, 0.0, 0.0);

        if(config.calib.find(object) == config.calib.end()) {
            ROS_ERROR("Could not find object calibration info for object `%s'", object.c_str());
            return false;
        }
        if(config.calib[object].dimensions.size() == 0) {
            ROS_ERROR("Object dimensions nonexistent for object `%s'", object.c_str());
            return false;
        }

        std::vector<float>& known_dims = config.calib[object].dimensions;
        std::vector<float> new_dims = known_dims;

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                if(i == j) continue;
                for(int k = 0; k < 3; k++) {
                    if(k == i || k == j) continue;
                    float score = 0.0; 
                    score += std::abs(sensed_dims[0] - known_dims[i]);
                    score += std::abs(sensed_dims[1] - known_dims[j]);
                    score += std::abs(sensed_dims[2] - known_dims[k]);

                    if(score < best_score) {
                        best_score = score;
                        best_rotation = Eigen::Matrix3f::Identity();
                        if(i == 0 && j == 1 && k == 2) {
                            new_dims = known_dims;
                        } else if(i == 0 && j == 2 && k == 1) {
                            best_rotation = Eigen::AngleAxisf(0.5*M_PI, x);
                            new_dims[0] = known_dims[0];
                            new_dims[1] = known_dims[2];
                            new_dims[2] = known_dims[1];
                        } else if(i == 1 && j == 0 && k == 2) {
                            best_rotation = Eigen::AngleAxisf(0.5*M_PI, z);
                            new_dims[0] = known_dims[1];
                            new_dims[1] = known_dims[0];
                            new_dims[2] = known_dims[2];
                        } else if(i == 1 && j == 2 && k == 0) {
                            best_rotation = Eigen::AngleAxisf(0.5*M_PI, y) * Eigen::AngleAxisf(0.5*M_PI, x);
                            new_dims[1] = known_dims[0];
                            new_dims[2] = known_dims[1];
                            new_dims[0] = known_dims[2];
                        } else if(i == 2 && j == 0 && k == 1) {
                            best_rotation = Eigen::AngleAxisf(0.5*M_PI, z) * Eigen::AngleAxisf(0.5*M_PI, y);
                            new_dims[2] = known_dims[0];
                            new_dims[0] = known_dims[1];
                            new_dims[1] = known_dims[2];
                        } else if(i == 2 && j == 1 && k == 0) {
                            best_rotation = Eigen::AngleAxisf(0.5*M_PI, y);
                            new_dims[0] = known_dims[2];
                            new_dims[1] = known_dims[1];
                            new_dims[2] = known_dims[0];
                        }
                    }
                }
            }
        }

        sensed_dims = known_dims;
        rotation = best_rotation * rotation;

        // Convert 'score' to a 'probability'
        // This is a rather arbitrary conversion
        return 2.0 - 2.0/(1 + std::exp(-7.0*best_score));
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_segmentation");
    ros::NodeHandle nh;

    if(argc <= 1) {
        printf("Please provide config file location");
        return -1;
    }

    std::string obj = argv[2];

    boost::filesystem::path config_path(argv[1]);

    std::cout << boost::filesystem::absolute(config_path).string() << std::endl;

    if(!boost::filesystem::exists(config_path) || !boost::filesystem::is_regular_file(config_path)) {
        std::cout << "Specified config file does not exist" << std::endl;
        return -1;
    }

    cv::FileStorage configFile(config_path.string(), cv::FileStorage::READ);
    Config config;

    for(cv::FileNodeIterator iter = configFile["objects"].begin(); iter != configFile["objects"].end(); iter++) {
        std::string key = (*iter).name();
        std::vector<std::string> files;

        (*iter)["files"] >> files;

        for(int i = 0; i < files.size(); i++) {
            boost::filesystem::path image_path(files[i]);
            // If the path to an image isn't absolute, assume it's
            // relative to where the config file is
            if(!image_path.is_absolute()) 
                files[i] = (config_path.parent_path() / files[i]).string();
        }

        std::vector<float> dimensions;
        (*iter)["size"] >> dimensions;

        config.calib[key] = ObjInfo(files, dimensions);
    }

    for(cv::FileNodeIterator iter = configFile["shelf"].begin(); iter != configFile["shelf"].end(); iter++) {
        std::string key = (*iter).name();
        std::vector<float> value;

        *iter >> value;

        config.bin_limits[key] = value;
    }

    VisionProcessor processor(nh, obj, config);

    ros::spin();
}
