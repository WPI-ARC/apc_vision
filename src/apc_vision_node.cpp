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
struct BestCluster{
    Eigen::Matrix3f rotation;
    PointT position;
    float score;
    PointCloud::Ptr out;
};
bool bestScoreComparison(const BestCluster &a, const BestCluster &b)
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
        left_index_image_sub(nh.subscribe(right_index_image_topic, 1, &VisionProcessor::left_index_image_cb, this)),
        right_index_image_sub(nh.subscribe(right_index_image_topic, 1, &VisionProcessor::right_index_image_cb, this)),
        left_sync(pointcloud_sub_left, uv_sub_left, 10),
        right_sync(pointcloud_sub_right, uv_sub_right, 10),
        pointcloud_pub(nh.advertise<PointCloud>(out_topic, 1)),
        pose_pub(nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1)),
        sample_server(nh.advertiseService("sample_vision", &VisionProcessor::sample_cb, this)),
        process_server(nh.advertiseService("process_vision", &VisionProcessor::process_cb, this)),
        config(config),
        marker_pub(nh.advertise<visualization_msgs::Marker>(marker_topic, 1)),
        limits_pub(nh.advertise<visualization_msgs::Marker>(limits_topic, 1))
    {
        left_sync.registerCallback(boost::bind(&VisionProcessor::process_left, this, _1, _2));
        right_sync.registerCallback(boost::bind(&VisionProcessor::process_right, this, _1, _2));

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
    ros::Subscriber left_index_image_sub;
    ros::Subscriber right_index_image_sub;

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> left_sync;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> right_sync;

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

    std::vector<PointCloud::Ptr> samples;

    void left_image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastImage_left = img;
    }

    void right_image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastImage_right = img;
    }

    void left_index_image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastIndexImage_left = img;
    }

    void right_index_image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastIndexImage_right = img;
    }

    void process_left(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, 
                 const sensor_msgs::PointCloud2::ConstPtr& uv_msg) {
        lastPC_left = pc_msg;
        lastUVC_left = uv_msg;
    }

    void process_right(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, const sensor_msgs::PointCloud2::ConstPtr& uv_msg) {
        lastPC_right = pc_msg;
        lastUVC_right = uv_msg;
    }

    bool sample_cb(SampleVision::Request& request, SampleVision::Response& response) {
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
            if(!lastPC.get()) return false;
            if(!lastUVC.get()) return false;

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
                std::cerr << "Couldn't lookup transform!" << std::endl;
                return false;
            }
            // Otherwise, get the transform
            listener.lookupTransform(shelf_frame, camera_frame, stamp, transform);
            // Get an eigen transform from the tf one
            tf::transformTFToEigen(transform, affine);
            // Transform the pointcloud
            pcl::transformPointCloud(*out, *out, affine);

            std::map<std::string, ObjectRecognizer>::iterator feature_matcher = objDetectors.find(request.command);
            if(feature_matcher == objDetectors.end()) {
                ROS_INFO("No feature matcher for object `%s'", request.command.c_str());
            } else {
                cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8);
                cv_bridge::CvImagePtr ind_ptr = cv_bridge::toCvCopy(lastIndexImage, sensor_msgs::image_encodings::TYPE_32SC1);
                std::vector<std::vector<cv::Point2f> > obj_bounds;
                float score = feature_matcher->second.detect(img_ptr->image, obj_bounds);
                ROS_INFO("Score %f for object `%s'", score, request.command.c_str());

                for(int obj = 0; obj < obj_bounds.size(); ++obj) {
                    int min_x=INT_MAX, max_x=0, min_y=INT_MAX, max_y=0;
                    for(int i = 0; i < obj_bounds[obj].size(); ++i) {
                        min_x = std::min(min_x, (int)obj_bounds[obj][i].x);
                        max_x = std::min(max_x, (int)obj_bounds[obj][i].x);
                        min_y = std::min(min_y, (int)obj_bounds[obj][i].y);
                        max_y = std::min(max_y, (int)obj_bounds[obj][i].y);
                    }

                    for(int j = min_y; j <= max_y; ++j) {
                        for(int i = min_x; j <= max_x; ++i) {
                            // if point in quadrilateral TODO: Fix this check
                            if(true) {
                                int32_t index = ind_ptr->image.at<int32_t>(j,i);
                                if(index >= 0) {
                                    out->points[index].id = string_to_id(request.command);
                                    out->points[index].r = 0;
                                    out->points[index].g = 0;
                                    out->points[index].b = 255;
                                }
                            }
                        }
                    }
                }
            }

            /*
            Eigen::Vector3d camera_heading = affine * Eigen::Vector3d(0.0, 0.0, 1.0).homogeneous();
            Eigen::Vector3d camera_origin = affine * Eigen::Vector3d(0.0, 0.0, 0.0).homogeneous();
            camera_heading = camera_heading - camera_origin;
            */

            // ------------------------------
            // Filter out bin
            // ------------------------------
            pcl::PassThrough<PointT> filter;
            //pcl::IndicesPtr indices(new std::vector<int>);

            if(config.bin_limits.find(request.command) == config.bin_limits.end()) return false;
            std::vector<float>& limits = config.bin_limits[request.command];

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

            ROS_INFO("Bin limits: x: [%f, %f], y: [%f, %f], z: [%f, %f]\n", limits[0], limits[1], limits[2], limits[3], limits[4], limits[5]);

            ROS_INFO("%lu points before filtering\n", out->points.size());

            filter.setInputCloud(out);
            filter.setFilterFieldName("x");
            filter.setFilterLimits(limits[0], limits[1]);
            //filter.filter(*indices);
            filter.filter(*out);

            ROS_INFO("%lu points left after filtering x\n", out->points.size());

            filter.setInputCloud(out);
            //filter.setIndices(indices);
            filter.setFilterFieldName("y");
            filter.setFilterLimits(limits[2], limits[3]);
            //filter.filter(*indices);
            filter.filter(*out);
            ROS_INFO("%lu points left after filtering y\n", out->points.size());

            filter.setInputCloud(out);
            //filter.setIndices(indices);
            filter.setFilterFieldName("z");
            filter.setFilterLimits(limits[4], limits[5]);
            //filter.filter(*indices);
            filter.filter(*out);
            ROS_INFO("%lu points left after filtering z\n", out->points.size());

            // ------------------------------------
            // Filter out statistical outliers
            // ------------------------------------
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud (out);
            //sor.setIndices(indices);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            //sor.filter (*indices);
            sor.filter(*out);

            ROS_INFO("%lu points left after SOR filter\n", out->points.size());

            // Use icp to match clouds
            //if(samples.size() > 0) {
			if(0){
                pcl::IterativeClosestPoint<PointT, PointT> icp;
                icp.setMaximumIterations(50);
                icp.setInputSource(out);
                icp.setInputTarget(samples[0]);
                PointCloud::Ptr p(new PointCloud);
                icp.align(*p);
                if(icp.hasConverged()) {
                    out = p;
                }
            }
			//pointcloud_pub.publish(out);

            samples.push_back(out);

            return true;
        } else {
            samples.clear();
            return true;
        }
    }
    bool showMarkers(std::list<BestCluster>::iterator result, std::vector<float> dims)
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

        std::string object = request.target.name;
        std::vector<float> dims = config.calib[object].dimensions;


        // Combine sampled pointclouds.
        PointCloud::Ptr out(new PointCloud);
        std::list<BestCluster> bestCluster;
        std::list<BestCluster>::iterator it=bestCluster.begin();
        for(int i = 0 ; i < samples.size(); i++, it++) {
            if(!samples[i]->points.size()) {
                ROS_WARN("Empty pointcloud from sample %d\n", i);
                continue;
            }
            bestCluster.push_back(extractClusters(samples[i], object));
            showMarkers(it, dims);
            pointcloud_pub.publish(samples[i]);
            *out += *samples[i];
            ros::Duration(2).sleep(); 
        }

        if(!out->points.size()) {
            ROS_ERROR("Empty combined pointcloud\n");
            return false;
        }

        bestCluster.push_back(extractClusters(out, object));

        if(config.calib.find(object) == config.calib.end()) return false;
        if(config.calib[object].dimensions.size() == 0) return false;

        // Get bestCluster
        std::list<BestCluster>::iterator result = std::min_element(bestCluster.begin(), bestCluster.end(), bestScoreComparison);
        if(bestCluster.end()==result)
        {
            return false;
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
                ROS_WARN("Couldn't lookup transform from shelf to base_link!");
                return false;
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

    BestCluster extractClusters(PointCloud::Ptr out, std::string object)
    {
        // ------------------------------
        // Cluster extraction
        // ------------------------------

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        //tree->setInputCloud (out, indices);
        tree->setInputCloud (out);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.01); // 1cm
        ec.setMinClusterSize (100);
        //ec.setMaxClusterSize (250000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (out);
        //ec.setIndices(indices);
        ec.extract (cluster_indices);

        //visualization_msgs::MarkerArray array_msg;

        BestCluster bestCluster;
        float best_score = 1.0;

        int i = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PointCloud::Ptr segment(new PointCloud);
            pcl::PointIndices::Ptr indices_(new pcl::PointIndices);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                //out->points[*pit].r = 0;
                //out->points[*pit].g = 200-i*50;
                //out->points[*pit].b = i*50;
                indices_->indices.push_back(*pit);
                segment->points.push_back(out->points[*pit]);
            }

            pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
            feature_extractor.setInputCloud(out);
            feature_extractor.setIndices(indices_);
            feature_extractor.compute();

            PointT minPoint, maxPoint, position;
            Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
            Eigen::Vector3f mass_center;

            feature_extractor.getOBB(minPoint, maxPoint, position, rotation);
            feature_extractor.getMassCenter(mass_center);

            std::vector<float> dims;
            dims.push_back(maxPoint.x - minPoint.x);
            dims.push_back(maxPoint.y - minPoint.y);
            dims.push_back(maxPoint.z - minPoint.z);


            float score = detectCuboidPose(object, position, rotation, dims);

            if(score < best_score) {
                best_score = score;
                bestCluster.score = score;
                bestCluster.rotation = rotation;
                bestCluster.position = position;
                bestCluster.out = segment;
            }

            i++;
        }

        out->header.frame_id = shelf_frame;

        return bestCluster;
    }

    float detectCuboidPose(
        // Object name
        std::string object,
        // Position and orientation from OBB
        PointT position, 
        Eigen::Matrix3f& rotation,
        // OBB size
        std::vector<float>& sensed_dims
        // Direction camera is pointing
        //Eigen::Vector3f camera_vector
    ) {
        Eigen::Vector3f x(1.0, 0.0, 0.0);
        Eigen::Vector3f y(0.0, 1.0, 0.0);
        Eigen::Vector3f z(0.0, 0.0, 1.0);
        x = rotation*x;
        y = rotation*y;
        z = rotation*z;

        //float xdot = std::abs(x.dot(camera_vector.cast<float>()));
        //float ydot = std::abs(y.dot(camera_vector.cast<float>()));
        //float zdot = std::abs(z.dot(camera_vector.cast<float>()));

        float best_score = 1.0;
        Eigen::Matrix3f best_rotation = Eigen::Matrix3f::Identity();
        Eigen::Vector3f offset(0.0, 0.0, 0.0);

        //float dot_threshold = 0.93;

        if(config.calib.find(object) == config.calib.end()) return false;
        if(config.calib[object].dimensions.size() == 0) return false;

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
                    /*
                    int num = 0;
                    if(xdot < dot_threshold) {
                        score += std::abs(sensed_dims[0] - known_dims[i]);
                        num++;
                    }
                    if(ydot < dot_threshold) {
                        score += std::abs(sensed_dims[1] - known_dims[j]);
                        num++;
                    }
                    if(zdot < dot_threshold) {
                        score += std::abs(sensed_dims[2] - known_dims[k]);
                        num++;
                    }
                    score /= num;
                    */
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

                    /*
                    if(xdot >= dot_threshold) {
                        float sign = xdot/x.dot(camera_heading.cast<float>());
                        offset = x*sign*(known_dims[i]/2 - measured_dims[0]/2);
                    }
                    if(ydot >= dot_threshold) {
                        float sign = ydot/y.dot(camera_heading.cast<float>());
                        offset = y*sign*(known_dims[j]/2 - measured_dims[1]/2);
                    }
                    if(zdot >= dot_threshold) {
                        float sign = zdot/z.dot(camera_heading.cast<float>());
                        offset = z*sign*(known_dims[k]/2 - measured_dims[2]/2);
                    }
                    */
                }
            }
        }

        sensed_dims = known_dims;//new_dims;
        rotation = best_rotation * rotation;
        //position += offset;
        return best_score;
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
