#define PCL_NO_PRECOMPILE
// ROS
#include <ros/ros.h>
// TF
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
// PCL
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
// ROS <-> OpenCV
#include <cv_bridge/cv_bridge.h>
// Image encodings
#include <sensor_msgs/image_encodings.h>
// Marker Message
#include <visualization_msgs/Marker.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// Message Filtering
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Filesystem
#include <boost/filesystem.hpp>
//
#include <limits>
#include <utility>
#include <vector>
#include <algorithm>
#include <list>
// Eigen
#include <Eigen/Geometry>
// PCL Conversions
#include <pcl_conversions/pcl_conversions.h>

#include "ObjectRecognizer.h"
#include "ColorDetector.h"
#include "apc_vision/ProcessSamples.h"
#include "util.h"
#include "ObjectIDs.h"

using namespace apc_vision;
using namespace apc_msgs;

const static std::string out_topic = "/object_segmentation/points_xyz";
const static std::string pose_topic = "/object_segmentation/pose";
const static std::string marker_topic = "/object_segmentation/bounding_boxes";
const static std::string limits_topic = "/object_segmentation/bin_limits";

const static std::string base_frame = "/base_link";
const static std::string shelf_frame = "/shelf";
const static std::string left_camera_frame = "/camera_left_depth_optical_frame";

struct ObjInfo {
    ObjInfo() {}
    ObjInfo(std::vector<std::string> calibFiles, std::vector<float> dimensions, std::vector<std::string> colors) : calibFiles(calibFiles), dimensions(dimensions), colors(colors){}

    std::vector<std::string> calibFiles;
    std::vector<float>       dimensions;
    std::vector<std::string> colors;
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
    bool valid;

    PointCluster() :
        rotation(Eigen::Matrix3f::Identity()),
        score(0.0),
        valid(false)
    {}
};

bool bestScoreComparison(const PointCluster &a, const PointCluster &b)
{
    return a.score<b.score;
}

class ProcessServer {
public:
    ProcessServer(ros::NodeHandle& nh, Config config) :
        process_server(nh.advertiseService("process_samples", &ProcessServer::process_cb, this)),
        pointcloud_pub(nh.advertise<PointCloud>(out_topic, 1)),
        pose_pub(nh.advertise<geometry_msgs::Pose>(pose_topic, 1)),
        config(config),
        marker_pub(nh.advertise<visualization_msgs::Marker>(marker_topic, 1)),
        limits_pub(nh.advertise<visualization_msgs::Marker>(limits_topic, 1))
    {
        for(Config::CalibMap::iterator it = config.calib.begin(); it != config.calib.end(); it++) {
            objDetectors[it->first] = ObjectRecognizer(it->second.calibFiles);
        }
    }

protected:
    ros::Publisher pointcloud_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_pub;
    ros::Publisher limits_pub;
    ros::ServiceServer process_server;
    tf::TransformListener listener;

    Config config;
    std::map<std::string, ObjectRecognizer> objDetectors;

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

    bool showMarkers(std::vector<PointCluster>::iterator result, std::vector<float> dims)
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


    void feature_match(std::string object, PointCloud::Ptr cloud, Sample& sample) {
        std::map<std::string, ObjectRecognizer>::iterator feature_matcher = objDetectors.find(object);
        if(feature_matcher == objDetectors.end()) {
            ROS_INFO("No feature matcher for object `%s'", object.c_str());
        } else {
            cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(sample.rgb, sensor_msgs::image_encodings::BGR8);
            ROS_DEBUG("Converting index image to OpenCV for object `%s'", object.c_str());
            cv_bridge::CvImagePtr ind_ptr = cv_bridge::toCvCopy(sample.indices, sensor_msgs::image_encodings::TYPE_32SC2);
            ROS_DEBUG("Feature matching for object `%s'", object.c_str());
            std::vector<std::vector<cv::Point2f> > obj_bounds;
            float score = feature_matcher->second.detect(img_ptr->image, obj_bounds);
            ROS_INFO("Score %f for object `%s'", score, object.c_str());

            ROS_DEBUG("Image size (%d,%d), indices size (%d,%d)", img_ptr->image.size().width, img_ptr->image.size().height, ind_ptr->image.size().width, ind_ptr->image.size().height);
            ROS_DEBUG("Object matches found: %zd", obj_bounds.size());

            for(int obj = 0; obj < obj_bounds.size(); ++obj) {
                int min_x=INT_MAX, max_x=0, min_y=INT_MAX, max_y=0;
                for(int p = 0; p < obj_bounds[obj].size(); ++p) {
                    ROS_DEBUG("Point (%f, %f)", obj_bounds[obj][p].x, obj_bounds[obj][p].y);
                    min_x = std::min(min_x, (int)obj_bounds[obj][p].x);
                    max_x = std::max(max_x, (int)obj_bounds[obj][p].x);
                    min_y = std::min(min_y, (int)obj_bounds[obj][p].y);
                    max_y = std::max(max_y, (int)obj_bounds[obj][p].y);
                }

                ROS_DEBUG("Object min/max (%d, %d)->(%d, %d)", min_x, min_y, max_x, max_y);

                min_x = (min_x * ind_ptr->image.size().width) / img_ptr->image.size().width;
                max_x = (max_x * ind_ptr->image.size().width) / img_ptr->image.size().width;
                min_y = (min_y * ind_ptr->image.size().height) / img_ptr->image.size().height;
                max_y = (max_y * ind_ptr->image.size().height) / img_ptr->image.size().height;

                ROS_DEBUG("Object min/max (%d, %d)->(%d, %d)", min_x, min_y, max_x, max_y);

                min_x = std::min(std::max(min_x, 0), ind_ptr->image.size().width-1);
                max_x = std::min(std::max(max_x, 0), ind_ptr->image.size().width-1);
                min_y = std::min(std::max(min_y, 0), ind_ptr->image.size().height-1);
                max_y = std::min(std::max(max_y, 0), ind_ptr->image.size().height-1);

                ROS_DEBUG("Object min/max (%d, %d)->(%d, %d)", min_x, min_y, max_x, max_y);

                int in_range=0, invalid=0, out_range=0;
                for(int y = min_y; y <= max_y; ++y) {
                    for(int x = min_x; x <= max_x; ++x) {
                        // if point in quadrilateral TODO: Fix this check
                        if(true) {
                            int32_t index = ind_ptr->image.at<cv::Vec2i>(y,x)[1];
                            if(index >= 0 && index < cloud->points.size()) {
                                cloud->points[index].id = string_to_id(object);
                            } else if(index == -1) {++invalid;}
                            else ++out_range;
                        }
                    }
                }

                int total = in_range+out_range+invalid;
                ROS_DEBUG("%d/%d points invalid, %d/%d points out of range", invalid, total, out_range, total);
            }
        }
    }

    // Use ICP to align two pointclouds
    void align(PointCloud::Ptr source, PointCloud::Ptr target) {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setMaximumIterations(50);
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.align(*source);

        if(icp.hasConverged()) {
            ROS_INFO("ICP converged\n");
        } else {
            ROS_WARN("ICP did not converge\n");
        }
    }

    bool process_cb(ProcessSamples::Request& request, ProcessSamples::Response& response) {
        std::string object = request.samples.order.name;
        std::vector<float> dims = config.calib[object].dimensions;
        std::vector<Sample>& samples = request.samples.samples;
        std::vector<PointCloud::Ptr> pcl_clouds(samples.size());

        for(int i = 0; i < samples.size(); ++i) {
            PointCloud::Ptr cloud(new PointCloud);
            pc2_to_pcl(samples[i].cloud, *cloud);
            pcl_clouds[i] = cloud;
        }

        PointCloud::Ptr out(new PointCloud);
        std::vector<PointCluster> clusters;
        for(int i = 0 ; i < samples.size(); ++i) {
            if(!pcl_clouds[i]->points.size()) {
                ROS_WARN("Empty pointcloud from sample %d\n", i);
                continue;
            }
            // Run feature matching
            feature_match(object, pcl_clouds[i], samples[i]);
            // Use icp to match clouds
            if(i > 0) { align(pcl_clouds[i], pcl_clouds[0]); }
            // Combine sampled pointclouds.
            *out += *pcl_clouds[i];
        }

        if(!out->points.size()) {
            ROS_ERROR("Empty combined pointcloud\n");
            response.result.status = ProcessedObject::EMPTY_POINTCLOUD;
            return true;
        }

        if(!filter(out, request.samples.order.bin)) {
            ROS_ERROR("Empty filtered pointcloud");
            response.result.status = ProcessedObject::EMPTY_POINTCLOUD;
            return true;
        }

        clusters.push_back(segmentCloud(out, object));

        if(config.calib.find(object) == config.calib.end()) {
            ROS_ERROR("Could not find calibration info for object `%s'", object.c_str());
            response.result.status = ProcessedObject::INVALID_OBJECT;
            return true;
        }

        if(config.calib[object].dimensions.size() == 0) {
            ROS_ERROR("Object dimensions nonexistent for object `%s'", object.c_str());
            response.result.status = ProcessedObject::INVALID_OBJECT;
            return true;
        }

        // Get best cluster
        std::vector<PointCluster>::iterator result = std::min_element(clusters.begin(), clusters.end(), bestScoreComparison);
        if(clusters.end() == result)
        {
            ROS_ERROR("Could not find best cluster");
            response.result.status = ProcessedObject::NO_OBJECTS_FOUND;
            return true;
        }
        else
        {
            if(!result->valid) {
                ROS_ERROR("'Best' cluster was not valid");
                response.result.status = ProcessedObject::NO_OBJECTS_FOUND;
            } else {
                response.result.status = ProcessedObject::SUCCESS;
            }

            ros::Time stamp = ros::Time::now();
            Eigen::Quaternionf quaternion(result->rotation);
            response.result.pose.position.x = result->position.x;
            response.result.pose.position.y = result->position.y;
            response.result.pose.position.z = result->position.z;
            response.result.pose.orientation.x = quaternion.x();
            response.result.pose.orientation.y = quaternion.y();
            response.result.pose.orientation.z = quaternion.z();
            response.result.pose.orientation.w = quaternion.w();

            geometry_msgs::PoseStamped posestamped;
            posestamped.header.frame_id = shelf_frame;
            posestamped.pose = response.result.pose;

            bool success = listener.waitForTransform(shelf_frame, base_frame, stamp, ros::Duration(2.0));
            // If transform isn't found in that time, give up
            if(!success) {
                ROS_ERROR("Couldn't lookup transform from shelf to base_link!");
                return true;
            }
            listener.transformPose(base_frame, posestamped, posestamped);

            response.result.pose = posestamped.pose;

            response.result.obb_extents.x = dims[0];
            response.result.obb_extents.y = dims[1];
            response.result.obb_extents.z = dims[2];

            pcl_to_pc2(*result->out, response.result.pointcloud);
            response.result.pointcloud.header.stamp = ros::Time::now();
            response.result.pointcloud.header.frame_id = shelf_frame;

            pose_pub.publish(response.result.pose);
            showMarkers(result,dims);
        }
        pointcloud_pub.publish(out);

        samples.clear();
        return true;
    }

    PointCluster segmentCloud(PointCloud::Ptr cloud, std::string object)
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
        float best_score = 0.0;

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
            // TODO: Get actual number of objects
            float featureMatchProbability = scoreFeatureMatch(object, segment, cloud, 1);

            float score = boundingBoxProbability * featureMatchProbability;
            ROS_INFO("Segment[%d] probabilities for object `%s'; box: %f, feature: %f, total: %f", i, object.c_str(), boundingBoxProbability, featureMatchProbability, score);

            if(score > best_score) {
                best_score = score;
                cluster.score = score;
                cluster.rotation = rotation;
                cluster.position = position;
                cluster.out = segment;
                cluster.valid = true;
            }

            i++;
        }

        return cluster;
    }

    float scoreFeatureMatch(
        std::string object,
        PointCloud::Ptr segment,
        PointCloud::Ptr cloud,
        int num_objs)
    {
        int num_total_points = 0, num_segment_points = 0;
        uint64_t obj_id = string_to_id(object);

        for(int i = 0; i < segment->points.size(); ++i) {
            if(segment->points[i].id == obj_id) ++num_segment_points;
        }

        for(int i = 0; i < cloud->points.size(); ++i) {
            if(cloud->points[i].id == obj_id) ++num_total_points;
        }

        if(num_total_points == 0) {
            return 1.0/num_objs;
        } else {
            return (float)num_segment_points / (float)num_total_points;
        }
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

        std::vector<std::string> colors;
        (*iter)["colors"] >> dimensions;

        config.calib[key] = ObjInfo(files, dimensions, colors);
    }

    for(cv::FileNodeIterator iter = configFile["shelf"].begin(); iter != configFile["shelf"].end(); iter++) {
        std::string key = (*iter).name();
        std::vector<float> value;

        *iter >> value;

        config.bin_limits[key] = value;
    }

    ProcessServer processor(nh, config);

    ros::spin();
}
