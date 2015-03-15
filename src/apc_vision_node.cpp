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

#include <iostream>

#include "ObjectRecognizer.h"
#include "apc_vision/ObjectDetect.h"

bool close_to(float x, float y, float tolerance_lower, float tolerance_upper) {
    return (x >= y-tolerance_lower) && (x <= y+tolerance_upper);
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::PointUV> UVCloud;

const static std::string xyz_topic = "/senz3d/points_xyzrgb";
const static std::string uv_topic = "/senz3d/points_uv";
const static std::string image_topic = "/senz3d/color";
const static std::string out_topic = "/object_segmentation/points_xyz";
const static std::string pose_topic = "/object_segmentation/pose";
const static std::string marker_topic = "/object_segmentation/bounding_boxes";

const static std::string shelf_frame = "/shelf";
const static std::string camera_frame = "/senz3d_depth_optical_frame";
const static std::string base_frame = "/base_link";

struct ObjInfo {
    ObjInfo() {}
    ObjInfo(std::vector<std::string> calibFiles) : calibFiles(calibFiles) {}

    std::vector<std::string> calibFiles;
};

struct Config {
    typedef std::map<std::string, ObjInfo> CalibMap;
    CalibMap calib;

    typedef std::map<std::string, std::vector<float> > BinLimitsMap;
    BinLimitsMap bin_limits;
};

class Segmenter {
public:
    Segmenter(ros::NodeHandle& nh, std::string obj, Config config) :
        pointcloud_sub(nh, xyz_topic, 1),
        uv_sub(nh, uv_topic, 1),
        image_sub(nh.subscribe(image_topic, 1, &Segmenter::image_cb, this)),
        sync(pointcloud_sub, uv_sub, 10),
        pointcloud_pub(nh.advertise<PointCloud>(out_topic, 1)),
        pose_pub(nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1)),
        server(nh.advertiseService("object_detect", &Segmenter::service_cb, this)),
        config(config),
        marker_pub(nh.advertise<visualization_msgs::Marker>(marker_topic, 1))
    {
        sync.registerCallback(boost::bind(&Segmenter::process, this, _1, _2));

        for(Config::CalibMap::iterator it = config.calib.begin(); it != config.calib.end(); it++) {
            objDetectors[it->first] = ObjectRecognizer(it->second.calibFiles);
        }
    }

protected:
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> uv_sub;
    ros::Subscriber image_sub;

    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync;

    ros::Publisher pointcloud_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_pub;
    ros::ServiceServer server;

    tf::TransformListener listener;

    sensor_msgs::ImageConstPtr lastImage;

    Config config;
    std::map<std::string, ObjectRecognizer> objDetectors;

    sensor_msgs::PointCloud2::ConstPtr lastPC;
    sensor_msgs::PointCloud2::ConstPtr lastUVC;

    void image_cb(const sensor_msgs::ImageConstPtr& img) {
        lastImage = img;
    }

    void process(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, 
                 const sensor_msgs::PointCloud2::ConstPtr& uv_msg) {
        lastPC = pc_msg;
        lastUVC = uv_msg;
    }

    bool service_cb(apc_vision::ObjectDetect::Request& request, apc_vision::ObjectDetect::Response& response) {
        response.found = false;

        if(!lastPC.get()) return true;
        if(!lastUVC.get()) return true;

        pcl::PCLPointCloud2 uv_pc2, pc_pc2;

        ros::Time stamp = lastPC->header.stamp;

        pcl_conversions::toPCL(*lastPC,pc_pc2);
        pcl_conversions::toPCL(*lastUVC,uv_pc2);

        PointCloud::Ptr out(new PointCloud);
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
            return true;
        }
        // Otherwise, get the transform
        listener.lookupTransform(shelf_frame, camera_frame, stamp, transform);
        // Get an eigen transform from the tf one
        tf::transformTFToEigen(transform, affine);
        // Transform the pointcloud
        pcl::transformPointCloud(*out, *out, affine);

        // ------------------------------
        // Filter out bin
        // ------------------------------
        pcl::PassThrough<PointT> filter;
        pcl::IndicesPtr indices(new std::vector<int>);

        if(config.bin_limits.find(request.bin) == config.bin_limits.end()) return true;
        std::vector<float>& limits = config.bin_limits[request.bin];
        visualization_msgs::Marker marker;
        marker.header.frame_id = shelf_frame;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (limits[0]+limits[1])/2;
        marker.pose.position.y = (limits[2]+limits[3])/2;
        marker.pose.position.z = (limits[4]+limits[5])/2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = limits[1]-limits[0];
        marker.scale.y = limits[3]-limits[2];
        marker.scale.z = limits[5]-limits[4];
        marker.color.a = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_pub.publish(marker);

        filter.setInputCloud(out);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(limits[0], limits[1]);
        filter.filter(*indices);

        filter.setInputCloud(out);
        filter.setIndices(indices);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(limits[2], limits[3]);
        filter.filter(*indices);

        filter.setInputCloud(out);
        filter.setIndices(indices);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(limits[4], limits[5]);
        filter.filter(*indices);

        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (out);
        sor.setIndices(indices);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*indices);

        // ------------------------------
        // Cluster extraction
        // ------------------------------

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (out, indices);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.01); // 1cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (out);
        ec.setIndices(indices);
        ec.extract (cluster_indices);

        //visualization_msgs::MarkerArray array_msg;

        int i = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointIndices::Ptr indices_(new pcl::PointIndices);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                out->points[*pit].r = 255 - 50*i;
                out->points[*pit].g = 50*i;
                out->points[*pit].b = 0;
                indices_->indices.push_back(*pit);
            }

            float min_u = 1.0;
            float min_v = 1.0;
            float max_u = 0.0;
            float max_v = 0.0;

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                pcl::PointUV img_coord = uvcloud->points[*pit];
                min_u = std::min(img_coord.u, min_u);
                max_u = std::max(img_coord.u, max_u);
                min_v = std::min(img_coord.v, min_v);
                max_v = std::max(img_coord.v, max_v);
            }

            if(lastImage.get()) {
                int w = lastImage->width;
                int h = lastImage->height;
                int min_x = min_u*w;
                int max_x = max_u*w;
                int min_y = min_v*h;
                int max_y = max_v*h;
                
                cv_bridge::CvImagePtr img_ptr; 
         
                try { 
                    img_ptr = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::BGR8); 
                } catch(cv_bridge::Exception& e) {
                    ROS_ERROR("Failed to extract opencv image; cv_bridge exception: %s", e.what());
                    exit(-1);
                } 

                if(img_ptr.get()) {
                    if(objDetectors.find(request.object) == objDetectors.end()) {
                        std::cerr << "Couldn't find detector for specified object!" << std::endl;
                        return true;
                    }

                    float score = objDetectors[request.object].detect(
                        img_ptr->image.rowRange(min_y, max_y)
                                      .colRange(min_x, max_x)
                    );
                    if(score > 0) {
                        pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
                        feature_extractor.setInputCloud(out);
                        feature_extractor.setIndices(indices_);
                        feature_extractor.compute();

                        PointT minPoint, maxPoint, position;
                        Eigen::Matrix3f rotation;
                        Eigen::Vector3f mass_center;

                        feature_extractor.getOBB(minPoint, maxPoint, position, rotation);
                        feature_extractor.getMassCenter(mass_center);


                        response.found = true;
                        response.pose.header.stamp = stamp;
                        response.pose.header.frame_id = shelf_frame;
                        Eigen::Quaternionf quaternion(rotation);
                        response.pose.pose.position.x = position.x;
                        response.pose.pose.position.y = position.y;
                        response.pose.pose.position.z = position.z;
                        response.pose.pose.orientation.x = -0.12384;//quaternion.x();
                        response.pose.pose.orientation.y = 0.0841883;//quaternion.y();
                        response.pose.pose.orientation.z = -0.730178;//quaternion.z();
                        response.pose.pose.orientation.w = 0.666646;//quaternion.w();
                        listener.transformPose(base_frame, response.pose, response.pose);
                        pose_pub.publish(response.pose);
                        //response.pose.pose.position.x = x;
                        //response.pose.pose.position.y = y;
                        //response.pose.pose.position.z = z;
                        //response.pose.pose.orientation.x = 0.0;
                        //response.pose.pose.orientation.y = 0.0;
                        //response.pose.pose.orientation.z = 0.0;
                        //response.pose.pose.orientation.w = 1.0;
                    }
                }
            }

            /*
            Eigen::Quaternionf quaternion(rotation);

            visualization_msgs::Marker marker;

            marker.header.frame_id = shelf_frame;
            marker.header.stamp = ros::Time(0);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = position.x;
            marker.pose.position.y = position.y;
            marker.pose.position.z = position.z;
            marker.pose.orientation.x = quaternion.x();
            marker.pose.orientation.y = quaternion.y();
            marker.pose.orientation.z = quaternion.z();
            marker.pose.orientation.w = quaternion.w();
            marker.scale.x = maxPoint.x-minPoint.x;
            marker.scale.y = maxPoint.y-minPoint.y;
            marker.scale.z = maxPoint.z-minPoint.z;
            marker.color.r = (255 - 50*i)/255.0;
            marker.color.g = (50*i)/255.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            array_msg.markers.push_back(marker);

            float xdim = maxPoint.x - minPoint.x;
            float ydim = maxPoint.y - minPoint.y;
            float zdim = maxPoint.z - minPoint.z;
            */

            /*
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<PointT> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_SPHERE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setRadiusLimits(.03, .04);
            seg.setDistanceThreshold (0.005);

            //seg.setInputCloud(cluster_cloud);
            seg.setInputCloud (out);
            seg.setIndices(indices);
            seg.segment (*inliers, *coefficients);

            for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit) {
                out->points[*pit].r = 255;
                out->points[*pit].g = 255;
                out->points[*pit].b = 255;
                //cluster_cloud->points.push_back(out->points[*pit]);
            }

            printf("%d: %d\n", i, inliers->indices.size());
            */

            i++;
        }

        out->header.frame_id = shelf_frame;

        //marker_pub.publish(array_msg);
        pointcloud_pub.publish(out);
        return true;
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
        std::vector<std::string> value;

        *iter >> value;

        for(int i = 0; i < value.size(); i++) {
            boost::filesystem::path image_path(value[i]);
            // If the path to an image isn't absolute, assume it's
            // relative to where the config file is
            if(!image_path.is_absolute()) 
                value[i] = (config_path.parent_path() / value[i]).string();
        }


        config.calib[key] = ObjInfo(value);
    }

    for(cv::FileNodeIterator iter = configFile["shelf"].begin(); iter != configFile["shelf"].end(); iter++) {
        std::string key = (*iter).name();
        std::vector<float> value;
        
        *iter >> value;

        config.bin_limits[key] = value;
    }

    Segmenter segmenter(nh, obj, config);

    ros::spin();
}
