#include <ros/ros.h>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const static std::string camera_topic = "/softkinetic_camera/points_xyz";
const static std::string out_topic = "/object_segmentation/points_xyz";

const static std::string shelf_frame = "/shelf";
const static std::string camera_frame = "/softkinetic_camera_depth_optical_frame";

class Segmenter {
public:
    Segmenter(ros::NodeHandle& nh) :
        pointcloud_sub(nh.subscribe(camera_topic, 1, &Segmenter::cloud_cb, this)),
        pointcloud_pub(nh.advertise<PointCloud>(out_topic, 1))
    {
    }

protected:
    ros::Publisher pointcloud_pub;
    ros::Subscriber pointcloud_sub;
    tf::TransformListener listener;

    void cloud_cb(const PointCloud::ConstPtr msg) {

        PointCloud::Ptr out (new PointCloud);

        tf::StampedTransform transform;
        Eigen::Affine3d affine;
        ros::Time now = ros::Time::now();

        // Wait up to 2 seconds for transform.
        bool success = listener.waitForTransform(shelf_frame, camera_frame, now, ros::Duration(2.0));
        // If transform isn't found in that time, give up
        if(!success) return;
        // Otherwise, get the transform
        listener.lookupTransform(shelf_frame, camera_frame, now, transform);
        // Get an eigen transform from the tf one
        tf::transformTFToEigen(transform, affine);
        // Transform the pointcloud
        pcl::transformPointCloud(*msg, *out, affine);

        // ------------------------------
        // Filter out bin
        // ------------------------------
        pcl::PassThrough<PointT> filter;

        filter.setInputCloud(out);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(.285, 0.580);
        filter.filter(*out);

        filter.setInputCloud(out);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(0, 0.44);
        filter.filter(*out);

        filter.setInputCloud(out);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0.035, 0.26);
        filter.filter(*out);

        /*
        filter.setInputCloud(msg);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-0.15, 0.15);
        filter.filter(*out);

        filter.setInputCloud(out);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-.14, 0.09);
        filter.filter(*out);

        filter.setInputCloud(out);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0, 0.5);
        filter.filter(*out);
        */

        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (out);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*out);

        // ------------------------------
        // Cluster extraction
        // ------------------------------

        // Creating the KdTree object for the search method of the extraction
        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (out);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.01); // 1cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (out);
        ec.extract (cluster_indices);

        int i;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                out->points[*pit].r = 255 - 50*i;
                out->points[*pit].g = 50*i;
                out->points[*pit].b = 0;
            }
            i++;
        }

        out->header.frame_id = shelf_frame;

        pointcloud_pub.publish(out);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_segmentation");
    ros::NodeHandle nh;

    Segmenter segmenter(nh);

    ros::spin();
}
