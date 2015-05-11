#include "util.h"

void pc2_to_pcl(const sensor_msgs::PointCloud2& source, PointCloud& target) {
    pcl::PCLPointCloud2 pc_pc2;
    pcl_conversions::toPCL(source,pc_pc2);
    pcl::fromPCLPointCloud2(pc_pc2, target);
}

void pcl_to_pc2(const PointCloud& source, sensor_msgs::PointCloud2& target) {
    pcl::PCLPointCloud2 pc_pc2;
    pcl::toPCLPointCloud2(source, pc_pc2);
    pcl_conversions::fromPCL(pc_pc2, target);
}
