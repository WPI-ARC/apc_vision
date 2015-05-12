#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "PointXYZRGB_ID.h"
#include "PointXYZI_ID.h"

//#define USE_XYZRGB_ID
#define USE_XYZI_ID

#if defined(USE_XYZRGB_ID)
typedef PointXYZRGB_ID PointT;
#elif defined(USE_XYZI_ID)
typedef PointXYZI_ID PointT;
#endif

typedef pcl::PointCloud<PointT> PointCloud;

void pc2_to_pcl(const sensor_msgs::PointCloud2& source, PointCloud& target);
void pcl_to_pc2(const PointCloud& source, sensor_msgs::PointCloud2& target);
