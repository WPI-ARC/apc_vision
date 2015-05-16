#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "PointXYZRGB_ID.h"
#include "PointXYZI_ID.h"
#include "apc_msgs/Sample.h"
#include <fstream>
#include <new>

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

std::string get_time_str();

template<typename T>
bool serialze_msg(T& sample, std::string filename) {
    std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
    if(!out) {
        ROS_ERROR("Failed to open file `%s' for writing", filename.c_str());
        return false;
    }

    uint32_t numBytes = ros::serialization::serializationLength(sample);
    uint8_t* buffer = new (std::nothrow) uint8_t[numBytes];

    if(!buffer) {
        ROS_ERROR("Failed to allocate buffer for serializing!");
        return false;
    }

    ros::serialization::OStream stream(buffer, numBytes);
    ros::serialization::serialize(stream, sample);
    out.write((char*)buffer, numBytes);
    out.close();
    delete[] buffer;
    return true;
}

template<typename T>
bool deserialize_msg(T& sample, std::string filename) {
    std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
    if(!in) {
        ROS_ERROR("Failed to open file `%s' for reading", filename.c_str());
        return false;
    }

    in.seekg(0, in.end);
    int len = in.tellg();
    in.seekg(0, in.beg);

    uint8_t* buffer = new (std::nothrow) uint8_t[len];
    if(!buffer) {
        ROS_ERROR("Failed to allocate buffer for deserializing!");
        return false;
    }

    in.read((char*) buffer, len);
    ros::serialization::IStream stream(buffer, len);
    ros::serialization::deserialize(stream, sample);

    in.close();
    delete[] buffer;

    return true;
}
