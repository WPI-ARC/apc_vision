#pragma once

struct PointXYZRGB_ID {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    uint64_t id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGB_ID,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, id, id)
)
