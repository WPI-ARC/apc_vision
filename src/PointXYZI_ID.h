#pragma once

struct PointXYZI_ID {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint64_t id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZI_ID,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, id, id)
)
