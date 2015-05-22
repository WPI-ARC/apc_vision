#pragma once

struct PointXYZI_ID {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint64_t feature_id;
    uint64_t color_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZI_ID,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, feature_id, feature_id)
                                   (float, color_id, color_id)
)
