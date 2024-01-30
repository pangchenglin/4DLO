#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
namespace my_pcl {

struct KITTIPointXYZT {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float timestamp;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct VeloddynePointXYZIRT {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  float time;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct AevaPointXYZIVT{
    PCL_ADD_POINT4D;
    float velocity;
    float time;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

struct PointXYZT {
  PCL_ADD_POINT4D;  /// quad-word XYZ
  float intensity;
  double timestamp;                /// laser timestamp
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZIRPYT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace my_pcl



POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::KITTIPointXYZT,           //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, timestamp, timestamp)            //
)

POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::VeloddynePointXYZIRT,           //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, intensity, intensity)  //
                                  (uint16_t, ring, ring)         //
                                  (float, time, time)            //
)

POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::OusterPointXYZIRT,              //
                                  (float, x, x)                           //
                                  (float, y, y)                           //
                                  (float, z, z)                           //
                                  (float, intensity, intensity)           //
                                  (uint32_t, t, t)                        //
                                  (uint8_t, ring, ring))               //

POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::PointXYZT,              //
                                  (float, x, x)                   //
                                  (float, y, y)                   //
                                  (float, z, z)                   //
                                  (float, intensity, intensity)   //
                                  (double, timestamp, timestamp)  //
)

POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::PointXYZIRPYT,         //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, intensity, intensity)  //
                                  (float, roll, roll)            //
                                  (float, pitch, pitch)          //
                                  (float, yaw, yaw)              //
                                  (double, time, time)           //
)

// https://github.com/PointCloudLibrary/pcl/issues/3190
POINT_CLOUD_REGISTER_POINT_STRUCT(my_pcl::AevaPointXYZIVT,           //
                                  (float, x, x)                  //
                                  (float, y, y)                  //
                                  (float, z, z)                  //
                                  (float, intensity, intensity)  //
                                  (float, velocity, velocity)  //
                                  (float, time, time)            //
)
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZ GPoint;
typedef pcl::PointCloud<GPoint> GPointCloud;

typedef my_pcl::VeloddynePointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;

typedef my_pcl::OusterPointXYZIRT OusterPoint;
typedef pcl::PointCloud<OusterPoint> OusterPointCloud;

typedef my_pcl::AevaPointXYZIVT RTVPoint;
typedef pcl::PointCloud<RTVPoint> RTVPointCloud;
typedef std::vector<RTVPoint, Eigen::aligned_allocator<RTVPoint>>  PointVector;

// typedef my_pcl::PointXYZT PosPoint;
// typedef pcl::PointCloud<PosPoint> PosCloud;

// typedef my_pcl::PointXYZIRPYT PosePoint;
// typedef pcl::PointCloud<PosePoint> PosePointCloud;

// typedef pcl::PointXYZRGB ColorPoint;
// typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
