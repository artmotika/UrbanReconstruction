#ifndef URBANRECONSTRUCTION_SHIFT_H
#define URBANRECONSTRUCTION_SHIFT_H

#include <pcl/PCLPointCloud2.h>
#include <tuple>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;

namespace urban_rec {
    using shift_coord = std::tuple<double, double, double>;

    PointCloud<PointNormal>::Ptr shiftCoord(const PCLPointCloud2::ConstPtr input_cloud, shift_coord shift);

    PointCloud<PointNormal>::Ptr shiftCoord(const PointCloud<PointNormal>::ConstPtr input_cloud, shift_coord shift);

    PointCloud<PointXYZ>::Ptr shiftCoord(const PointCloud<PointXYZ>::ConstPtr input_cloud, shift_coord shift);
}

#endif //URBANRECONSTRUCTION_SHIFT_H
