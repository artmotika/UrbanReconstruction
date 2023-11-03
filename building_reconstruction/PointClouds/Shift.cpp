#include "Shift.h"

using namespace pcl;

PointCloud<PointNormal>::Ptr urban_rec::shiftCoord(PCLPointCloud2::Ptr input_cloud, urban_rec::shift_coord shift) {
    PointCloud<PointNormal>::Ptr new_vertices(new PointCloud <PointNormal>);

    PointCloud<PointNormal>::Ptr all_vertices(new PointCloud <PointNormal>);
    fromPCLPointCloud2(*input_cloud, *all_vertices);
    for (PointNormal p: *all_vertices) {
        p.x = p.x + std::get<0>(shift);
        p.y = p.y + std::get<1>(shift);
        p.z = p.z + std::get<2>(shift);
        new_vertices->push_back(p);
    }
    return new_vertices;
}

PointCloud<PointNormal>::Ptr
urban_rec::shiftCoord(PointCloud<PointNormal>::Ptr input_cloud, urban_rec::shift_coord shift) {
    PointCloud<PointNormal>::Ptr new_vertices(new PointCloud <PointNormal>);

    for (PointNormal p: *input_cloud) {
        p.x = p.x + std::get<0>(shift);
        p.y = p.y + std::get<1>(shift);
        p.z = p.z + std::get<2>(shift);
        new_vertices->push_back(p);
    }
    return new_vertices;
}

PointCloud<PointXYZ>::Ptr urban_rec::shiftCoord(PointCloud<PointXYZ>::Ptr input_cloud, urban_rec::shift_coord shift) {
    PointCloud<PointXYZ>::Ptr new_vertices(new PointCloud <PointXYZ>);

    for (PointXYZ p: *input_cloud) {
        p.x = p.x + std::get<0>(shift);
        p.y = p.y + std::get<1>(shift);
        p.z = p.z + std::get<2>(shift);
        new_vertices->push_back(p);
    }
    return new_vertices;
}