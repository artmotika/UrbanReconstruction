#ifndef URBANRECONSTRUCTION_GEOMETRY_PCL_H
#define URBANRECONSTRUCTION_GEOMETRY_PCL_H

#include <cmath>
#include <typeinfo>
#include <pcl/impl/point_types.hpp>

using namespace pcl;

class Geometry_pcl {
    public:
        bool point_in_radius(PointXYZ p, PointXYZ center, double radius);

        bool point_in_radius(PointNormal p, PointXYZ center, double radius);

        double euclidean_dist_between_two_points(PointXYZ a, PointXYZ b);

        double max_euclidean_dist_side_in_polygon(std::vector<PointXYZ> ps);

        double triangle_area_geron(PointXYZ p1, PointXYZ p2, PointXYZ p3);

        double min_euclidean_dist_between_point_and_polygon_points(PointXYZ p, std::vector<PointXYZ> ps);

};


#endif //URBANRECONSTRUCTION_GEOMETRY_PCL_H
