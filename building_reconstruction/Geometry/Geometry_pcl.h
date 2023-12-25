#ifndef URBANRECONSTRUCTION_GEOMETRY_PCL_H
#define URBANRECONSTRUCTION_GEOMETRY_PCL_H

#include <cmath>
#include <typeinfo>
#include <pcl/impl/point_types.hpp>
#include <Eigen/Dense>

using namespace pcl;

namespace Geometry_pcl {
    bool point_in_radius(PointXYZ p, PointXYZ center, double radius);

    bool point_in_radius(PointNormal p, PointXYZ center, double radius);

    double euclidean_dist_between_two_points(PointXYZ a, PointXYZ b);

    double euclidean_dist_between_two_points(PointXYZRGB a, PointXYZRGB b);

    double max_euclidean_dist_side_in_polygon(std::vector <PointXYZ> ps);

    double triangle_area_geron(PointXYZ p1, PointXYZ p2, PointXYZ p3);

    double min_euclidean_dist_between_point_and_polygon_points(PointXYZ p, std::vector <PointXYZ> ps);

    bool check_point_in_triangle(Eigen::Vector2i p, Eigen::Vector2i p1, Eigen::Vector2i p2, Eigen::Vector2i p3);

    bool check_point_in_triangle_vector(Eigen::Vector2i p, Eigen::Vector2i p1, Eigen::Vector2i p2, Eigen::Vector2i p3);

    double dist_between_two_points(Eigen::Vector2i p1, Eigen::Vector2i p2);

}


#endif //URBANRECONSTRUCTION_GEOMETRY_PCL_H
