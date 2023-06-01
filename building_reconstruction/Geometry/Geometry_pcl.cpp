#include "Geometry_pcl.h"

bool Geometry_pcl::point_in_radius(PointXYZ p, PointXYZ center, double radius)
{
    return pow(p.x - center.x, 2) + pow(p.y - center.y, 2) + pow(p.z - center.z, 2) < pow(radius, 2);
}

bool Geometry_pcl::point_in_radius(PointNormal p, PointXYZ center, double radius)
{
    return pow(p.x - center.x, 2) + pow(p.y - center.y, 2) + pow(p.z - center.z, 2) < pow(radius, 2);
}

double Geometry_pcl::euclidean_dist_between_two_points(PointXYZ a, PointXYZ b)
{
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}

double Geometry_pcl::max_euclidean_dist_side_in_polygon(std::vector<PointXYZ> ps)
{
    if (ps.size() < 3) return 0.0;
    pcl::PointXYZ p1 = ps[0];
    pcl::PointXYZ p2 = ps[1];
    pcl::PointXYZ p3 = ps[2];
    double max_dist = euclidean_dist_between_two_points(p1, p2);
    max_dist = std::max(max_dist, euclidean_dist_between_two_points(p2, p3));
    max_dist = std::max(max_dist, euclidean_dist_between_two_points(p1, p3));
    return max_dist;
}

double Geometry_pcl::triangle_area_geron(PointXYZ p1, PointXYZ p2, PointXYZ p3)
{
    double a = euclidean_dist_between_two_points(p1, p2);
    double b = euclidean_dist_between_two_points(p2, p3);
    double c = euclidean_dist_between_two_points(p1, p3);
    double p = (a+b+c) / 2;
    return sqrt(p*(p - a)*(p - b)*(p - c));
}

double Geometry_pcl::min_euclidean_dist_between_point_and_polygon_points(PointXYZ p, std::vector<PointXYZ> ps)
{
    pcl::PointXYZ start_p = *ps.begin();
    double min_dist = euclidean_dist_between_two_points(p, start_p);
    for (std::vector<pcl::PointXYZ>::iterator it = ps.begin() + 1; it < ps.end(); it++) {
        min_dist = std::min(min_dist, euclidean_dist_between_two_points(p, *it));
    }
    return min_dist;
}

