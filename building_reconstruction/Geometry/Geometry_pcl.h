#ifndef URBANRECONSTRUCTION_GEOMETRY_PCL_H
#define URBANRECONSTRUCTION_GEOMETRY_PCL_H

#include <cmath>
#include <typeinfo>

using namespace pcl;

class Geometry_pcl {
    public:
        double euclidean_dist_between_two_points(pcl::PointXYZ a, pcl::PointXYZ b)
        {
            return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
        }

        double max_euclidean_dist_side_in_polygon(std::vector<pcl::PointXYZ> ps)
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

        double triangle_area_geron(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
        {
            double a = euclidean_dist_between_two_points(p1, p2);
            double b = euclidean_dist_between_two_points(p2, p3);
            double c = euclidean_dist_between_two_points(p1, p3);
            double p = (a+b+c) / 2;
            return sqrt(p*(p - a)*(p - b)*(p - c));
        }

        double min_euclidean_dist_between_point_and_polygon_points(pcl::PointXYZ p, std::vector<pcl::PointXYZ> ps)
        {
            pcl::PointXYZ start_p = *ps.begin();
            double min_dist = euclidean_dist_between_two_points(p, start_p);
            for (std::vector<pcl::PointXYZ>::iterator it = ps.begin() + 1; it < ps.end(); it++) {
                min_dist = std::min(min_dist, euclidean_dist_between_two_points(p, *it));
            }
            return min_dist;
        }

};


#endif //URBANRECONSTRUCTION_GEOMETRY_PCL_H
