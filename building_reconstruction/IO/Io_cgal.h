#ifndef URBANRECONSTRUCTION_IO_CGAL_H
#define URBANRECONSTRUCTION_IO_CGAL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <fstream>
#include <vector>

class Io_cgal {
    public:
        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using Point = Kernel::Point_3;
        using Vector = Kernel::Vector_3;
        using PNI = boost::tuple<Point, Vector, int>;
        using Point_vector = std::vector<PNI>;
        using Surface_mesh = CGAL::Surface_mesh<Point>;
        using Point_set = CGAL::Point_set_3<Point, Vector>;
        using Point_map = CGAL::Nth_of_tuple_property_map<0, PNI>;
        using Normal_map = CGAL::Nth_of_tuple_property_map<1, PNI>;
        using Plane_index_map = CGAL::Nth_of_tuple_property_map<2, PNI>;
        void readFileToPointSet(std::string filename, Point_set *points_start);
        void saveFileFromPointSet(std::string filename, Point_set points_start);
        void readFileToPointVector(std::string filename, Point_vector *points);
        void saveFileFromPointVector(std::string filename, Surface_mesh model);
};


#endif //URBANRECONSTRUCTION_IO_CGAL_H
