#ifndef URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H
#define URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h> // since Cgal 5.6 changed
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/Timer.h>

#include <CGAL/Point_set_3.h>

#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <cstdlib>
#include <vector>

#include "IO/Io_cgal.h"
#include "Index_map.h"

#ifdef CGAL_USE_SCIP  // defined (or not) by CMake scripts, do not define by hand
#include <CGAL/SCIP_mixed_integer_program_traits.h>
#elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand
#include <CGAL/GLPK_mixed_integer_program_traits.h>
#endif

#if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)

class Surface_reconstruction {
    public:
        #ifdef CGAL_USE_SCIP  // defined (or not) by CMake scripts, do not define by hand
        using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;
        #elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand
        using MIP_Solver = CGAL::GLPK_mixed_integer_program_traits<double>;
        #endif
        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using FT = Kernel::FT;
        using Point = Kernel::Point_3;
        using Vector = Kernel::Vector_3;
        // Point with normal, and plane index.
        using PNI = boost::tuple<Point, Vector, int>;
        using Point_vector = std::vector<PNI>;

        using Point_map = CGAL::Nth_of_tuple_property_map<0, PNI>;
        using Normal_map = CGAL::Nth_of_tuple_property_map<1, PNI>;
        using Plane_index_map = CGAL::Nth_of_tuple_property_map<2, PNI>;

        using Neighbor_query = CGAL::Shape_detection::Point_set::
        Sphere_neighbor_query<Kernel, Point_vector, Point_map>;
        using Region_type = CGAL::Shape_detection::Point_set::
        Least_squares_plane_fit_region<Kernel, Point_vector, Point_map, Normal_map>;
        using Region_growing = CGAL::Shape_detection::
        Region_growing<Point_vector, Neighbor_query, Region_type>;

        using Surface_mesh = CGAL::Surface_mesh<Point>;
        using Polygonal_surface_reconstruction = CGAL::Polygonal_surface_reconstruction<Kernel>;

        using Point_set = CGAL::Point_set_3<Point, Vector>;

        void setInputFile(std::string file_name);
        std::string getInputFile();
        void setOutputFile(std::string file_name);
        std::string getOutputFile();
        void setShapeDetectionType(int type);
        int getShapeDetectionType();
        void setSimplificationAverageSpacing(int average_spacing);
        int getSimplificationAverageSpacing();
        void setEstimateNormalsNeighbors(int neighbors_number);
        int getEstimateNormalsNeighbors();
        void setSearchSphereRadius(FT radius);
        FT getSearchSphereRadius();
        void setMaxDistanceToPlane(FT distance);
        FT getMaxDistanceToPlane();
        void setMaxAcceptedAngle(FT angle);
        FT getMaxAcceptedAngle();
        void setMinRegionSize(std::size_t min_size);
        std::size_t getMinRegionSize();

        Surface_mesh reconstruct();

    private:
        std::string input_file;
        std::string output_file;
        int shape_detection_type = 0;
        int simplification_average_spacing = 200;
        int estimate_normals_neighbors = 6;
        FT search_sphere_radius = FT(2) / FT(2.3);
        FT max_distance_to_plane = FT(2) / FT(80);
        FT max_accepted_angle = FT(25);
        std::size_t min_region_size = 40;
        void simplifyMesh(Point_set *points);
        void estimateNormals(Point_set *points);
        Surface_mesh reconstructRegionGrowing(Point_vector points);
};

#endif  // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
#endif //URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H
