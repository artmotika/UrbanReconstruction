#ifndef URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H
#define URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
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

        void setInputFile(std::string file_name) {
            input_file = file_name;
        }
        std::string getInputFile() {
            return input_file;
        }
        void setOutputFile(std::string file_name) {
            output_file = file_name;
        }
        std::string getOutputFile() {
            return output_file;
        }
        void setShapeDetectionType(int type) {
            shape_detection_type = type;
        }
        int getShapeDetectionType() {
            return shape_detection_type;
        }
        void setSimplificationAverageSpacing(int average_spacing) {
            simplification_average_spacing = average_spacing;
        }
        int getSimplificationAverageSpacing() {
            return simplification_average_spacing;
        }
        void setEstimateNormalsNeighbors(int neighbors_number) {
            estimate_normals_neighbors = neighbors_number;
        }
        int getEstimateNormalsNeighbors() {
            return estimate_normals_neighbors;
        }
        void setSearchSphereRadius(FT radius) {
            search_sphere_radius = radius;
        }
        FT getSearchSphereRadius() {
            return search_sphere_radius;
        }
        void setMaxDistanceToPlane(FT distance) {
            max_distance_to_plane = distance;
        }
        FT getMaxDistanceToPlane() {
            return max_distance_to_plane;
        }
        void setMaxAcceptedAngle(FT angle) {
            max_accepted_angle = angle;
        }
        FT getMaxAcceptedAngle() {
            return max_accepted_angle;
        }
        void setMinRegionSize(std::size_t min_size) {
            min_region_size = min_size;
        }
        std::size_t getMinRegionSize() {
            return min_region_size;
        }

        Surface_mesh reconstruct() {
            Io_cgal IO;
            CGAL::Timer t;
            t.start();
            Point_set points_start;
            Point_vector points;
            if (simplification_average_spacing > 0 && estimate_normals_neighbors > 0) {
                IO.readFileToPointSet(input_file, &points_start);
                switch(simplification_average_spacing) {
                    case 0:
                        break;
                    default:
                        simplifyMesh(&points_start);
                        break;
                }
                switch(estimate_normals_neighbors) {
                    case 0:
                        break;
                    default:
                        estimateNormals(&points_start);
                        break;
                }
                IO.saveFileFromPointSet(output_file, points_start);
                IO.readFileToPointVector(output_file, &points);
            } else {
                IO.readFileToPointVector(input_file, &points);
            }

            Surface_mesh surface_mesh;
            switch(shape_detection_type) {
                case 0:
                    surface_mesh = reconstructRegionGrowing(points);
                    break;
                case 1:
                    break;
            }
            return surface_mesh;
        }
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
        void simplifyMesh(Point_set *points) {
            int number_removed_points = 0;
            // Compute average spacing using neighborhood of simplification_average_spacing points
            double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag> (*points, simplification_average_spacing);
            // Simplify using a grid of size 2 * average spacing
            typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set (*points, 2. * spacing);
            points->remove(gsim_it, points->end());
            number_removed_points += points->number_of_removed_points();
            std::cout << number_removed_points << " point(s) removed after simplification." << std::endl;
            points->collect_garbage();
        }
        void estimateNormals(Point_set *points) {
            CGAL::jet_estimate_normals<CGAL::Sequential_tag>
              (*points, estimate_normals_neighbors); // Use estimate_normals_neighbors neighbors
            // Orientation of normals, returns iterator to first unoriented point
            typename Point_set::iterator unoriented_points_begin =
              CGAL::mst_orient_normals(*points, estimate_normals_neighbors); // Use estimate_normals_neighbors neighbors
            points->remove (unoriented_points_begin, points->end());
        }
        Surface_mesh reconstructRegionGrowing(Point_vector points) {
            CGAL::Timer t;
            t.start();
            // Shape detection.
            // Create instances of the classes Neighbor_query and Region_type.
            Neighbor_query neighbor_query(
                points,
                search_sphere_radius);

            Region_type region_type(
                points,
                max_distance_to_plane, max_accepted_angle, min_region_size);

            // Create an instance of the region growing class.
            Region_growing region_growing(
                points, neighbor_query, region_type);

            std::cout << "Extracting planes...";
            std::vector< std::vector<std::size_t> > regions;
            t.reset();
            region_growing.detect(std::back_inserter(regions));
            std::cout << " Done. " << regions.size() << " planes extracted. Time: "
            << t.time() << " sec." << std::endl;

            // Stores the plane index of each point as the third element of the tuple.
            Index_map index_map(points, regions);
            for (std::size_t i = 0; i < points.size(); ++i) {
                // Uses the get function from the property map that accesses the 3rd element of the tuple.
                const int plane_index = get(index_map, i);
                points[i].get<2>() = plane_index;
            }

            //////////////////////////////////////////////////////////////////////////

            // Reconstruction.
            std::cout << "Generating candidate faces...";
            t.reset();
            Polygonal_surface_reconstruction algo(
                points,
                Point_map(),
                Normal_map(),
                Plane_index_map()
            );
            std::cout << " Done. Time: " << t.time() << " sec." << std::endl;

            Surface_mesh model;
            std::cout << "Reconstructing...";
            t.reset();
            if (!algo.reconstruct<MIP_Solver>(model)) {
                std::cerr << "Failed: " << algo.error_message() << std::endl;
                throw "error to reconstruct in reconstructRegionGrowing()";
            }
            std::cout << " Done. Time: " << t.time() << " sec." << std::endl;
            return model;
        }
};

#endif  // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
#endif //URBANRECONSTRUCTION_SURFACE_RECONSTRUCTION_H
