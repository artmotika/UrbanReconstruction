#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
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

#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>

#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <cstdlib>
#include <vector>
#include <fstream>

#ifdef CGAL_USE_SCIP  // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;

#elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/GLPK_mixed_integer_program_traits.h>
typedef CGAL::GLPK_mixed_integer_program_traits<double>        MIP_Solver;

#endif

#if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)

#include <fstream>
#include <CGAL/Timer.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel        Kernel;

typedef Kernel::FT       FT;
typedef Kernel::Point_3         Point;
typedef Kernel::Vector_3 Vector;

// Point with normal, and plane index.
typedef boost::tuple<Point, Vector, int> PNI;
typedef std::vector<PNI> Point_vector;

typedef CGAL::Nth_of_tuple_property_map<0, PNI>        Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI>        Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI>        Plane_index_map;

typedef CGAL::Shape_detection::Point_set::
Sphere_neighbor_query<Kernel, Point_vector, Point_map> Neighbor_query;
typedef CGAL::Shape_detection::Point_set::
Least_squares_plane_fit_region<Kernel, Point_vector, Point_map, Normal_map> Region_type;
typedef CGAL::Shape_detection::
Region_growing<Point_vector, Neighbor_query, Region_type> Region_growing;

typedef CGAL::Surface_mesh<Point>        Surface_mesh;
typedef        CGAL::Polygonal_surface_reconstruction<Kernel> Polygonal_surface_reconstruction;

typedef CGAL::Point_set_3<Point, Vector> Point_set;

class Index_map {

public:
  using key_type = std::size_t;
  using value_type = int;
  using reference = value_type;
  using category = boost::readable_property_map_tag;

  Index_map() { }
  template<typename PointRange>
  Index_map(const PointRange& points,
            const std::vector< std::vector<std::size_t> >& regions)
    : m_indices(new std::vector<int>(points.size(), -1))
  {
    for (std::size_t i = 0; i < regions.size(); ++i)
      for (const std::size_t idx : regions[i])
        (*m_indices)[idx] = static_cast<int>(i);
  }

  inline friend value_type get(const Index_map& index_map,
                               const key_type key)
  {
    const auto& indices = *(index_map.m_indices);
    return indices[key];
  }

private:
  std::shared_ptr< std::vector<int> > m_indices;
};

/*
* This example first extracts planes from the input point cloud
* (using region growing) and then reconstructs
* the surface model from the planes.
*/

int main()
{

    Point_set points_start;
//  std::string fname = CGAL::data_file_path("points_3/out_af.ply");
//  std::ifstream stream (fname, std::ios_base::binary);
//  if (!stream)
//  {
//    std::cerr << "Error: cannot read file " << fname << std::endl;
//    return EXIT_FAILURE;
//  }
//  stream >> points_start;
//  std::cout << "Read " << points_start.size () << " point(s)" << std::endl;
//  if (points_start.empty())
//    return EXIT_FAILURE;
//
//
//  // Simplification
//
//  // Compute average spacing using neighborhood of 6 points
//  double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag> (points_start, 30); //30 до этого 200 исходник CGAL::data_file_path("points_3/Construction_corner_sample_oriented_normals_start.ply")
//  // Simplify using a grid of size 2 * average spacing
//  typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set (points_start, 2. * spacing);
//  points_start.remove(gsim_it, points_start.end());
//  std::cout << points_start.number_of_removed_points()
//            << " point(s) removed after simplification." << std::endl;
//  points_start.collect_garbage();
//
//
//  std::ofstream f ("data/out_af.ply");
//  f << points_start;
//  f.close ();



//  std::string fname = "data/out_af.ply";//CGAL::data_file_path("points_3/out_af.ply");
//  std::ifstream stream (fname, std::ios_base::binary);
//  if (!stream)
//  {
//    std::cerr << "Error: cannot read file " << fname << std::endl;
//    return EXIT_FAILURE;
//  }
//  stream >> points_start;
//  std::cout << "Read " << points_start.size () << " point(s)" << std::endl;
//  if (points_start.empty())
//    return EXIT_FAILURE;
//
//  // Estimate_normals
//
//    CGAL::jet_estimate_normals<CGAL::Sequential_tag>
//      (points_start, 6); // Use 24 neighbors
//    // Orientation of normals, returns iterator to first unoriented point
//    typename Point_set::iterator unoriented_points_begin =
//      CGAL::mst_orient_normals(points_start, 6); // Use 24 neighbors
//    points_start.remove (unoriented_points_begin, points_start.end());
//
//    std::ofstream f ("data/out_af.ply");
//    f << points_start;
//    f.close ();






  Point_vector points;

  // Load point set from a file.
  const std::string input_file("data/out_af.ply"); //CGAL::data_file_path("points_3/Construction_corner_sample_oriented_normals_start.ply")
    std::ifstream input_stream(input_file.c_str());
  if (input_stream.fail()) {
    std::cerr << "Failed open file \'" << input_file << "\'" << std::endl;
    return EXIT_FAILURE;
  }
  input_stream.close();
  std::cout << "Loading point cloud: " << input_file << "...";

  CGAL::Timer t;
  t.start();
  if (!CGAL::IO::read_points(input_file.c_str(), std::back_inserter(points),
                             CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))) {

    std::cerr << "Error: cannot read file " << input_file << std::endl;
    return EXIT_FAILURE;
  }
  else
    std::cout << " Done. " << points.size() << " points. Time: "
    << t.time() << " sec." << std::endl;


  //////////////////////////////////////////////////////////////////////////

  // Shape detection.

  // Default parameter values for the data file cube.pwn.
  const FT          search_sphere_radius  = FT(2) / FT(2.3); //100 Когда `sphere_radius` = 0,3 (c), получается лучший визуальный результат
//  (b) 17 регионов встречаются, когда `sphere_radius` = 0,1;
//(c) 8 регионов встречаются, когда `sphere_radius` = 0,3;
//(d) 4 региона найдены, когда `sphere_radius` = 1,2.
  const FT          max_distance_to_plane = FT(2) / FT(80); //1000
  const FT          max_accepted_angle    = FT(25); //25
  const std::size_t min_region_size       = 40; // ?40? ?60?

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
    return EXIT_FAILURE;
  }
  std::cout << " Done. Time: " << t.time() << " sec." << std::endl;

  std::cout << "Saving...";
  t.reset();
  const std::string& output_file("data/region_growing.ply");
  if (CGAL::IO::write_PLY(output_file, model, CGAL::parameters::use_binary_mode(false)))
    std::cout << " Done. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
  else {
    std::cerr << " Failed saving file." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

#else

int main(int, char**)
{
    std::cerr << "This test requires either GLPK or SCIP.\n";
    return EXIT_SUCCESS;
}

#endif  // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
