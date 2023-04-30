#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Polygonal_surface_reconstruction.h>

#ifdef CGAL_USE_SCIP  // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double>                        MIP_Solver;

#elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/GLPK_mixed_integer_program_traits.h>
typedef CGAL::GLPK_mixed_integer_program_traits<double>                        MIP_Solver;

#endif


#if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)

#include <CGAL/Timer.h>

#include <fstream>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal, and plane index
typedef boost::tuple<Point, Vector, int> PNI;
typedef std::vector<PNI> Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI> Plane_index_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Point_vector, Point_map, Normal_map> Traits;

typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Point_to_shape_index_map<Traits> Point_to_shape_index_map;

typedef CGAL::Polygonal_surface_reconstruction<Kernel> Polygonal_surface_reconstruction;
typedef CGAL::Surface_mesh<Point> Surface_mesh;

// PCL

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#include <iostream>
#include <fstream>
#include <utility>
#include <typeinfo>
#include <cmath>

using PointT = PointXYZ;
using CloudT = PointCloud<PointT>;


bool loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

static bool loadCloud (std::string const& filename, PointCloud<pcl::PointXYZ> &cloud)
{
  TicToc tt;
  print_highlight ("Loading ");
  print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPLYFile (filename, cloud) < 0)
    return (false);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
  print_info ("Available dimensions: ");
  print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void compute_poisson (const pcl::PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight)
{
  PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  fromPCLPointCloud2 (*input, *xyz_cloud);

  print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

  Poisson<PointNormal> poisson;
  poisson.setDepth (depth);
  poisson.setSolverDivide (solver_divide);
  poisson.setIsoDivide (iso_divide);
  poisson.setPointWeight (point_weight);
  poisson.setInputCloud (xyz_cloud);

  TicToc tt;
  tt.tic ();
  print_highlight ("Computing ...");
  poisson.reconstruct (output);

  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void compute_poisson1(PointCloud<PointNormal>::Ptr &input, PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight)
{
  print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

  Poisson<PointNormal> poisson;
  poisson.setDepth (depth);
  poisson.setSolverDivide (solver_divide);
  poisson.setIsoDivide (iso_divide);
  poisson.setPointWeight (point_weight);
  poisson.setInputCloud (input);

  TicToc tt;
  tt.tic ();
  print_highlight ("Computing ...");
  poisson.reconstruct (output);

  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void compute_greedy_triangulation (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PolygonMesh &output,
         double mu, double radius)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  fromPCLPointCloud2 (*input, *xyz_cloud);

  PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal> ());
  for (std::size_t i = 0; i < xyz_cloud->size (); ++i)
    if (std::isfinite ((*xyz_cloud)[i].x))
      cloud->push_back ((*xyz_cloud)[i]);

  cloud->width = cloud->size ();
  cloud->height = 1;
  cloud->is_dense = true;

  GreedyProjectionTriangulation<PointNormal> gpt;
  gpt.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  gpt.setInputCloud (cloud);
  gpt.setSearchRadius (radius);
  gpt.setMu (mu);

  gpt.reconstruct (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%lu", output.polygons.size ()); print_info (" polygons]\n");
}

void compute_hull (const pcl::PCLPointCloud2::ConstPtr &cloud_in,
         bool convex_concave_hull,
         float alpha,
         PolygonMesh &mesh_out)
{
    PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    fromPCLPointCloud2 (*cloud_in, *xyz_cloud);
  if (!convex_concave_hull)
  {
    print_info ("Computing the convex hull of a cloud with %lu points.\n", xyz_cloud->size ());
    ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud (xyz_cloud);
    convex_hull.reconstruct (mesh_out);
  }
  else
  {
    print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, xyz_cloud->size ());
    ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud (xyz_cloud);
    concave_hull.setAlpha (alpha);
    concave_hull.reconstruct (mesh_out);
  }
}

void compute_hull1 (const PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in,
         bool convex_concave_hull,
         float alpha,
         PolygonMesh &mesh_out)
{
  if (!convex_concave_hull)
  {
    print_info ("Computing the convex hull of a cloud with %lu points.\n", cloud_in->size ());
    ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud (cloud_in);
    convex_hull.reconstruct (mesh_out);
  }
  else
  {
    print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, cloud_in->size ());
    ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setInputCloud (cloud_in);
    concave_hull.setAlpha (alpha);
    concave_hull.reconstruct (mesh_out);
  }
}

//static CloudT::Ptr calculateHull (std::vector<pcl::Vertices>& polygons, int& dim, const pcl::PCLPointCloud2::ConstPtr &cloud_in, double alpha)
//{
//  pcl::ConcaveHull<PointT> hull_calculator;
//  CloudT::Ptr hull (new CloudT);
//  hull_calculator.setInputCloud (cloud);
//  hull_calculator.setAlpha (alpha);
//  hull_calculator.reconstruct (*hull, polygons);
//
//  dim = hull_calculator.getDimension ();
//  return hull;
//}

//void cropToHull (PointCloud<PointXYZ>::Ptr output, pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr hull_cloud, std::vector<pcl::Vertices> const& polygons, int dim)
//{
//  TicToc tt;
//  tt.tic ();
//
//  PointCloud<PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<PointXYZ> ());
//  fromPCLPointCloud2 (*input, *xyz_cloud);
//
//  PointCloud<PointXYZ>::Ptr xyz_hull_cloud (new pcl::PointCloud<PointXYZ> ());
//  fromPCLPointCloud2 (*hull_cloud, *xyz_hull_cloud);
//
//  print_highlight ("Cropping ");
//
//  CropHull<PointXYZ> crop_filter;
//  crop_filter.setInputCloud (xyz_cloud);
//  crop_filter.setHullCloud (xyz_hull_cloud);
//  crop_filter.setHullIndices (polygons);
//  crop_filter.setDim (dim);
//
//  crop_filter.filter (*output);
//
//  print_info ("[done, ");
//  print_value ("%g", tt.toc ());
//  print_info (" ms : ");
//  print_value ("%d", output->size());
//  print_info (" points passed crop]\n");
//}

static void cropToHull (PointCloud<pcl::PointXYZ>::Ptr output, PointCloud<pcl::PointXYZ>::Ptr input, PointCloud<pcl::PointXYZ>::Ptr hull_cloud, std::vector<pcl::Vertices> & polygons, int dim)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Cropping ");

  CropHull<pcl::PointXYZ> crop_filter;
  crop_filter.setInputCloud (input);
  crop_filter.setHullCloud (hull_cloud);
  crop_filter.setHullIndices (polygons);
  crop_filter.setCropOutside(true);
  crop_filter.setDim (dim);

  crop_filter.filter (*output);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", output->size());
  print_info (" points passed crop]\n");
}

static PointCloud<pcl::PointXYZ>::Ptr calculateHull (std::vector<pcl::Vertices>& polygons, int& dim, PointCloud<pcl::PointXYZ>::Ptr cloud, double alpha)
{
  pcl::ConcaveHull<pcl::PointXYZ> hull_calculator;
  PointCloud<pcl::PointXYZ>::Ptr hull_t (new PointCloud<pcl::PointXYZ>);
  hull_calculator.setInputCloud (cloud);
  hull_calculator.setAlpha (alpha);
  hull_calculator.reconstruct (*hull_t, polygons);

  dim = hull_calculator.getDimension ();
  return hull_t;
}

void saveCloud (const std::string &filename, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePLYFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

static void saveCloud (std::string const& filename, PointCloud<pcl::PointXYZ> const& cloud)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving ");
  print_value ("%s ", filename.c_str ());

  pcl::io::savePLYFile (filename, cloud);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
}

float euclidean_dist_between_two_points(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}

float max_euclidean_dist_side_in_polygon(std::vector<pcl::PointXYZ> ps)
{
    if (ps.size() < 3) return 0.0;
    pcl::PointXYZ p1 = ps[0];
    pcl::PointXYZ p2 = ps[1];
    pcl::PointXYZ p3 = ps[2];
    float max_dist = euclidean_dist_between_two_points(p1, p2);
    max_dist = std::max(max_dist, euclidean_dist_between_two_points(p2, p3));
    max_dist = std::max(max_dist, euclidean_dist_between_two_points(p1, p3));
    return max_dist;
}

float triangle_area_geron(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    float a = euclidean_dist_between_two_points(p1, p2);
    float b = euclidean_dist_between_two_points(p2, p3);
    float c = euclidean_dist_between_two_points(p1, p3);
    float p = (a+b+c) / 2;
    return sqrt(p*(p - a)*(p - b)*(p - c));
}

float min_euclidean_dist_between_point_and_polygon_points(pcl::PointXYZ p, std::vector<pcl::PointXYZ> ps)
{
    pcl::PointXYZ start_p = *ps.begin();
    float min_dist = euclidean_dist_between_two_points(p, start_p);
    for (std::vector<pcl::PointXYZ>::iterator it = ps.begin() + 1; it < ps.end(); it++) {
        min_dist = std::min(min_dist, euclidean_dist_between_two_points(p, *it));
    }
    return min_dist;
}

/*
* This example first extracts planes from the input point cloud
* (using RANSAC with default parameters) and then reconstructs
* the surface model from the planes.
*/

int main()
{
//  Point_vector points;
//
//  // Loads point set from a file.
//  const std::string input_file("data/out_af.ply"); //CGAL::data_file_path("points_3/Construction_corner_sample_oriented_normals_start_2.ply")
//    std::ifstream input_stream(input_file.c_str());
//  if (input_stream.fail()) {
//    std::cerr << "failed open file \'" <<input_file << "\'" << std::endl;
//    return EXIT_FAILURE;
//  }
//  input_stream.close();
//  std::cout << "Loading point cloud: " << input_file << "...";
//
//  CGAL::Timer t;
//  t.start();
//  if (!CGAL::IO::read_points(input_file.c_str(), std::back_inserter(points),
//                             CGAL::parameters::point_map(Point_map()).normal_map(Normal_map())))
//  {
//    std::cerr << "Error: cannot read file " << input_file << std::endl;
//    return EXIT_FAILURE;
//  }
//  else
//    std::cout << " Done. " << points.size() << " points. Time: " << t.time() << " sec." << std::endl;
//
//  // Shape detection
//  Efficient_ransac ransac;
//  ransac.set_input(points);
//  ransac.add_shape_factory<Plane>();
//
//  std::cout << "Extracting planes...";
//  t.reset();
//
//  // Register shapes for detection.
////  ransac.add_shape_factory<Plane>();
////  ransac.add_shape_factory<Sphere>();
////  ransac.add_shape_factory<Cylinder>();
////  ransac.add_shape_factory<Cone>();
////  ransac.add_shape_factory<Torus>();
//
//  // Set parameters for shape detection.
//  Efficient_ransac::Parameters parameters;
//
//  // Set probability to miss the largest primitive at each iteration.
//  parameters.probability = 0.05; //0.05 Более низкая вероятность обеспечивает более высокую надежность и детерминизм ценой более длительного времени работы из-за более высокой выносливости поиска
//
//  // Detect shapes with at least 200 points.
//  parameters.min_points = 200; // 200 * 400 ****96 <-----
//
//  // Set maximum Euclidean distance between a point and a shape.
//  parameters.epsilon = 2; // 0.002 чем меньше, тем больше плоскостей более детально 2.0 не разлечаются фасады, 0.5 различаются <-----
//
//  // Set maximum Euclidean distance between points to be clustered.
//  parameters.cluster_epsilon = 2;// 0.01 * 0.2 Большое значение `cluster_epsilon` приводит к обнаружению одной плоской формы
//
//  // Set maximum normal deviation.
//  // 0.9 < dot(surface_normal, point_normal);
//  parameters.normal_threshold = 0.9; // 0.9
//
//  // Detect shapes.
//  ransac.detect(parameters);
//
//  Efficient_ransac::Plane_range planes = ransac.planes();
//  std::size_t num_planes = planes.size();
//
//  std::cout << " Done. " << num_planes << " planes extracted. Time: " << t.time() << " sec." << std::endl;
//
//  // Stores the plane index of each point as the third element of the tuple.
//  Point_to_shape_index_map shape_index_map(points, planes);
//  for (std::size_t i = 0; i < points.size(); ++i) {
//    // Uses the get function from the property map that accesses the 3rd element of the tuple.
//    int plane_index = get(shape_index_map, i);
//    points[i].get<2>() = plane_index;
//  }
//
//  //////////////////////////////////////////////////////////////////////////
//
//  std::cout << "Generating candidate faces...";
//  t.reset();
//
//  Polygonal_surface_reconstruction algo(
//    points,
//    Point_map(),
//    Normal_map(),
//    Plane_index_map()
//  );
//
//  std::cout << " Done. Time: " << t.time() << " sec." << std::endl;
//
//  //////////////////////////////////////////////////////////////////////////
//
//  Surface_mesh model;
//
//  std::cout << "Reconstructing...";
//  t.reset();
//
//  if (!algo.reconstruct<MIP_Solver>(model)) {
//    std::cerr << " Failed: " << algo.error_message() << std::endl;
//    return EXIT_FAILURE;
//  }
//
//  const std::string& output_file("data/construction.ply");
//  if (CGAL::IO::write_PLY(output_file, model))
//    std::cout << " Done. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
//  else {
//    std::cerr << " Failed saving file." << std::endl;
//    return EXIT_FAILURE;
//  }
//
//  //////////////////////////////////////////////////////////////////////////
//
//  // Also stores the candidate faces as a surface mesh to a file
//  Surface_mesh candidate_faces;
//  algo.output_candidate_faces(candidate_faces);
//  const std::string& candidate_faces_file("data/cube_candidate_faces.off");
//  std::ofstream candidate_stream(candidate_faces_file.c_str());
//  if (CGAL::IO::write_OFF(candidate_stream, candidate_faces))
//    std::cout << "Candidate faces saved to " << candidate_faces_file << "." << std::endl;



  // PCL
    int default_depth = 10; //20
    int default_solver_divide = 8;
    int default_iso_divide = 8;
    float default_point_weight = 4.0f;

    int depth = default_depth;
    int solver_divide = default_solver_divide;
    int iso_divide = default_iso_divide;
    float point_weight = default_point_weight;

    // Load the first file
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    std::string filename = CGAL::data_file_path("points_3/Construction_corner_sample_oriented_normals.pcd");
    if (!loadCloud (filename, *cloud))
        return (-1);

    // Apply the Poisson surface reconstruction algorithm
    PolygonMesh output;
    compute_poisson (cloud, output, depth, solver_divide, iso_divide, point_weight);

    // Delete extra polygons
    std::vector< ::pcl::Vertices> output_polygons = output.polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(output.cloud, *all_vertices);
    TicToc tt;
    tt.tic ();
    std::vector< ::pcl::Vertices> new_polygons;
    for (std::vector< ::pcl::Vertices>::iterator it = output_polygons.begin(); it != output_polygons.end(); it++) {
        Indices vertecies = it->vertices;
        pcl::PointXYZ p1;
        pcl::PointXYZ p2;
        pcl::PointXYZ p3;
        if (vertecies.size() >= 3) {
            p1 = all_vertices->points[vertecies[0]];
            p2 = all_vertices->points[vertecies[1]];
            p3 = all_vertices->points[vertecies[2]];
        } else continue;

        std::vector<pcl::PointXYZ> polygon_points = {p1, p2, p3};

        if (triangle_area_geron(p1, p2, p3) < 0.06) {
            std::cout << max_euclidean_dist_side_in_polygon(polygon_points) << std::endl;
            if (max_euclidean_dist_side_in_polygon(polygon_points) < 0.2) {
                new_polygons.push_back(*it);
            }
        }
    }

    output.polygons = new_polygons;
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

    std::string filename_output = "data/Construction_corner_poisson_end.ply";
    saveCloud (filename_output, output);

    // Load the second file polygonmesh
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PLYReader Reader;
      Reader.read("data/region_growing_points.ply", *cloud_msr);
      filename_output = "data/cloud_msr.ply";
//      saveCloud ("data/cloud_msr1.ply", cloud_msr);
      pcl::io::savePLYFileBinary(filename_output, *cloud_msr);
      pcl::PolygonMesh mesh_surface_reconstruct;
      pcl::io::loadPLYFile ("data/region_growing.ply", mesh_surface_reconstruct); //region_growing_delanay

      PolygonMesh delanay_mesh;
      double mu = 0.1;
      double radius = 1000.0;
      std::shared_ptr<pcl::PCLPointCloud2> shared_ptr_cloud = std::make_shared<pcl::PCLPointCloud2>(mesh_surface_reconstruct.cloud);
      compute_greedy_triangulation (shared_ptr_cloud, delanay_mesh, mu, radius);
      saveCloud ("data/delanay_mesh.ply", delanay_mesh);

      float default_alpha = 8.0f;
      bool convex_concave_hull = true;
      float alpha = default_alpha;
      PolygonMesh hull_mesh;
      compute_hull(shared_ptr_cloud, convex_concave_hull, alpha, hull_mesh);
      saveCloud ("data/hull_mesh.ply", hull_mesh);





      //---------------------------------------------------------------


//      PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new PointCloud<pcl::PointXYZ>);
//      PointCloud<pcl::PointXYZ>::Ptr hull_points (new PointCloud<pcl::PointXYZ>);
//      PointCloud<pcl::PointXYZ>::Ptr input_cloud (new PointCloud<pcl::PointXYZ>);
//      PointCloud<pcl::PointXYZ>::Ptr output_cloud (new PointCloud<pcl::PointXYZ>);
//      std::vector<pcl::Vertices> hull_polygons;
//      int dim = 0;
//
//      filename_output = "data/Construction_corner_sample_inverted_oriented_normals.ply";
//      if (!loadCloud (filename_output, *input_cloud))
//        return (-1);
//
//      if (!loadCloud ("data/region_growing.ply", *hull_cloud))
//        return (-1);
//
//      std::cout << "hull_cloud size: " << hull_cloud->size() << std::endl;
//
//
//      hull_points = calculateHull (hull_polygons, dim, hull_cloud, 8.0f);
//
//      saveCloud ("data/region_growing.ply.ply", *hull_points);
//
//      PolygonMesh hull_mesh_1;
//      compute_hull1(hull_points, convex_concave_hull, alpha, hull_mesh_1);
////      compute_poisson1 (mls_cloud, hull_mesh_1, 1, solver_divide, iso_divide, point_weight);
//        std::cout << hull_points->size() << std::endl;
//        std::cout << dim << std::endl;
////      hull_mesh_1.polygons = hull_polygons;
//      saveCloud ("data/mesh_hull.ply", hull_mesh_1);
//
//
//      cropToHull (output_cloud, input_cloud, hull_points, hull_polygons, dim);
//
//      saveCloud ("data/crop_hull.ply", *output_cloud);
//
//
//
//      PointCloud<PointXYZ>::Ptr in_hull (new pcl::PointCloud<PointXYZ> ());
//      std::shared_ptr<pcl::PCLPointCloud2> ptr_hull_mesh_cloud = std::make_shared<pcl::PCLPointCloud2>(hull_mesh.cloud);


//---------------------------------------------------------------
      pcl::PolygonMesh poisson_mesh;
      pcl::io::loadPLYFile ("data/Construction_corner_poisson_end.ply", poisson_mesh);

//      pcl::PCLPointCloud2::Ptr cloud_msr (new pcl::PCLPointCloud2);
//      filename = "data/region_growing_points.pcd";
//      if (!loadCloud (filename, *cloud_msr))
//          return (-1);

//      filename_output = "data/Construction_corner_region_growing_end.ply";
//      savePLYFile (filename_output, mesh_surface_reconstruct);
      output_polygons = output.polygons;
      std::vector< ::pcl::Vertices> mesh_surface_reconstruct_polygons = mesh_surface_reconstruct.polygons;
      std::cout << "number polygons region_growing " << mesh_surface_reconstruct_polygons.size() << std::endl;
      std::vector< ::pcl::Vertices> hull_mesh_polygons = hull_mesh.polygons;
      std::cout << "number polygons hull region_growing " << hull_mesh_polygons.size() << std::endl;

      // Load the third first file cloud
      tt.tic ();
//    pcl::PCLPointCloud2::Ptr cloud_sr (new pcl::PCLPointCloud2);
//    filename = "data/out_af.pcd";
//    if (!loadCloud (filename, *cloud_sr))
//        return (-1);

    std::vector< ::pcl::Vertices> new_polygons_pp;
      // Apply the Poisson surface reconstruction algorithm
//    compute (cloud_msr, mesh_surface_reconstruct, 1, solver_divide, iso_divide, point_weight);
//    mesh_surface_reconstruct.polygons = mesh_surface_reconstruct_polygons;
//    std::vector< ::pcl::Vertices> mesh_surface_reconstruct_polygons = mesh_surface_reconstruct.polygons;
    filename_output = "data/Construction_corner_region_growing2_end.ply";
    pcl::io::savePLYFile(filename_output, mesh_surface_reconstruct); //saveCloud (filename_output, mesh_surface_reconstruct);


    pcl::PointCloud<pcl::PointXYZ>::Ptr all_vertices_rg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh_surface_reconstruct.cloud, *all_vertices_rg);

    std::vector< ::pcl::Vertices> new_polygons_sr;
    int polygons_left = poisson_mesh.polygons.size();
    std::cout << "number polygons: " << polygons_left << std::endl << mesh_surface_reconstruct.polygons.size() << std::endl;



    pcl::io::savePLYFile ("data/hull_mesh_cloud.ply", hull_mesh.cloud);
    PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    fromPCLPointCloud2 (*shared_ptr_cloud, *xyz_cloud);
    for (std::vector< ::pcl::Vertices>::iterator it = mesh_surface_reconstruct.polygons.begin();
            it != mesh_surface_reconstruct.polygons.end(); it++) {
            Indices vertecies = it->vertices;
            size_t size = vertecies.size();
            pcl::PointXYZ p1;
            pcl::PointXYZ p2;
            pcl::PointXYZ p3;
            pcl::PointXYZ p4;
            pcl::PointXYZ p5;
            pcl::PointXYZ pcenter;

            pcl::PointXYZ pcenter45;
            pcl::PointXYZ pcenter51;
            if (size == 3) {
                p1 = all_vertices_rg->points[vertecies[0]];
                p2 = all_vertices_rg->points[vertecies[1]];
                p3 = all_vertices_rg->points[vertecies[2]];
                pcenter.x = (p1.x + p2.x + p3.x) / 3.0;
                pcenter.y = (p1.y + p2.y + p3.y) / 3.0;
                pcenter.z = (p1.z + p2.z + p3.z) / 3.0;
                pcl::PointXYZ pcenter12;
                pcl::PointXYZ pcenter23;
                pcl::PointXYZ pcenter31;
                pcenter12.x = (pcenter.x + p1.x + p2.x) / 3.0;
                pcenter23.x = (pcenter.x + p2.x + p3.x) / 3.0;
                pcenter31.x = (pcenter.x + p3.x + p1.x) / 3.0;
                pcenter12.y = (pcenter.y + p1.y + p2.y) / 3.0;
                pcenter23.y = (pcenter.y + p2.y + p3.y) / 3.0;
                pcenter31.y = (pcenter.y + p3.y + p1.y) / 3.0;
                pcenter12.z = (pcenter.z + p1.z + p2.z) / 3.0;
                pcenter23.z = (pcenter.z + p2.z + p3.z) / 3.0;
                pcenter31.z = (pcenter.z + p3.z + p1.z) / 3.0;
                (*xyz_cloud).push_back(pcenter);
                (*xyz_cloud).push_back(pcenter12);
                (*xyz_cloud).push_back(pcenter23);
                (*xyz_cloud).push_back(pcenter31);
            } else if (size == 4) {
                p1 = all_vertices_rg->points[vertecies[0]];
                p2 = all_vertices_rg->points[vertecies[1]];
                p3 = all_vertices_rg->points[vertecies[2]];
                p4 = all_vertices_rg->points[vertecies[3]];
                pcl::PointXYZ pcenter;
                pcenter.x = (p1.x + p2.x + p3.x + p4.x) / 4.0;
                pcenter.y = (p1.y + p2.y + p3.y + p4.y) / 4.0;
                pcenter.z = (p1.z + p2.z + p3.z + p4.z) / 4.0;
                pcl::PointXYZ pcenter12;
                pcl::PointXYZ pcenter23;
                pcl::PointXYZ pcenter34;
                pcl::PointXYZ pcenter41;
                pcenter12.x = (pcenter.x + p1.x + p2.x) / 3.0;
                pcenter23.x = (pcenter.x + p2.x + p3.x) / 3.0;
                pcenter34.x = (pcenter.x + p3.x + p4.x) / 3.0;
                pcenter41.x = (pcenter.x + p4.x + p1.x) / 3.0;
                pcenter12.y = (pcenter.y + p1.y + p2.y) / 3.0;
                pcenter23.y = (pcenter.y + p2.y + p3.y) / 3.0;
                pcenter34.y = (pcenter.y + p3.y + p4.y) / 3.0;
                pcenter41.y = (pcenter.y + p4.y + p1.y) / 3.0;
                pcenter12.z = (pcenter.z + p1.z + p2.z) / 3.0;
                pcenter23.z = (pcenter.z + p2.z + p3.z) / 3.0;
                pcenter34.z = (pcenter.z + p3.z + p4.z) / 3.0;
                pcenter41.z = (pcenter.z + p4.z + p1.z) / 3.0;
                (*xyz_cloud).push_back(pcenter);
                (*xyz_cloud).push_back(pcenter12);
                (*xyz_cloud).push_back(pcenter23);
                (*xyz_cloud).push_back(pcenter34);
                (*xyz_cloud).push_back(pcenter41);
            } else if (size == 5) {
                p1 = all_vertices_rg->points[vertecies[0]];
                p2 = all_vertices_rg->points[vertecies[1]];
                p3 = all_vertices_rg->points[vertecies[2]];
                p4 = all_vertices_rg->points[vertecies[3]];
                p5 = all_vertices_rg->points[vertecies[4]];
                pcl::PointXYZ pcenter;
                pcenter.x = (p1.x + p2.x + p3.x + p4.x + p5.x) / 5.0;
                pcenter.y = (p1.y + p2.y + p3.y + p4.y + p5.y) / 5.0;
                pcenter.z = (p1.z + p2.z + p3.z + p4.z + p5.z) / 5.0;
                pcl::PointXYZ pcenter12;
                pcl::PointXYZ pcenter23;
                pcl::PointXYZ pcenter34;
                pcl::PointXYZ pcenter45;
                pcl::PointXYZ pcenter51;
                pcenter12.x = (pcenter.x + p1.x + p2.x) / 3.0;
                pcenter23.x = (pcenter.x + p2.x + p3.x) / 3.0;
                pcenter34.x = (pcenter.x + p3.x + p4.x) / 3.0;
                pcenter45.x = (pcenter.x + p4.x + p5.x) / 3.0;
                pcenter51.x = (pcenter.x + p5.x + p1.x) / 3.0;
                pcenter12.y = (pcenter.y + p1.y + p2.y) / 3.0;
                pcenter23.y = (pcenter.y + p2.y + p3.y) / 3.0;
                pcenter34.y = (pcenter.y + p3.y + p4.y) / 3.0;
                pcenter45.y = (pcenter.y + p4.y + p5.y) / 3.0;
                pcenter51.y = (pcenter.y + p5.y + p1.y) / 3.0;
                pcenter12.z = (pcenter.z + p1.z + p2.z) / 3.0;
                pcenter23.z = (pcenter.z + p2.z + p3.z) / 3.0;
                pcenter34.z = (pcenter.z + p3.z + p4.z) / 3.0;
                pcenter45.z = (pcenter.z + p4.z + p5.z) / 3.0;
                pcenter51.z = (pcenter.z + p5.z + p1.z) / 3.0;
                (*xyz_cloud).push_back(pcenter);
                (*xyz_cloud).push_back(pcenter12);
                (*xyz_cloud).push_back(pcenter23);
                (*xyz_cloud).push_back(pcenter34);
                (*xyz_cloud).push_back(pcenter45);
                (*xyz_cloud).push_back(pcenter51);
            }
    }

    alpha = 2.5f;
    PolygonMesh hull_mesh1;
    compute_hull1(xyz_cloud, convex_concave_hull, alpha, hull_mesh1);
    saveCloud ("data/hull_mesh1.ply", hull_mesh1);
    pcl::io::savePLYFile ("data/hull_mesh1_cloud.ply", hull_mesh1.cloud);

    hull_mesh_polygons = hull_mesh1.polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_vertices_sr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(hull_mesh1.cloud, *all_vertices_sr);



    for (std::vector< ::pcl::Vertices>::iterator it1 = poisson_mesh.polygons.begin(); it1 != poisson_mesh.polygons.end(); it1++) {
        Indices vertecies = it1->vertices;
        pcl::PointXYZ p1;
        pcl::PointXYZ p2;
        pcl::PointXYZ p3;
        if (vertecies.size() >= 3) {
            p1 = all_vertices->points[vertecies[0]];
            p2 = all_vertices->points[vertecies[1]];
            p3 = all_vertices->points[vertecies[2]];
        } else continue;

        polygons_left --;
        if (polygons_left % 10000 == 0) std::cout << "number polygons left: " << polygons_left << std::endl;

        pcl::PointXYZ pcenter;
        pcenter.x = (p1.x + p2.x + p3.x) / 3.0;
        pcenter.y = (p1.y + p2.y + p3.y) / 3.0;
        pcenter.z = (p1.z + p2.z + p3.z) / 3.0;
        bool pcenter_accepted = false;

        for (std::vector< ::pcl::Vertices>::iterator it2 = hull_mesh_polygons.begin();
            it2 != hull_mesh_polygons.end(); it2++) {
            Indices vertecies = it2->vertices;
            pcl::PointXYZ p1_sr;
            pcl::PointXYZ p2_sr;
            pcl::PointXYZ p3_sr;
//            pcl::PointXYZ pcenter_sr;

            if (vertecies.size() == 3) {
                p1_sr = all_vertices_sr->points[vertecies[0]];
                p2_sr = all_vertices_sr->points[vertecies[1]];
                p3_sr = all_vertices_sr->points[vertecies[2]];
            } else continue;

//            pcenter_sr.x = (p1_sr.x + p2_sr.x + p3_sr.x) / 3.0;
//            pcenter_sr.y = (p1_sr.y + p2_sr.y + p3_sr.y) / 3.0;
//            pcenter_sr.z = (p1_sr.z + p2_sr.z + p3_sr.z) / 3.0;
//            std::vector<float> side_distances = {euclidean_dist_between_two_points(p1_sr, p2_sr),
//                                            euclidean_dist_between_two_points(p2_sr, p3_sr),
//                                            euclidean_dist_between_two_points(p3_sr, p1_sr)};
//            std::sort(side_distances.begin(), side_distances.end());
//            float max_side = side_distances[2];
//
//            if (!(euclidean_dist_between_two_points(pcenter, pcenter_sr) < max_side)) continue;
//
            float s1 = triangle_area_geron(p1_sr, p2_sr, p3_sr);

            if (!pcenter_accepted) {
                float s2 = triangle_area_geron(pcenter, p1_sr, p2_sr) + triangle_area_geron(pcenter, p2_sr, p3_sr) + triangle_area_geron(pcenter, p1_sr, p3_sr);
                if (s2 < s1*1.6) {
                    pcenter_accepted = true;
                }
            }
            if (pcenter_accepted) {
                new_polygons_sr.push_back(*it1);
                break;
            }
        }
    }

      poisson_mesh.polygons = new_polygons_sr;
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

    // Save into the second file
    filename_output = "data/Construction_corner_end.ply";
    saveCloud (filename_output, poisson_mesh);

  return EXIT_SUCCESS;
}


#else

int main(int, char**)
{
    std::cerr << "This test requires either GLPK or SCIP.\n";
    return EXIT_SUCCESS;
}

#endif  // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
