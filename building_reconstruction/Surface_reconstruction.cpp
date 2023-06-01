#include "Surface_reconstruction.h"

void Surface_reconstruction::simplifyMesh(Point_set *points) {
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

void Surface_reconstruction::estimateNormals(Point_set *points) {
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>
            (*points, estimate_normals_neighbors); // Use estimate_normals_neighbors neighbors
    // Orientation of normals, returns iterator to first unoriented point
    typename Point_set::iterator unoriented_points_begin =
            CGAL::mst_orient_normals(*points, estimate_normals_neighbors); // Use estimate_normals_neighbors neighbors
    points->remove (unoriented_points_begin, points->end());
}
Surface_reconstruction::Surface_mesh Surface_reconstruction::reconstructRegionGrowing(Point_vector points) {
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

    Surface_reconstruction::Surface_mesh model;
    std::cout << "Reconstructing...";
    t.reset();
    if (!algo.reconstruct<MIP_Solver>(model)) {
        std::cerr << "Failed: " << algo.error_message() << std::endl;
        throw "error to reconstruct in reconstructRegionGrowing()";
    }
    std::cout << " Done. Time: " << t.time() << " sec." << std::endl;
    return model;
}

void Surface_reconstruction::setInputFile(std::string file_name) {
    input_file = file_name;
}
std::string Surface_reconstruction::getInputFile() {
    return input_file;
}
void Surface_reconstruction::setOutputFile(std::string file_name) {
    output_file = file_name;
}
std::string Surface_reconstruction::getOutputFile() {
    return output_file;
}
void Surface_reconstruction::setShapeDetectionType(int type) {
    shape_detection_type = type;
}
int Surface_reconstruction::getShapeDetectionType() {
    return shape_detection_type;
}
void Surface_reconstruction::setSimplificationAverageSpacing(int average_spacing) {
    simplification_average_spacing = average_spacing;
}
int Surface_reconstruction::getSimplificationAverageSpacing() {
    return simplification_average_spacing;
}
void Surface_reconstruction::setEstimateNormalsNeighbors(int neighbors_number) {
    estimate_normals_neighbors = neighbors_number;
}
int Surface_reconstruction::getEstimateNormalsNeighbors() {
    return estimate_normals_neighbors;
}
void Surface_reconstruction::setSearchSphereRadius(FT radius) {
    search_sphere_radius = radius;
}
Surface_reconstruction::FT Surface_reconstruction::getSearchSphereRadius() {
    return search_sphere_radius;
}
void Surface_reconstruction::setMaxDistanceToPlane(FT distance) {
    max_distance_to_plane = distance;
}
Surface_reconstruction::FT Surface_reconstruction::getMaxDistanceToPlane() {
    return max_distance_to_plane;
}
void Surface_reconstruction::setMaxAcceptedAngle(FT angle) {
    max_accepted_angle = angle;
}
Surface_reconstruction::FT Surface_reconstruction::getMaxAcceptedAngle() {
    return max_accepted_angle;
}
void Surface_reconstruction::setMinRegionSize(std::size_t min_size) {
    min_region_size = min_size;
}
std::size_t Surface_reconstruction::getMinRegionSize() {
    return min_region_size;
}

Surface_reconstruction::Surface_mesh Surface_reconstruction::reconstruct() {
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

