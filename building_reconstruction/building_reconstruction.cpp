#include "Building_reconstruction.h"

using namespace pcl;

int main()
{
    Io_pcl IO;
    Building_reconstruction building_reconstruction;
    building_reconstruction.setInputFile("../data/Construction_corner_oriented_normals.pcd"); //Construction_corner_oriented_normals_not_sample.pcd//Construction_corner_sample_oriented_normals.pcd
    building_reconstruction.setInputFileSurfaces("../data/region_growing.ply");
    building_reconstruction.setOutputFile("../data/building_reconstruction.ply");
    PolygonMesh building_mesh;
    building_mesh = building_reconstruction.reconstruct();
    PolygonMesh input_mesh;
    IO.loadCloud("../data/building_reconstruction.ply", input_mesh);
    PolygonMesh upsample_hull_mesh;
    IO.loadCloud("../data/hull_mesh1.ply", upsample_hull_mesh);
    building_mesh = building_reconstruction.filter_mesh_by_mesh(input_mesh, upsample_hull_mesh);
    IO.saveCloud("../data/building_reconstruction1.ply", building_mesh);
    return 0;
}