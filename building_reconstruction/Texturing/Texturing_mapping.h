#ifndef URBANRECONSTRUCTION_TEXTURING_MAPPING_H
#define URBANRECONSTRUCTION_TEXTURING_MAPPING_H

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

using namespace std;
using namespace pcl;

PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::Normal))
)

namespace urban_rec {
    class Texturing_mapping {
    public:
        void setInputPolygonMesh(PolygonMesh polygon_mesh);

        PolygonMesh getInputPolygonMesh();

        void texture_mesh(vector <string> argv);

        static bool read_cam_pose_file(string filename,
                                TextureMapping<pcl::PointXYZ>::Camera &cam);

        static bool getPointUVCoords(const PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
                                     PointXY &UV_coordinates);

    private:
        PolygonMesh::Ptr input_polygon_mesh{nullptr};

    };
}

#endif //URBANRECONSTRUCTION_TEXTURING_MAPPING_H
