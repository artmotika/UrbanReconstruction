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

PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::Normal))
)

namespace urban_rec {
    class Texturing_mapping {
    public:
        void setInputPolygonMesh(pcl::PolygonMesh polygon_mesh);

        pcl::PolygonMesh getInputPolygonMesh();

        void texture_mesh(std::vector <std::string> argv);

    private:
        pcl::PolygonMesh::Ptr input_polygon_mesh{nullptr};

        bool read_cam_pose_file(std::string filename,
                                pcl::TextureMapping<pcl::PointXYZ>::Camera &cam);
    };
}

#endif //URBANRECONSTRUCTION_TEXTURING_MAPPING_H
