#ifndef URBANRECONSTRUCTION_LIDAR2DEPTH_H
#define URBANRECONSTRUCTION_LIDAR2DEPTH_H

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/types.hpp>
#include <tuple>
#include <limits.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include "../Texturing_mapping.h"
#include "../../StringUtils/PathUtils.h"
#include "../../IO/Io_pcl.h"
#include "../../Geometry/Geometry_pcl.h"

using namespace std;

namespace urban_rec {
    using Camera = pcl::TextureMapping<pcl::PointXYZ>::Camera;
    using PointCoordsSet = vector<tuple<
            tuple<PointXYZ, PointXYZ, PointXYZ>,
    tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2i>,
    tuple<bool, bool, bool>>>;
    class Lidar2depth {
    public:
        Lidar2depth(string base_dir_path, string input_mesh_file_path = "") {
            setBaseDirPath(base_dir_path);
            if (input_mesh_file_path != "") {
                setInputMeshFilePath(input_mesh_file_path);
            }
        }

        void setInputPolygonMesh(pcl::PolygonMesh polygon_mesh);

        pcl::PolygonMesh getInputPolygonMesh();

        void setInputMeshFilePath(string input_mesh_file_path);

        string getInputMeshFilePath();

        void setBaseDirPath(string base_dir_path);

        string getBaseDirPath();

        void createDepthImages();

    private:
        string input_mesh_file_path;
        string base_dir_path;
        pcl::PolygonMesh::Ptr input_polygon_mesh{nullptr};

        void readCamPoses(vector<Camera> *cams, vector<boost::filesystem::path> *filenames);

        void fillPointCoordsSet(Camera cam, PointCoordsSet *polygons);
    };
}

#endif //URBANRECONSTRUCTION_LIDAR2DEPTH_H
