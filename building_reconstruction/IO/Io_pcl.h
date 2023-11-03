#ifndef URBANRECONSTRUCTION_IO_PCL_H
#define URBANRECONSTRUCTION_IO_PCL_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

namespace Io_pcl {

        bool loadCloudPLY (const std::string &filename, PCLPointCloud2 &cloud);

        bool loadCloud (const std::string &filename, PCLPointCloud2 &cloud);

        bool loadCloud (const std::string &filename, PointCloud<PointXYZ> &cloud);

        bool loadCloud (const std::string &filename, PolygonMesh &cloud);

        void saveCloud (const std::string &filename, const PolygonMesh &cloud);

        void saveCloud (std::string const& filename, PointCloud<PointXYZ> const& cloud);

        void saveCloud (std::string const& filename, PointCloud<PointNormal> const& cloud);

        void saveCloud (std::string const& filename, PCLPointCloud2 const& cloud);

        void saveCloudPCD (std::string const& filename, PointCloud<PointXYZ> const& cloud);

        void saveCloudPCD (std::string const& filename, PointCloud<PointNormal> const& cloud);

        void saveCloudPCD (std::string const& filename, PCLPointCloud2 const& cloud);
}

#endif //URBANRECONSTRUCTION_IO_PCL_H
