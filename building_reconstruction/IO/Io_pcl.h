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

class Io_pcl {
    public:
        bool loadCloudPLY (const std::string &filename, PCLPointCloud2 &cloud)
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

        bool loadCloud (const std::string &filename, PCLPointCloud2 &cloud)
        {
            TicToc tt;
            print_highlight ("Loading ");
            print_value ("%s ", filename.c_str ());

            tt.tic ();
            if (loadPCDFile (filename, cloud) < 0)
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

        bool loadCloud (const std::string &filename, PointCloud<PointXYZ> &cloud)
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

        bool loadCloud (const std::string &filename, PolygonMesh &cloud)
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

            return (true);
        }

        void saveCloud (const std::string &filename, const PolygonMesh &cloud)
        {
            TicToc tt;
            tt.tic ();

            print_highlight ("Saving ");
            print_value ("%s ", filename.c_str ());

            savePLYFile (filename, cloud);

            print_info ("[done, ");
            print_value ("%g", tt.toc ());
            print_info (" ms]\n");
        }

        void saveCloud (std::string const& filename, PointCloud<PointXYZ> const& cloud)
        {
            TicToc tt;
            tt.tic ();

            print_highlight ("Saving ");
            print_value ("%s ", filename.c_str ());

            savePLYFile (filename, cloud);

            print_info ("[done, ");
            print_value ("%g", tt.toc ());
            print_info (" ms : ");
            print_value ("%d", cloud.width * cloud.height);
            print_info (" points]\n");
        }

        void saveCloud (std::string const& filename, PointCloud<PointNormal> const& cloud)
        {
            TicToc tt;
            tt.tic ();

            print_highlight ("Saving ");
            print_value ("%s ", filename.c_str ());

            savePLYFile (filename, cloud);

            print_info ("[done, ");
            print_value ("%g", tt.toc ());
            print_info (" ms : ");
            print_value ("%d", cloud.width * cloud.height);
            print_info (" points]\n");
        }

        void saveCloud (std::string const& filename, PCLPointCloud2 const& cloud)
        {
            TicToc tt;
            tt.tic ();

            print_highlight ("Saving ");
            print_value ("%s ", filename.c_str ());

            savePLYFile (filename, cloud);

            print_info ("[done, ");
            print_value ("%g", tt.toc ());
            print_info (" ms : ");
            print_value ("%d", cloud.width * cloud.height);
            print_info (" points]\n");
        }
};


#endif //URBANRECONSTRUCTION_IO_PCL_H
