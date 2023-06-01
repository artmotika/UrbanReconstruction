#ifndef URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H
#define URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include <iostream>
#include <fstream>
#include <utility>
#include <limits>
#include <set>
#include <iomanip>
#include <typeinfo>
#include <stack>
#include <deque>

#include "IO/Io_pcl.h"
#include "Geometry/Geometry_pcl.h"

using namespace pcl;
using namespace pcl::io;

struct polygon_struct {
    PointXYZ p1;
    PointXYZ p2;
    PointXYZ p3;
};

class Building_reconstruction {
    public:
        void setInputFile(std::string file_name);
        std::string getInputFile();
        void setInputFileSurfaces(std::string file_name);
        std::string getInputFileSurfaces();
        void setOutputFile(std::string file_name);
        std::string getOutputFile();
        void setPoissonDepth(int depth);
        int getPoissonDepth();
        void setSolverDivide(int divide);
        int getSolverDivide();
        void setIsoDivide(int divide);
        int getIsoDivide();
        void setPoissonPointWeight(float point_weight);
        float getPoissonPointWeight();
        void setConcaveHullAlpha(float alpha);
        float getConcaveHullAlpha();
        void setConcaveHullAlphaUpsample(float alpha);
        float getConcaveHullAlphaUpsample();
        void setConvexConcaveHull(bool hull_type);
        bool getConvexConcaveHull();
        void setFilterRadius(double radius);
        double getFilterRadius();

        PolygonMesh filter_mesh_by_mesh(PolygonMesh mesh_input, PolygonMesh mesh_filter);

        PolygonMesh filter_mesh_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius);

        PolygonMesh filter_mesh_poisson_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius);

        PointCloud<PointXYZ>::Ptr filter_points_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius);

        PointCloud<PointXYZ>::Ptr filter_points_by_mesh(PolygonMesh mesh_input);

        void upsample_mesh(PointCloud<PointXYZ>::Ptr &cloud_in, PolygonMesh mesh, double max_polygon_size,
                           double max_polygon_side);

        std::pair<unsigned long, double> calculate_repeatability_metric(PCLPointCloud2::Ptr cloud_ideal, PCLPointCloud2::Ptr cloud_repeat,
                                              double max_mistake, std::string cloud_not_repeat_path);

        std::pair<unsigned long, double> calculate_hole_metric(PCLPointCloud2::Ptr cloud_ideal, PCLPointCloud2::Ptr cloud_repeat, double max_mistake,
                                     std::string cloud_hole_path);

        PolygonMesh reconstruct();

        PolygonMesh reconstruct2();

    private:
        std::string input_file;
        std::string input_file_surfaces;
        std::string output_file;
        int poisson_depth = 10;
        int solver_divide = 8;
        int iso_divide = 8;
        float poisson_point_weight = 4.0f;
        float coefficient_distance_filtering = 1.8;
        float concave_hull_alpha = 8.0f;
        float concave_hull_alpha_upsample = 3.0f;
        bool convex_concave_hull = true;
        double filter_radius = 0.5;

        void compute_poisson (const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                              int depth, int solver_divide, int iso_divide, float point_weight);

        void compute_poisson(PointCloud<PointNormal>::Ptr &input, PolygonMesh &output,
                             int depth, int solver_divide, int iso_divide, float point_weight);

        void compute_hull (const PCLPointCloud2::ConstPtr &cloud_in,
                           bool convex_concave_hull,
                           float alpha,
                           PolygonMesh &mesh_out);

        void compute_hull (const PointCloud<PointXYZ>::ConstPtr &cloud_in,
                           bool convex_concave_hull,
                           float alpha,
                           PolygonMesh &mesh_out);

        void upsample_by_mesh(PointCloud<PointXYZ>::Ptr &cloud_in, PolygonMesh mesh);
};


#endif //URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H
