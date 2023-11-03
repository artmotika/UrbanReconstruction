#include "Algo_reconstruction.h"

namespace algo_rec {
    void compute_poisson (const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                          int depth, int solver_divide, int iso_divide, float point_weight)
    {
        PointCloud<PointNormal>::Ptr xyz_cloud (new PointCloud<PointNormal> ());
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

    void compute_poisson(PointCloud<PointNormal>::Ptr &input, PolygonMesh &output,
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

    void compute_greedy_triangulation (const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                                       double mu, double radius)
    {
        TicToc tt;
        tt.tic ();

        print_highlight (stderr, "Computing ");

        PointCloud<PointNormal>::Ptr xyz_cloud (new PointCloud<PointNormal> ());
        fromPCLPointCloud2 (*input, *xyz_cloud);

        PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal> ());
        for (std::size_t i = 0; i < xyz_cloud->size (); ++i)
            if (std::isfinite ((*xyz_cloud)[i].x))
                cloud->push_back ((*xyz_cloud)[i]);

        cloud->width = cloud->size ();
        cloud->height = 1;
        cloud->is_dense = true;

        GreedyProjectionTriangulation<PointNormal> gpt;
        gpt.setSearchMethod (pcl::search::KdTree<PointNormal>::Ptr (new pcl::search::KdTree<PointNormal>));
        gpt.setInputCloud (cloud);
        gpt.setSearchRadius (radius);
        gpt.setMu (mu);

        gpt.reconstruct (output);

        print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%lu", output.polygons.size ()); print_info (" polygons]\n");
    }

    void compute_hull (const PCLPointCloud2::ConstPtr &cloud_in,
                                                bool convex_concave_hull,
                                                float alpha,
                                                PolygonMesh &mesh_out)
    {
        PointCloud<PointXYZ>::Ptr xyz_cloud (new PointCloud<PointXYZ> ());
        fromPCLPointCloud2 (*cloud_in, *xyz_cloud);
        if (!convex_concave_hull)
        {
            print_info ("Computing the convex hull of a cloud with %lu points.\n", xyz_cloud->size ());
            ConvexHull<PointXYZ> convex_hull;
            convex_hull.setInputCloud (xyz_cloud);
            convex_hull.reconstruct (mesh_out);
        }
        else
        {
            print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, xyz_cloud->size ());
            ConcaveHull<PointXYZ> concave_hull;
            concave_hull.setInputCloud (xyz_cloud);
            concave_hull.setAlpha (alpha);
            concave_hull.reconstruct (mesh_out);
        }
    }

    void compute_hull (const PointCloud<PointXYZ>::ConstPtr &cloud_in,
                                                bool convex_concave_hull,
                                                float alpha,
                                                PolygonMesh &mesh_out)
    {
        if (!convex_concave_hull)
        {
            print_info ("Computing the convex hull of a cloud with %lu points.\n", cloud_in->size ());
            ConvexHull<PointXYZ> convex_hull;
            convex_hull.setInputCloud (cloud_in);
            convex_hull.reconstruct (mesh_out);
        }
        else
        {
            print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, cloud_in->size ());
            ConcaveHull<PointXYZ> concave_hull;
            concave_hull.setInputCloud (cloud_in);
            concave_hull.setAlpha (alpha);
            concave_hull.reconstruct (mesh_out);
        }
    }
}