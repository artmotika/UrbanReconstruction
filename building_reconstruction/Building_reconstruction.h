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

#include "IO/Io_pcl.h"
#include "Geometry/Geometry_pcl.h"

using namespace pcl;
using namespace pcl::io;

class Building_reconstruction {
    public:
        using PointT = PointXYZ;
        using CloudT = PointCloud<PointT>;
        void setInputFile(std::string file_name) {
            input_file = file_name;
        }
        std::string getInputFile() {
            return input_file;
        }
        void setInputFileSurfaces(std::string file_name) {
            input_file_surfaces = file_name;
        }
        std::string getInputFileSurfaces() {
            return input_file_surfaces;
        }
        void setOutputFile(std::string file_name) {
            output_file = file_name;
        }
        std::string getOutputFile() {
            return output_file;
        }
        void setPoissonDepth(int depth) {
            poisson_depth = depth;
        }
        int getPoissonDepth() {
            return poisson_depth;
        }
        void setSolverDivide(int divide) {
            solver_divide = divide;
        }
        int getSolverDivide() {
            return solver_divide;
        }
        void setIsoDivide(int divide) {
            iso_divide = divide;
        }
        int getIsoDivide() {
            return iso_divide;
        }
        void setPoissonPointWeight(float point_weight) {
            poisson_point_weight = point_weight;
        }
        float getPoissonPointWeight() {
            return poisson_point_weight;
        }
        void setConcaveHullAlpha(float alpha) {
            concave_hull_alpha = alpha;
        }
        float getConcaveHullAlpha() {
            return concave_hull_alpha;
        }
        void setConcaveHullAlphaUpsample(float alpha) {
            concave_hull_alpha_upsample = alpha;
        }
        float getConcaveHullAlphaUpsample() {
            return concave_hull_alpha_upsample;
        }
        void setConvexConcaveHull(bool hull_type) {
            convex_concave_hull = hull_type;
        }
        bool getConvexConcaveHull() {
            return convex_concave_hull;
        }

        PolygonMesh filter_mesh_by_mesh(PolygonMesh mesh_input, PolygonMesh mesh_filter) {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();

            std::vector<Vertices> new_polygons_sr;

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_input.cloud, *all_vertices);

            PointCloud<PointXYZ>::Ptr all_vertices_hull(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_filter.cloud, *all_vertices_hull);

            int polygons_left = mesh_input.polygons.size();
            std::cout << "number polygons: " << polygons_left << std::endl;

            for (std::vector<Vertices>::iterator it1 = mesh_input.polygons.begin(); it1 != mesh_input.polygons.end(); it1++) {
                Indices vertecies = it1->vertices;
                PointXYZ p1;
                PointXYZ p2;
                PointXYZ p3;
                if (vertecies.size() >= 3) {
                    p1 = all_vertices->points[vertecies[0]];
                    p2 = all_vertices->points[vertecies[1]];
                    p3 = all_vertices->points[vertecies[2]];
                } else continue;

                polygons_left --;
                if (polygons_left % 10000 == 0) std::cout << "number polygons left: " << polygons_left << std::endl;

                PointXYZ pcenter;
                pcenter.x = (p1.x + p2.x + p3.x) / 3.0;
                pcenter.y = (p1.y + p2.y + p3.y) / 3.0;
                pcenter.z = (p1.z + p2.z + p3.z) / 3.0;
                bool pcenter_accepted = false;

                for (std::vector<Vertices>::iterator it2 = mesh_filter.polygons.begin();
                     it2 != mesh_filter.polygons.end(); it2++) {
                    Indices vertecies = it2->vertices;
                    PointXYZ p1_sr;
                    PointXYZ p2_sr;
                    PointXYZ p3_sr;
                    PointXYZ pcenter_sr;

                    if (vertecies.size() == 3) {
                        p1_sr = all_vertices_hull->points[vertecies[0]];
                        p2_sr = all_vertices_hull->points[vertecies[1]];
                        p3_sr = all_vertices_hull->points[vertecies[2]];
                    } else continue;

                    pcenter_sr.x = (p1_sr.x + p2_sr.x + p3_sr.x) / 3.0;
                    pcenter_sr.y = (p1_sr.y + p2_sr.y + p3_sr.y) / 3.0;
                    pcenter_sr.z = (p1_sr.z + p2_sr.z + p3_sr.z) / 3.0;
                    std::vector<double> side_distances = {geometryPcl.euclidean_dist_between_two_points(p1_sr, p2_sr),
                                                          geometryPcl.euclidean_dist_between_two_points(p2_sr, p3_sr),
                                                          geometryPcl.euclidean_dist_between_two_points(p3_sr, p1_sr)};
                    std::sort(side_distances.begin(), side_distances.end());
                    double max_side = side_distances[2];

                    if (geometryPcl.euclidean_dist_between_two_points(pcenter, pcenter_sr) > 1.5 * max_side) continue;

                    double s1 = geometryPcl.triangle_area_geron(p1_sr, p2_sr, p3_sr);

                    if (!pcenter_accepted) {
                        double s2 = geometryPcl.triangle_area_geron(pcenter, p1_sr, p2_sr) + geometryPcl.triangle_area_geron(pcenter, p2_sr, p3_sr) + geometryPcl.triangle_area_geron(pcenter, p1_sr, p3_sr);
                        if (s2 < s1*coefficient_distance_filtering) {
                            pcenter_accepted = true;
                        }
                    }
                    if (pcenter_accepted) {
                        new_polygons_sr.push_back(*it1);
                        break;
                    }
                }
            }

            mesh_input.polygons = new_polygons_sr;
            print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

            return mesh_input;

        }

        PolygonMesh reconstruct() {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();
            PCLPointCloud2::Ptr input_cloud (new PCLPointCloud2);
            IO.loadCloud(input_file, *input_cloud);

            // Apply the Poisson surface reconstruction algorithm
            PolygonMesh poisson_mesh1;
            compute_poisson(input_cloud, poisson_mesh1, poisson_depth, solver_divide, iso_divide, poisson_point_weight);
            IO.saveCloud ("../data/poisson_mesh.ply", poisson_mesh1);

            PolygonMesh surfaces_mesh;
            IO.loadCloud(input_file_surfaces, surfaces_mesh);

            PointCloud<PointXYZ>::Ptr cloud_surfaces (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (surfaces_mesh.cloud, *cloud_surfaces);

            PointCloud<PointXYZ>::Ptr upsample_cloud (new PointCloud<PointXYZ> ());
            copyPointCloud(*cloud_surfaces, *upsample_cloud);

            upsample_by_mesh(upsample_cloud, surfaces_mesh);

            PolygonMesh upsample_hull_mesh;
            compute_hull(upsample_cloud, convex_concave_hull, concave_hull_alpha_upsample, upsample_hull_mesh);

            PointCloud<PointXYZ>::Ptr new_vertices(new PointCloud<PointXYZ>);

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(*input_cloud, *all_vertices);

            PointCloud<PointXYZ>::Ptr all_vertices_hull(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(upsample_hull_mesh.cloud, *all_vertices_hull);

            int number_points = all_vertices->size();

            for (PointXYZ p : *all_vertices) {

                number_points --;
                if (number_points % 10000 == 0) std::cout << "number points left: " << number_points << std::endl;
                bool p_accepted = false;

                for (std::vector<Vertices>::iterator it2 = upsample_hull_mesh.polygons.begin();
                     it2 != upsample_hull_mesh.polygons.end(); it2++) {
                    Indices vertecies = it2->vertices;
                    PointXYZ p1_sr;
                    PointXYZ p2_sr;
                    PointXYZ p3_sr;
                    PointXYZ pcenter_sr;

                    if (vertecies.size() == 3) {
                        p1_sr = all_vertices_hull->points[vertecies[0]];
                        p2_sr = all_vertices_hull->points[vertecies[1]];
                        p3_sr = all_vertices_hull->points[vertecies[2]];
                    } else continue;

                    pcenter_sr.x = (p1_sr.x + p2_sr.x + p3_sr.x) / 3.0;
                    pcenter_sr.y = (p1_sr.y + p2_sr.y + p3_sr.y) / 3.0;
                    pcenter_sr.z = (p1_sr.z + p2_sr.z + p3_sr.z) / 3.0;
                    std::vector<double> side_distances = {geometryPcl.euclidean_dist_between_two_points(p1_sr, p2_sr),
                                                          geometryPcl.euclidean_dist_between_two_points(p2_sr, p3_sr),
                                                          geometryPcl.euclidean_dist_between_two_points(p3_sr, p1_sr)};
                    std::sort(side_distances.begin(), side_distances.end());
                    double max_side = side_distances[2];

                    if (geometryPcl.euclidean_dist_between_two_points(p, pcenter_sr) > 1.5 * max_side) continue;

                    double s1 = geometryPcl.triangle_area_geron(p1_sr, p2_sr, p3_sr);

                    if (!p_accepted) {
                        double s2 = geometryPcl.triangle_area_geron(p, p1_sr, p2_sr) + geometryPcl.triangle_area_geron(p, p2_sr, p3_sr) + geometryPcl.triangle_area_geron(p, p1_sr, p3_sr);
                        if (s2 < s1*coefficient_distance_filtering) {
                            p_accepted = true;
                        }
                    } else {
                        new_vertices->push_back(p);
                        break;
                    }
                }
            }


        PointCloud<PointNormal>::Ptr new_vertices_normals (new PointCloud<PointNormal> ());
        copyPointCloud(*new_vertices, *new_vertices_normals);
        std::cout << "new_vertices_normals number points: " << new_vertices_normals->size() << std::endl;

        // Apply the Poisson surface reconstruction algorithm
        PolygonMesh poisson_mesh;
        compute_poisson(new_vertices_normals, poisson_mesh, poisson_depth, solver_divide, iso_divide, poisson_point_weight);

        print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

        return poisson_mesh;
    }



    private:
        std::string input_file;
        std::string input_file_surfaces;
        std::string output_file;
        int poisson_depth = 10;
        int solver_divide = 8;
        int iso_divide = 8;
        float coefficient_distance_filtering = 1.8;
        float poisson_point_weight = 4.0f;
        float concave_hull_alpha = 8.0f;
        float concave_hull_alpha_upsample = 3.0f;
        bool convex_concave_hull = true;

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

        void upsample_by_mesh(PointCloud<PointXYZ>::Ptr &cloud_in, PolygonMesh mesh) {
            PointCloud<PointXYZ>::Ptr all_vertices_surf(new PointCloud<PointXYZ> ());
            fromPCLPointCloud2(mesh.cloud, *all_vertices_surf);

            for (std::vector<Vertices>::iterator it = mesh.polygons.begin();
                 it != mesh.polygons.end(); it++) {
                Indices vertecies = it->vertices;
                size_t size = vertecies.size();
                PointXYZ p1;
                PointXYZ p2;
                PointXYZ p3;
                PointXYZ p4;
                PointXYZ p5;
                PointXYZ pcenter;
                if (size == 3) {
                    p1 = all_vertices_surf->points[vertecies[0]];
                    p2 = all_vertices_surf->points[vertecies[1]];
                    p3 = all_vertices_surf->points[vertecies[2]];
                    pcenter.x = (p1.x + p2.x + p3.x) / 3.0;
                    pcenter.y = (p1.y + p2.y + p3.y) / 3.0;
                    pcenter.z = (p1.z + p2.z + p3.z) / 3.0;
                    PointXYZ pcenter12;
                    PointXYZ pcenter23;
                    PointXYZ pcenter31;
                    pcenter12.x = (pcenter.x + p1.x + p2.x) / 3.0;
                    pcenter23.x = (pcenter.x + p2.x + p3.x) / 3.0;
                    pcenter31.x = (pcenter.x + p3.x + p1.x) / 3.0;
                    pcenter12.y = (pcenter.y + p1.y + p2.y) / 3.0;
                    pcenter23.y = (pcenter.y + p2.y + p3.y) / 3.0;
                    pcenter31.y = (pcenter.y + p3.y + p1.y) / 3.0;
                    pcenter12.z = (pcenter.z + p1.z + p2.z) / 3.0;
                    pcenter23.z = (pcenter.z + p2.z + p3.z) / 3.0;
                    pcenter31.z = (pcenter.z + p3.z + p1.z) / 3.0;
                    cloud_in->push_back(pcenter);
                    cloud_in->push_back(pcenter12);
                    cloud_in->push_back(pcenter23);
                    cloud_in->push_back(pcenter31);
                } else if (size == 4) {
                    p1 = all_vertices_surf->points[vertecies[0]];
                    p2 = all_vertices_surf->points[vertecies[1]];
                    p3 = all_vertices_surf->points[vertecies[2]];
                    p4 = all_vertices_surf->points[vertecies[3]];
                    pcenter.x = (p1.x + p2.x + p3.x + p4.x) / 4.0;
                    pcenter.y = (p1.y + p2.y + p3.y + p4.y) / 4.0;
                    pcenter.z = (p1.z + p2.z + p3.z + p4.z) / 4.0;
                    PointXYZ pcenter12;
                    PointXYZ pcenter23;
                    PointXYZ pcenter34;
                    PointXYZ pcenter41;
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
                    cloud_in->push_back(pcenter);
                    cloud_in->push_back(pcenter12);
                    cloud_in->push_back(pcenter23);
                    cloud_in->push_back(pcenter34);
                    cloud_in->push_back(pcenter41);
                } else if (size == 5) {
                    p1 = all_vertices_surf->points[vertecies[0]];
                    p2 = all_vertices_surf->points[vertecies[1]];
                    p3 = all_vertices_surf->points[vertecies[2]];
                    p4 = all_vertices_surf->points[vertecies[3]];
                    p5 = all_vertices_surf->points[vertecies[4]];
                    pcenter.x = (p1.x + p2.x + p3.x + p4.x + p5.x) / 5.0;
                    pcenter.y = (p1.y + p2.y + p3.y + p4.y + p5.y) / 5.0;
                    pcenter.z = (p1.z + p2.z + p3.z + p4.z + p5.z) / 5.0;
                    PointXYZ pcenter12;
                    PointXYZ pcenter23;
                    PointXYZ pcenter34;
                    PointXYZ pcenter45;
                    PointXYZ pcenter51;
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
                    cloud_in->push_back(pcenter);
                    cloud_in->push_back(pcenter12);
                    cloud_in->push_back(pcenter23);
                    cloud_in->push_back(pcenter34);
                    cloud_in->push_back(pcenter45);
                    cloud_in->push_back(pcenter51);
                }
            }
        }



};


#endif //URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H
