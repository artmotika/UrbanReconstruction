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
        void setFilterRadius(double radius) {
            filter_radius = radius;
        }
        double getFilterRadius() {
            return filter_radius;
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

        PolygonMesh filter_mesh_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius) {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();

            PointCloud<PointXYZ>::Ptr points_filter_xyz (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*points_filter, *points_filter_xyz);

            std::vector<Vertices> new_polygons_sr;

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_input.cloud, *all_vertices);

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
                for (PointXYZ p : *points_filter_xyz) {
                    if (geometryPcl.point_in_radius(pcenter, p, filter_radius)) {
                        new_polygons_sr.push_back(*it1);
                        break;
                    }
                }
            }

            mesh_input.polygons = new_polygons_sr;
            print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

            return mesh_input;
        }

        PolygonMesh filter_mesh_poisson_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius) {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();

            PointCloud<PointXYZ>::Ptr points_filter_xyz (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*points_filter, *points_filter_xyz);

            std::vector<Vertices> new_polygons_sr;
            std::vector<Vertices> new_polygons_no_extra;
            std::vector<Vertices> new_polygons_filtering;

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_input.cloud, *all_vertices);

            PCLPointCloud2::Ptr filtered_cloud (new PCLPointCloud2);

            PointCloud<PointXYZ>::Ptr points_filtered_xyz (new PointCloud<PointXYZ> ());

            int polygons_left = mesh_input.polygons.size();
            std::cout << "number polygons: " << polygons_left << std::endl;

            bool first_try = true;

            size_t array_size = all_vertices->points.size();
            unsigned short* vertex_indices = (unsigned short*)malloc(array_size * sizeof(unsigned short));
            memset(vertex_indices, 0, array_size * sizeof(unsigned short));

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
                for (PointXYZ p : *points_filter_xyz) {
                    if (geometryPcl.point_in_radius(pcenter, p, filter_radius)) {
                        vertex_indices[vertecies[0]] = vertex_indices[vertecies[0]] + 1;
                        vertex_indices[vertecies[1]] = vertex_indices[vertecies[1]] + 1;
                        vertex_indices[vertecies[2]] = vertex_indices[vertecies[2]] + 1;
                        new_polygons_sr.push_back(*it1);
                        break;
                    }
                }
            }

            new_polygons_filtering = new_polygons_sr;
            bool is_deleted_polygon;

            do {
                is_deleted_polygon = false;
                std::vector<Vertices> temp_new_polygons_filtering;
                for (std::vector<Vertices>::iterator it = new_polygons_filtering.begin(); it != new_polygons_filtering.end(); it++) {
                    Indices vertecies = it->vertices;
                    PointXYZ p1;
                    PointXYZ p2;
                    PointXYZ p3;
                    int index1 = vertecies[0];
                    int index2 = vertecies[1];
                    int index3 = vertecies[2];
                    if (vertecies.size() >= 3) {
                        p1 = all_vertices->points[index1];
                        p2 = all_vertices->points[index2];
                        p3 = all_vertices->points[index3];
                    } else continue;
                    unsigned short count1 = vertex_indices[index1];
                    unsigned short count2 = vertex_indices[index2];
                    unsigned short count3 = vertex_indices[index3];
                    double side1 = geometryPcl.euclidean_dist_between_two_points(p1, p2);
                    double side2 = geometryPcl.euclidean_dist_between_two_points(p2, p3);
                    double side3 = geometryPcl.euclidean_dist_between_two_points(p1, p3);

                    double s = geometryPcl.triangle_area_geron(p1, p2, p3);
                    if (s > 0.3 || side1 >= 0.6 || side2 >= 0.6 || side3 >= 0.6) {
                        if ((count1 >= 1 && count1 <= 3) || (count2 >= 1 && count2 <= 3) || (count3 >= 1 && count3 <= 3)) {
                            vertex_indices[index1] = vertex_indices[index1] - 1;
                            vertex_indices[index2] = vertex_indices[index2] - 1;
                            vertex_indices[index3] = vertex_indices[index3] - 1;
                            is_deleted_polygon = true;
                            continue;
                        }
                    }
                    if (s > 0.05 || side1 >= 0.2 || side2 >= 0.2 || side3 >= 0.2) {
                        if ((count1 >= 1 && count1 <= 2) || (count2 >= 1 && count2 <= 2) || (count3 >= 1 && count3 <= 2)
                        || ((count1 >= 1 && count1 <= 3) && (count2 >= 1 && count2 <= 3))
                        || ((count1 >= 1 && count1 <= 3) && (count3 >= 1 && count3 <= 3))
                        || ((count2 >= 1 && count2 <= 3) && (count3 >= 1 && count3 <= 3))
                        || ((count1 >= 1 && count1 <= 2) && (count2 >= 1 && count2 <= 4))
                        || ((count1 >= 1 && count1 <= 4) && (count2 >= 1 && count2 <= 2))
                        || ((count1 >= 1 && count1 <= 2) && (count3 >= 1 && count3 <= 4))
                        || ((count1 >= 1 && count1 <= 4) && (count3 >= 1 && count3 <= 2))
                        || ((count2 >= 1 && count2 <= 2) && (count3 >= 1 && count3 <= 4))
                        || ((count2 >= 1 && count2 <= 4) && (count3 >= 1 && count3 <= 2))) {
                            vertex_indices[index1] = vertex_indices[index1] - 1;
                            vertex_indices[index2] = vertex_indices[index2] - 1;
                            vertex_indices[index3] = vertex_indices[index3] - 1;
                            is_deleted_polygon = true;
                            continue;
                        } else {
                            temp_new_polygons_filtering.push_back(*it);
                            continue;
                        }
                    } else {
                        if (count1 == 1 || count2 == 1 || count3 == 1) {
                            vertex_indices[index1] = vertex_indices[index1] - 1;
                            vertex_indices[index2] = vertex_indices[index2] - 1;
                            vertex_indices[index3] = vertex_indices[index3] - 1;
                            is_deleted_polygon = true;
                            continue;
                        } else {
                            temp_new_polygons_filtering.push_back(*it);
                            continue;
                        }
                    }

                    new_polygons_no_extra.push_back(*it);
                }
                new_polygons_filtering = temp_new_polygons_filtering;
            } while (is_deleted_polygon);

            for (std::vector<Vertices>::iterator it = new_polygons_filtering.begin(); it != new_polygons_filtering.end(); it++) {
                new_polygons_no_extra.push_back(*it);
            }

            free(vertex_indices);
            mesh_input.polygons = new_polygons_no_extra;
            print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

            return mesh_input;
        }

        PointCloud<PointXYZ>::Ptr filter_points_by_points(PolygonMesh mesh_input, const PCLPointCloud2::ConstPtr &points_filter, double filter_radius) {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();

            PointCloud<PointXYZ>::Ptr points_filter_xyz (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*points_filter, *points_filter_xyz);

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_input.cloud, *all_vertices);

            PointCloud<PointXYZ>::Ptr points_filtered_xyz (new PointCloud<PointXYZ> ());

            int points_left = all_vertices->size();
            std::cout << "number points: " << points_left << std::endl;

            for (PointXYZ p : *all_vertices) {
                points_left --;
                if (points_left % 10000 == 0) std::cout << "number points left: " << points_left << std::endl;

                for (PointXYZ p_filter : *points_filter_xyz) {
                    if (geometryPcl.point_in_radius(p, p_filter, filter_radius)) {
                        points_filtered_xyz->push_back(p);
                        break;
                    }
                }
            }

            print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

            return points_filtered_xyz;
        }

        PointCloud<PointXYZ>::Ptr filter_points_by_mesh(PolygonMesh mesh_input) {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(mesh_input.cloud, *all_vertices);

            PointCloud<PointXYZ>::Ptr points_filtered_xyz (new PointCloud<PointXYZ> ());

            size_t array_size = all_vertices->points.size();
            unsigned short* vertex_indices = (unsigned short*)malloc(array_size * sizeof(unsigned short));
            memset(vertex_indices, 0, array_size * sizeof(unsigned short));

            for (std::vector<Vertices>::iterator it = mesh_input.polygons.begin();
                 it != mesh_input.polygons.end(); it++) {
                Indices vertecies = it->vertices;
                int index1 = vertecies[0];
                int index2 = vertecies[1];
                int index3 = vertecies[2];
                vertex_indices[index1] = vertex_indices[index1] + 1;
                vertex_indices[index2] = vertex_indices[index2] + 1;
                vertex_indices[index3] = vertex_indices[index3] + 1;
            }

            for (std::vector<Vertices>::iterator it = mesh_input.polygons.begin();
                 it != mesh_input.polygons.end(); it++) {
                Indices vertecies = it->vertices;
                int index1 = vertecies[0];
                int index2 = vertecies[1];
                int index3 = vertecies[2];
                if (vertex_indices[index1] != 0) {
                    points_filtered_xyz->push_back(all_vertices->points[index1]);
                }
                if (vertex_indices[index2] != 0) {
                    points_filtered_xyz->push_back(all_vertices->points[index2]);
                }
                if (vertex_indices[index3] != 0) {
                    points_filtered_xyz->push_back(all_vertices->points[index3]);
                }
                vertex_indices[index1] = 0;
                vertex_indices[index2] = 0;
                vertex_indices[index3] = 0;
            }

            free(vertex_indices);
            print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

            return points_filtered_xyz;
        }

        void upsample_mesh(PointCloud<PointXYZ>::Ptr &cloud_in, PolygonMesh mesh, double max_polygon_size,
                           double max_polygon_side) {
            std::vector<polygon_struct> new_polygons{};
            Geometry_pcl geometryPcl;

            PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud<PointXYZ> ());
            fromPCLPointCloud2(mesh.cloud, *all_vertices);

            for (std::vector<Vertices>::iterator it = mesh.polygons.begin();
                 it != mesh.polygons.end(); it++) {
                Indices vertecies = it->vertices;
                size_t size = vertecies.size();
                PointXYZ p1 = all_vertices->points[vertecies[0]];
                PointXYZ p2 = all_vertices->points[vertecies[1]];
                PointXYZ p3 = all_vertices->points[vertecies[2]];
                double area = geometryPcl.triangle_area_geron(p1, p2, p3);
                double side = std::max(std::max(geometryPcl.euclidean_dist_between_two_points(p1, p2),
                                  geometryPcl.euclidean_dist_between_two_points(p2, p3)),
                                  geometryPcl.euclidean_dist_between_two_points(p1, p3));
                if (area > max_polygon_size || side > max_polygon_side) {
                    new_polygons.push_back({p1, p2, p3});
                }
            }

            while (!new_polygons.empty()) {
                std::vector<polygon_struct> new_polygons_adding{};
                for (polygon_struct polygon : new_polygons) {
                    PointXYZ pcenter;
                    PointXYZ pcenter1;
                    PointXYZ pcenter2;
                    PointXYZ pcenter3;
                    PointXYZ p1 = polygon.p1;
                    PointXYZ p2 = polygon.p2;
                    PointXYZ p3 = polygon.p3;
                    pcenter.x = (p1.x + p2.x + p3.x) / 3.0;
                    pcenter.y = (p1.y + p2.y + p3.y) / 3.0;
                    pcenter.z = (p1.z + p2.z + p3.z) / 3.0;
                    pcenter1.x = (p1.x + p2.x) / 2.0;
                    pcenter2.x = (p2.x + p3.x) / 2.0;
                    pcenter3.x = (p1.x + p3.x) / 2.0;
                    pcenter1.y = (p1.y + p2.y) / 2.0;
                    pcenter2.y = (p2.y + p3.y) / 2.0;
                    pcenter3.y = (p1.y + p3.y) / 2.0;
                    pcenter1.z = (p1.z + p2.z) / 2.0;
                    pcenter2.z = (p2.z + p3.z) / 2.0;
                    pcenter3.z = (p1.z + p3.z) / 2.0;
                    double side_p1_p2 = geometryPcl.euclidean_dist_between_two_points(p1, p2);
                    double side_p2_p3 = geometryPcl.euclidean_dist_between_two_points(p2, p3);
                    double side_p1_p3 = geometryPcl.euclidean_dist_between_two_points(p1, p3);

                    double area1 = geometryPcl.triangle_area_geron(p1, pcenter1, pcenter);
                    double side11 = geometryPcl.euclidean_dist_between_two_points(p1, pcenter1);
                    double side12 = geometryPcl.euclidean_dist_between_two_points(pcenter1, pcenter);
                    double side13 = geometryPcl.euclidean_dist_between_two_points(p1, pcenter);
                    double side1 = std::max(std::max(side11, side12), side13);

                    double area2 = geometryPcl.triangle_area_geron(pcenter1, p2, pcenter);
                    double side21 = geometryPcl.euclidean_dist_between_two_points(pcenter1, p2);
                    double side22 = geometryPcl.euclidean_dist_between_two_points(p2, pcenter);
                    double side23 = geometryPcl.euclidean_dist_between_two_points(pcenter1, pcenter);
                    double side2 = std::max(std::max(side21, side22), side23);


                    double area3 = geometryPcl.triangle_area_geron(p1, pcenter3, pcenter);
                    double side31 = geometryPcl.euclidean_dist_between_two_points(p1, pcenter3);
                    double side32 = geometryPcl.euclidean_dist_between_two_points(pcenter3, pcenter);
                    double side33 = geometryPcl.euclidean_dist_between_two_points(p1, pcenter);
                    double side3 = std::max(std::max(side31, side32), side33);

                    double area4 = geometryPcl.triangle_area_geron(pcenter3, p3, pcenter);
                    double side41 = geometryPcl.euclidean_dist_between_two_points(pcenter3, p3);
                    double side42 = geometryPcl.euclidean_dist_between_two_points(p3, pcenter);
                    double side43 = geometryPcl.euclidean_dist_between_two_points(pcenter3, pcenter);
                    double side4 = std::max(std::max(side41, side42), side43);


                    double area5 = geometryPcl.triangle_area_geron(pcenter2, p3, pcenter);
                    double side51 = geometryPcl.euclidean_dist_between_two_points(pcenter2, p3);
                    double side52 = geometryPcl.euclidean_dist_between_two_points(p3, pcenter);
                    double side53 = geometryPcl.euclidean_dist_between_two_points(pcenter2, pcenter);
                    double side5 = std::max(std::max(side51, side52), side53);

                    double area6 = geometryPcl.triangle_area_geron(p2, pcenter2, pcenter);
                    double side61 = geometryPcl.euclidean_dist_between_two_points(p2, pcenter2);
                    double side62 = geometryPcl.euclidean_dist_between_two_points(pcenter2, pcenter);
                    double side63 = geometryPcl.euclidean_dist_between_two_points(p2, pcenter);
                    double side6 = std::max(std::max(side61, side62), side63);

                    cloud_in->push_back(pcenter);

                    if (area1 > max_polygon_size || side1 > max_polygon_side) {
                        if (side_p1_p2 > max_polygon_side) {
                            new_polygons_adding.push_back({p1, pcenter1, pcenter});
                            if (area2 > max_polygon_size || side2 > max_polygon_side) {
                                new_polygons_adding.push_back({pcenter1, p2, pcenter});
                            }
                        } else {
                            new_polygons_adding.push_back({p1, p2, pcenter});
                        }
                    }
                    if (area3 > max_polygon_size || side3 > max_polygon_side) {
                        if (side_p1_p3 > max_polygon_side) {
                            new_polygons_adding.push_back({p1, pcenter3, pcenter});
                            if (area4 > max_polygon_size || side4 > max_polygon_side) {
                                new_polygons_adding.push_back({pcenter3, p3, pcenter});
                            }
                        } else {
                            new_polygons_adding.push_back({p1, p3, pcenter});
                        }
                    }

                    if (area5 > max_polygon_size || side5 > max_polygon_side) {
                        if (side_p2_p3 > max_polygon_side) {
                            new_polygons_adding.push_back({pcenter2, p3, pcenter});
                            if (area6 > max_polygon_size || side6 > max_polygon_side) {
                                new_polygons_adding.push_back({p2, pcenter2, pcenter});
                            }
                        } else {
                            new_polygons_adding.push_back({p2, p3, pcenter});
                        }
                    }
                }
                new_polygons = new_polygons_adding;
            }
        }

        std::pair<unsigned long, double> calculate_repeatability_metric(PCLPointCloud2::Ptr cloud_ideal, PCLPointCloud2::Ptr cloud_repeat,
                                              double max_mistake, std::string cloud_not_repeat_path) {
            Geometry_pcl geometryPcl;
            Io_pcl IO;
            double sum = 0.0;
            unsigned long number_points_not_repeated = 0;
            PointCloud<PointXYZ>::Ptr xyz_cloud_ideal (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*cloud_ideal, *xyz_cloud_ideal);
            PointCloud<PointXYZ>::Ptr xyz_cloud_repeat (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*cloud_repeat, *xyz_cloud_repeat);
            PointCloud<PointXYZ>::Ptr xyz_cloud_not_repeat (new PointCloud<PointXYZ> ());
            unsigned long number_points = xyz_cloud_repeat->size();
            unsigned long number_points_left = number_points;

            for (PointXYZ p_repeat : *xyz_cloud_repeat) {
                double min_dist_p_repeat = std::numeric_limits<double>::infinity();
                number_points_left --;
                if (number_points_left % 10000 == 0) std::cout << "number points left: " << number_points_left << std::endl;
                for (PointXYZ p_ideal : *xyz_cloud_ideal) {
                    if (min_dist_p_repeat < abs(p_repeat.x - p_ideal.x) || min_dist_p_repeat < abs(p_repeat.y - p_ideal.y)
                        || min_dist_p_repeat < abs(p_repeat.z - p_ideal.z)) continue;
                    double dist = geometryPcl.euclidean_dist_between_two_points(p_repeat, p_ideal);
                    if (min_dist_p_repeat > dist) {
                        min_dist_p_repeat = dist;
                    }
                }
                if (min_dist_p_repeat > max_mistake) {
                    sum += pow(min_dist_p_repeat + 1 - max_mistake, 2);
                    number_points_not_repeated ++;
                    xyz_cloud_not_repeat->push_back(p_repeat);
                }
            }
            IO.saveCloud(cloud_not_repeat_path, *xyz_cloud_not_repeat);
            return std::make_pair(number_points_not_repeated, std::sqrt(sum / number_points));
        }

        std::pair<unsigned long, double> calculate_hole_metric(PCLPointCloud2::Ptr cloud_ideal, PCLPointCloud2::Ptr cloud_repeat, double max_mistake,
                                     std::string cloud_hole_path) {
            Geometry_pcl geometryPcl;
            Io_pcl IO;
            unsigned long number_hole_points = 0;
            double sum = 0.0;
            PointCloud<PointXYZ>::Ptr xyz_cloud_ideal (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (*cloud_ideal, *xyz_cloud_ideal);
            PointCloud<PointXYZ>::Ptr xyz_cloud_repeat(new PointCloud<PointXYZ>);
            fromPCLPointCloud2(*cloud_repeat, *xyz_cloud_repeat);

            unsigned long number_points = xyz_cloud_ideal->size();
            unsigned long number_points_left = number_points;

            PointCloud<PointXYZ>::Ptr xyz_cloud_hole (new PointCloud<PointXYZ> ());

            for (PointXYZ p_ideal : *xyz_cloud_ideal) {
                double min_dist_p_ideal = std::numeric_limits<double>::infinity();
                number_points_left --;
                if (number_points_left % 10000 == 0) std::cout << "number points left: " << number_points_left << std::endl;
                for (PointXYZ p_repeat : *xyz_cloud_repeat) {
                    if (min_dist_p_ideal < abs(p_repeat.x - p_ideal.x) || min_dist_p_ideal < abs(p_repeat.y - p_ideal.y)
                        || min_dist_p_ideal < abs(p_repeat.z - p_ideal.z)) continue;
                    double dist = geometryPcl.euclidean_dist_between_two_points(p_ideal, p_repeat);
                    if (min_dist_p_ideal > dist) {
                        min_dist_p_ideal = dist;
                    }
                }

                if (min_dist_p_ideal > max_mistake) {
                    sum += pow(min_dist_p_ideal + 1 - max_mistake, 2);
                    number_hole_points ++;
                    xyz_cloud_hole->push_back(p_ideal);
                }
            }

            IO.saveCloud(cloud_hole_path, *xyz_cloud_hole);
            return std::make_pair(number_hole_points, std::sqrt(sum / number_points));
        }

        PolygonMesh reconstruct() {
            TicToc tt;
            Io_pcl IO;
            Geometry_pcl geometryPcl;
            tt.tic ();
            PCLPointCloud2::Ptr input_cloud (new PCLPointCloud2);
            IO.loadCloud(input_file, *input_cloud); // "../data/Construction_home_plane.pcd"

            PolygonMesh surfaces_mesh;
            IO.loadCloud(input_file_surfaces, surfaces_mesh); //"../data/Construction_home_plane.ply"

            PointCloud<PointXYZ>::Ptr cloud_surfaces (new PointCloud<PointXYZ> ());
            fromPCLPointCloud2 (surfaces_mesh.cloud, *cloud_surfaces);

            PolygonMesh hull_mesh;
            compute_hull(cloud_surfaces, convex_concave_hull, concave_hull_alpha_upsample, hull_mesh);
            IO.saveCloud ("../data/hull_mesh_plane.ply", hull_mesh);

            PointCloud<PointXYZ>::Ptr upsample_cloud (new PointCloud<PointXYZ> ());
            copyPointCloud(*cloud_surfaces, *upsample_cloud);
            IO.saveCloud ("../data/hull_mesh1_cloud1.ply", *upsample_cloud);

            upsample_by_mesh(upsample_cloud, surfaces_mesh);

            IO.saveCloud ("../data/hull_mesh1_cloud2.ply", *upsample_cloud);
            PolygonMesh upsample_hull_mesh;
            compute_hull(upsample_cloud, convex_concave_hull, concave_hull_alpha_upsample, upsample_hull_mesh);
            IO.saveCloud ("../data/hull_mesh1.ply", upsample_hull_mesh);

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

    PolygonMesh reconstruct2() {
        TicToc tt;
        Io_pcl IO;
        Geometry_pcl geometryPcl;
        tt.tic ();
        PCLPointCloud2::Ptr input_cloud (new PCLPointCloud2);
        IO.loadCloud(input_file, *input_cloud); // "../data/Construction_home_plane.pcd"

        PolygonMesh surfaces_mesh;
        IO.loadCloud(input_file_surfaces, surfaces_mesh); //"../data/Construction_home_plane.ply"

        PointCloud<PointNormal>::Ptr new_vertices(new PointCloud<PointNormal>);

        PointCloud<PointNormal>::Ptr all_vertices(new PointCloud<PointNormal>);
        fromPCLPointCloud2(*input_cloud, *all_vertices);

        PointCloud<PointXYZ>::Ptr all_vertices_plane(new PointCloud<PointXYZ>);
        fromPCLPointCloud2(surfaces_mesh.cloud, *all_vertices_plane);

        int number_points = all_vertices->size();

        for (PointNormal p : *all_vertices) {
            number_points --;
            if (number_points % 10000 == 0) std::cout << "number points left: " << number_points << std::endl;
            for (PointXYZ p_plane : *all_vertices_plane) {
                if (geometryPcl.point_in_radius(p, p_plane, filter_radius)) {
                    new_vertices->push_back(p);
                    break;
                }
            }
        }

        IO.saveCloudPCD("../data/new_vertices.pcd", *new_vertices);

        // Apply the Poisson surface reconstruction algorithm
        PolygonMesh poisson_mesh;
        compute_poisson(new_vertices, poisson_mesh, poisson_depth, solver_divide, iso_divide, poisson_point_weight);
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
        float poisson_point_weight = 4.0f;
        float coefficient_distance_filtering = 1.8;
        float concave_hull_alpha = 8.0f;
        float concave_hull_alpha_upsample = 3.0f;
        bool convex_concave_hull = true;
        double filter_radius = 0.5;

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
