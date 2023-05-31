# UrbanReconstruction

<img src="img/poisson_compare.png" alt="Сравнение мешей">

## Class Surface_reconstruction 

### Public Functions:
* void setInputFile(std::string file_name)
* std::string getInputFile()
* void setOutputFile(std::string file_name)
* std::string getOutputFile()
* void setShapeDetectionType(int type)
* int getShapeDetectionType()
* void setSimplificationAverageSpacing(int average_spacing)
* int getSimplificationAverageSpacing()
* void setEstimateNormalsNeighbors(int neighbors_number)
* int getEstimateNormalsNeighbors()
* void setSearchSphereRadius(FT radius)
* FT getSearchSphereRadius()
* void setMaxDistanceToPlane(FT distance)
* FT getMaxDistanceToPlane()
* void setMaxAcceptedAngle(FT angle)
* FT getMaxAcceptedAngle()
* void setMinRegionSize(std::size_t min_size)
* std::size_t getMinRegionSize()
* Surface_mesh reconstruct()

### Private Attributes:
* std::string input_file
* std::string output_file
* int shape_detection_type
* int simplification_average_spacing 
* int estimate_normals_neighbors
* CGAL::Exact_predicates_inexact_constructions_kernel::FT search_sphere_radius
* CGAL::Exact_predicates_inexact_constructions_kernel::FT max_distance_to_plane
* CGAL::Exact_predicates_inexact_constructions_kernel::FT max_accepted_angle
* std::size_t min_region_size

### Private Functions:
* void simplifyMesh(Point_set *points)
* void estimateNormals(Point_set *points)
* Surface_mesh reconstructRegionGrowing(Point_vector points)


## Class Building_reconstruction
### Public Functions:
* void setInputFile(std::string file_name)
* std::string getInputFile()
* void setInputFileSurfaces(std::string file_name)
* std::string getInputFileSurfaces()
* void setOutputFile(std::string file_name)
* std::string getOutputFile()
* void setPoissonDepth(int depth)
* int getPoissonDepth()
* void setSolverDivide(int divide)
* int getSolverDivide()
* void setIsoDivide(int divide)
* int getIsoDivide()
* void setPoissonPointWeight(float point_weight)
* float getPoissonPointWeight()
* void setConcaveHullAlpha(float alpha)
* float getConcaveHullAlpha()
* void setConcaveHullAlphaUpsample(float alpha)
* float getConcaveHullAlphaUpsample()
* void setConvexConcaveHull(bool hull_type)
* bool getConvexConcaveHull()
* PolygonMesh filter_mesh_by_mesh(PolygonMesh mesh_input, PolygonMesh mesh_filter)
* PolygonMesh reconstruct()


### Private Attributes:
* std::string input_file
* std::string input_file_surfaces
* std::string output_file
* int poisson_depth
* int solver_divide
* int iso_divide
* float coefficient_distance_filtering
* float poisson_point_weight
* float concave_hull_alpha
* float concave_hull_alpha_upsample
* bool convex_concave_hull

### Private Functions:
* void compute_poisson (const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                    int depth, int solver_divide, int iso_divide, float point_weight)
* void compute_poisson (PointCloud<PointNormal>::Ptr &input, PolygonMesh &output,
                    int depth, int solver_divide, int iso_divide, float point_weight)
* void compute_greedy_triangulation (const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                                double mu, double radius)
* void compute_hull (const PCLPointCloud2::ConstPtr &cloud_in,
                    bool convex_concave_hull,
                    float alpha,
                    PolygonMesh &mesh_out)
* void compute_hull (const PointCloud<PointXYZ>::ConstPtr &cloud_in,
                    bool convex_concave_hull,
                    float alpha,
                    PolygonMesh &mesh_out)
* void upsample_by_mesh (PointCloud<PointXYZ>::Ptr &cloud_in, PolygonMesh mesh)




