#include "Lidar2depth.h"

void urban_rec::Lidar2depth::setInputPolygonMesh(pcl::PolygonMesh polygon_mesh) {
    input_polygon_mesh = std::make_shared<pcl::PolygonMesh>(polygon_mesh);
}

pcl::PolygonMesh urban_rec::Lidar2depth::getInputPolygonMesh() {
    return *input_polygon_mesh;
}

void urban_rec::Lidar2depth::setInputMeshFilePath(string input_mesh_file_path) {
    this->input_mesh_file_path = input_mesh_file_path;
    pcl::PolygonMesh input_mesh;
    Io_pcl::loadCloud(input_mesh_file_path, input_mesh);
    input_polygon_mesh = std::make_shared<pcl::PolygonMesh>(input_mesh);
}

string urban_rec::Lidar2depth::getInputMeshFilePath() {
    return input_mesh_file_path;
}

void urban_rec::Lidar2depth::setBaseDirPath(string base_dir_path) {
    this->base_dir_path = base_dir_path;
}

string urban_rec::Lidar2depth::getBaseDirPath() {
    return base_dir_path;
}

void urban_rec::Lidar2depth::readCamPoses(vector<Camera> *cams, vector<boost::filesystem::path> *filenames) {
    const boost::filesystem::path base_dir(base_dir_path);
    string extension(".txt");
    try {
        for (boost::filesystem::directory_iterator it(base_dir);
             it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(it->status()) &&
                boost::filesystem::extension(it->path()) == extension) {
                filenames->push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    sort(filenames->begin(), filenames->end());

    for (int i = 0; i < filenames->size(); i++) {
        Camera cam;
        urban_rec::Texturing_mapping::read_cam_pose_file((*filenames)[i].string(), cam);
        cams->push_back(cam);
    }
}

void urban_rec::Lidar2depth::fillPointCoordsSet(Camera cam, PointCoordsSet *polygons) {
    PointCloud<PointXYZ>::Ptr all_vertices(new PointCloud <PointXYZ>);
    fromPCLPointCloud2(input_polygon_mesh->cloud, *all_vertices);

    PointCloud<PointXYZ>::Ptr camera_transformed_cloud (new PointCloud<PointXYZ>());

    // get camera transform
    Eigen::Affine3f cam_trans = cam.pose;

    // transform cloud into current camera frame
    transformPointCloud (*all_vertices, *camera_transformed_cloud, cam_trans.inverse());

    // fill PointCoordsSet to calculate depth image then
    for (std::vector<Vertices>::iterator it = input_polygon_mesh->polygons.begin();
    it != input_polygon_mesh->polygons.end(); it++) {
        // iterate through all polygons in mesh
        Indices vertecies = it->vertices;
        pcl::PointXYZ p1, p2, p3, p1_original, p2_original, p3_original;
        if (vertecies.size() >= 3) {
            p1 = camera_transformed_cloud->points[vertecies[0]];
            p2 = camera_transformed_cloud->points[vertecies[1]];
            p3 = camera_transformed_cloud->points[vertecies[2]];
            p1_original = all_vertices->points[vertecies[0]];
            p2_original = all_vertices->points[vertecies[1]];
            p3_original = all_vertices->points[vertecies[2]];
        } else continue;

        /*
         * fill PointCoordsSet with polygon consisting of point XYZ coords, 2d coords in Eigen::Vector2i format,
         * inforamtion is the point projected onto the image frame
         */
        pcl::PointXY p_xy1, p_xy2, p_xy3;
        bool is_projected1 = urban_rec::Texturing_mapping::getPointUVCoords(p1, cam, p_xy1);
        bool is_projected2 = urban_rec::Texturing_mapping::getPointUVCoords(p2, cam, p_xy2);
        bool is_projected3 = urban_rec::Texturing_mapping::getPointUVCoords(p3, cam, p_xy3);
        if (is_projected1 || is_projected2 || is_projected3) {
            // translating UV coords from pcl to coords onto image frame
            int iX = int (round(p_xy1.x * cam.width));
            int iY = int (round((1-p_xy1.y) * cam.height));
//            int iY = int (round((1-p_xy1.x) * cam.width));
//            int iX = int (round((1-p_xy1.y) * cam.height));
            Eigen::Vector2i pixel1 (iX, iY);
            iX = int (round(p_xy2.x * cam.width));
            iY = int (round((1-p_xy2.y) * cam.height));
//            iY = int (round((1-p_xy2.x) * cam.width));
//            iX = int (round((1-p_xy2.y) * cam.height));
            Eigen::Vector2i pixel2 (iX, iY);
            iX = int (round(p_xy3.x * cam.width));
            iY = int (round((1-p_xy3.y) * cam.height));
//            iY = int (round((1-p_xy3.x) * cam.width));
//            iX = int (round((1-p_xy3.y) * cam.height));
            Eigen::Vector2i pixel3 (iX, iY);
            polygons->push_back(
                    make_tuple(
                            tuple<PointXYZ, PointXYZ, PointXYZ>{p1_original, p2_original, p3_original},
                            tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2i>{pixel1, pixel2, pixel3},
                            tuple<bool, bool, bool>{is_projected1, is_projected2, is_projected3}
                    ));
        }
    }
}

void urban_rec::Lidar2depth::createDepthImages() {
    vector<Camera> cams;
    vector<boost::filesystem::path> filenames;
    readCamPoses(&cams, &filenames);
    for (int i = 0; i < filenames.size(); i++ ) {
        // get current camera parameters
        Camera current_cam = cams[i];

        // create 1 channel with 16 bit image for depth image
        cv::Mat destination(int(current_cam.height), int(current_cam.width), CV_16UC1, cv::Scalar(0));

        // create PointCoordsSet and then fill it
        PointCoordsSet polygons;

        // fill PointCoordsSet to make depth image
        fillPointCoordsSet(current_cam, &polygons);

        // creating PointXYZ of current camera
        pcl::PointXYZ p_cam;
        p_cam.x = current_cam.pose(0, 3);
        p_cam.y = current_cam.pose(1, 3);
        p_cam.z = current_cam.pose(2, 3);

        // fill depth image matching PointCoordsSet
        for (auto polygon : polygons) {
            tuple<PointXYZ, PointXYZ, PointXYZ> points = get<0>(polygon);
            tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2i> pixels = get<1>(polygon);
            tuple<bool, bool, bool> is_projected = get<2>(polygon);
            // find the maximum and minimum coordinates in x and y
            vector<int> coords = {get<0>(pixels)(0), get<1>(pixels)(0), get<2>(pixels)(0)};
            std::sort(begin(coords), end(coords));
            int min_x = coords[0];
            int max_x = coords[2];
            coords = {get<0>(pixels)(1), get<1>(pixels)(1), get<2>(pixels)(1)};
            std::sort(begin(coords), end(coords));
            int min_y = coords[0];
            int max_y = coords[2];

            Eigen::Vector2i p1, p2, p3;
            p1 = get<0>(pixels);
            p2 = get<1>(pixels);
            p3 = get<2>(pixels);
            // the distances from the camera to the point of the mesh triangle
            uint16_t dist1_3d, dist2_3d, dist3_3d;
            // information is the point of the mesh triangle projected on the image
            bool is_projected1, is_projected2, is_projected3;
            is_projected1 = get<0>(is_projected);
            is_projected2 = get<1>(is_projected);
            is_projected3 = get<2>(is_projected);
            // information is the point placed behind other existing mesh
            bool is_overlayed = false;
            // draw the points belonging to the current mesh triangle
            if (is_projected1) {
                dist1_3d = static_cast<uint16_t>(Geometry_pcl::euclidean_dist_between_two_points(get<0>(points), p_cam) * 1000); //(in millimetrs)
                /*
                * We check whether the pixel is filled in, if so, then the new distance should be less,
                 * in order to vaporize the existing pixel, it is necessary not to collect invisible ones
                 * (for the camera) pixels from lidar
                */
                if (destination.at<uint16_t>(p1(1), p1(0)) == 0 || dist1_3d <= destination.at<uint16_t>(p1(1), p1(0))) {
                    destination.at<uint16_t>(p1(1), p1(0)) = dist1_3d;
                } else is_overlayed = true;
            }
            if (is_projected2) {
                dist2_3d = static_cast<uint16_t>(Geometry_pcl::euclidean_dist_between_two_points(get<1>(points), p_cam) * 1000);
                if (destination.at<uint16_t>(p2(1), p2(0)) == 0 || dist2_3d <= destination.at<uint16_t>(p2(1), p2(0))) {
                    destination.at<uint16_t>(p2(1), p2(0)) = dist2_3d;
                } else is_overlayed = true;
            }
            if (is_projected3) {
                dist3_3d = static_cast<uint16_t>(Geometry_pcl::euclidean_dist_between_two_points(get<2>(points), p_cam) * 1000);
                if (destination.at<uint16_t>(p3(1), p3(0)) == 0 || dist3_3d <= destination.at<uint16_t>(p3(1), p3(0))) {
                    destination.at<uint16_t>(p3(1), p3(0)) = dist3_3d;
                } else is_overlayed = true;
            }

            // we walk through the rectangle (min_x, min_y) to (max_x, max_y) in which contains our triangle
            if (!is_overlayed) {
                for (int x = min_x; x <= max_x; x++ ) {
                    for(int y = min_y; y <= max_y; y++) {
                        if (x >= 0 && y >= 0 && x <= current_cam.width && y <= current_cam.height) {
                            Eigen::Vector2i p (x, y);
                            if (Geometry_pcl::check_point_in_triangle(p, p1, p2, p3)) {
                                double dist1, dist2, dist3;
                                if (is_projected1) dist1 = Geometry_pcl::dist_between_two_points(p, p1);
                                else dist1 = std::numeric_limits<double>::max();
                                if (is_projected2) dist2 = Geometry_pcl::dist_between_two_points(p, p2);
                                else dist2 = std::numeric_limits<double>::max();
                                if (is_projected3) dist3 = Geometry_pcl::dist_between_two_points(p, p3);
                                else dist3 = std::numeric_limits<double>::max();
                                // the tactics of the nearest neighbor (We paint in the color of the nearest neighbor)
                                if (dist1 < dist2) {
                                    if (dist1 < dist3) {
                                        destination.at<uint16_t>(p(1), p(0)) = dist1_3d;
                                    } else {
                                        destination.at<uint16_t>(p(1), p(0)) = dist3_3d;
                                    }
                                } else {
                                    if (dist2 < dist3) {
                                        destination.at<uint16_t>(p(1), p(0)) = dist2_3d;
                                    } else {
                                        destination.at<uint16_t>(p(1), p(0)) = dist3_3d;
                                    }
                                }
                            }
                        }
                    }
                }
            } else {
                for (int x = min_x; x <= max_x; x++ ) {
                    for(int y = min_y; y <= max_y; y++) {
                        if (x >= 0 && y >= 0 && x <= current_cam.width && y <= current_cam.height) {
                            Eigen::Vector2i p (x, y);
                            /*
                             * We do not recolor pixels that are already shaded, since the triangle being checked
                             * is located behind the shaded pixels
                             */
                            if (destination.at<uint16_t>(p(1), p(0)) == 0) {
                                if (Geometry_pcl::check_point_in_triangle(p, p1, p2, p3)) {
                                    double dist1, dist2, dist3;
                                    if (is_projected1) dist1 = Geometry_pcl::dist_between_two_points(p, p1);
                                    else dist1 = std::numeric_limits<double>::max();
                                    if (is_projected2) dist2 = Geometry_pcl::dist_between_two_points(p, p2);
                                    else dist2 = std::numeric_limits<double>::max();
                                    if (is_projected3) dist3 = Geometry_pcl::dist_between_two_points(p, p3);
                                    else dist3 = std::numeric_limits<double>::max();
                                    // the tactics of the nearest neighbor (We paint in the color of the nearest neighbor)
                                    if (dist1 < dist2) {
                                        if (dist1 < dist3) {
                                            destination.at<uint16_t>(p(1), p(0)) = dist1_3d;
                                        } else {
                                            destination.at<uint16_t>(p(1), p(0)) = dist3_3d;
                                        }
                                    } else {
                                        if (dist2 < dist3) {
                                            destination.at<uint16_t>(p(1), p(0)) = dist2_3d;
                                        } else {
                                            destination.at<uint16_t>(p(1), p(0)) = dist3_3d;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

//        // take image
//        std::string image_path = filenames[i].stem().string() + ".jpg";
//
//        // get image name
//        dot_index = path_utils::getIndexBeforeChar(image_path, '.');
//        image_name = output_image_path.substr(0, dot_index);

        // save depth image
        std::ostringstream oss;
        oss << base_dir_path << filenames[i].stem().string() << "_depth_image" << ".png";
        cout << oss.str() << endl;
        cv::imwrite(oss.str(), destination);
    }
}