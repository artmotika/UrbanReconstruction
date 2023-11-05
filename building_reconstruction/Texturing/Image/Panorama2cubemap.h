#ifndef POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_PANORAMA2CUBEMAP_H
#define POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_PANORAMA2CUBEMAP_H

#include <chrono> // for high_resolution_clock
#include <iostream>
#include <math.h> /* modf */
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include "../../../include/rapidscv/rapidcsv.h"
#include <Eigen/Dense>
#include "../../PointClouds/Shift.h"

namespace urban_rec {

    enum DestinationType {
        Yplus = 0, Xplus = 1, Yminus = 2, Xminus = 3, Zminus = 4, Zplus = 5
    };

    class Panorama2cubemap {
    public:
        Panorama2cubemap(std::string input_image_path, std::string output_image_path, std::string directory_path,
                         bool description_file = true) {
            setImagePanorama(input_image_path);
            setInputImagePath(input_image_path);
            setOutputImagePath(output_image_path);
            setDirPath(directory_path);
            setDescriptionFileOption(description_file);
        }

        void setImagePanorama(std::string image_path);

        cv::Mat getImagePanorama();

        void setOutputImagePath(std::string image_path);

        std::string getOutputImagePath();

        void setInputImagePath(std::string image_path);

        std::string getInputImagePath();

        void setDirPath(std::string path);

        std::string getDirPath();

        void setDescriptionFileOption(bool description_file);

        bool getDescriptionFileOption();

        void setFocalLengthW(double focal_length);

        double getFocalLengthW();

        void setFocalLengthH(double focal_length);

        double getFocalLengthH();

        void setFocalLength(double focal_length_w, double focal_length_h);

        double getFocalLength();

        void setShift(double x, double y, double z);

        void setShift(shift_coord input_shift);

        shift_coord getShift();

        void setCsvFile(std::string file_path, int pColumnNameIdx=0, int pRowNameIdx=0, char separatorParams='\t');

        rapidcsv::Document getCsvFile();

        void transform();

        void transform_dir();

    private:
        cv::Mat imagePanorama;
        std::string input_image_path;
        std::string output_image_path;
        std::string dir_path;
        int input_width;
        int input_height;
        bool description_file_option;
        double focal_length_w = 1000.0;
        double focal_length_h = 1000.0;
        shift_coord shift = std::make_tuple(0, 0, 0);
        rapidcsv::Document csv_file;

        double degree_to_radian(double degree);

        float getTheta(float x, float y);

        int getIndexBeforeChar(std::string path, char sign);

        std::string getFileName(std::string file_path);

        void createDescriptionFiles(std::string output_image_subpath1);

        void createDescriptionFile(DestinationType type, std::string output_path);

    };
}

#endif //POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_PANORAMA2CUBEMAP_H
