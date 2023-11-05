#include "Panorama2cubemap.h"

using namespace std;
using namespace urban_rec;

double Panorama2cubemap::degree_to_radian(double degree) {
    return degree * M_PI / 180;
}

float Panorama2cubemap::getTheta(float x, float y) {
    float rtn = 0;
    if (y < 0) {
        rtn = atan2(y, x) * -1;
    } else {
        rtn = M_PI + (M_PI - atan2(y, x));
    }
    return rtn;
}

int Panorama2cubemap::getIndexBeforeChar(std::string path, char sign) {
    unsigned int path_len = path.size();
    for (int i = path_len - 1; i >= 0; i--) {
        if (path[i] == sign) return i;
    }
    std::ostringstream oss;
    oss << "path argument in Panorama2cubemap::getIndexBeforeChar(std::string path, char sign) doesn't have a " << sign
        << " :  " << path << "!\n";
    throw invalid_argument(oss.str());
}

void Panorama2cubemap::setImagePanorama(std::string image_path) {
    imagePanorama = cv::imread(image_path, cv::IMREAD_COLOR);
    input_width = imagePanorama.cols;
    input_height = imagePanorama.rows;
    float side_ratio = input_width / input_height;
    if (side_ratio != 2) {
        std::ostringstream oss;
        oss << "Input imagePanorama must have width 8000px and height 4000px,"
               " but have width " << input_width << "and height " << input_height << "!\n";
        throw std::domain_error(oss.str());
    }
}

cv::Mat Panorama2cubemap::getImagePanorama() {
    return imagePanorama;
}

void Panorama2cubemap::setInputImagePath(std::string image_path) {
    input_image_path = image_path;
}

std::string Panorama2cubemap::getInputImagePath() {
    return input_image_path;
}

void Panorama2cubemap::setOutputImagePath(std::string image_path) {
    output_image_path = image_path;
}

std::string Panorama2cubemap::getOutputImagePath() {
    return output_image_path;
}

void Panorama2cubemap::setDirPath(std::string path) {
    dir_path = path;
}

std::string Panorama2cubemap::getDirPath() {
    return dir_path;
}

void Panorama2cubemap::setDescriptionFileOption(bool description_file) {
    description_file_option = description_file;
}

bool Panorama2cubemap::getDescriptionFileOption() {
    return description_file_option;
}

void Panorama2cubemap::setFocalLengthW(double focal_length) {
    focal_length_w = focal_length;
}

double Panorama2cubemap::getFocalLengthW() {
    return focal_length_w;
}

void Panorama2cubemap::setFocalLengthH(double focal_length) {
    focal_length_h = focal_length;
}

double Panorama2cubemap::getFocalLengthH() {
    return focal_length_h;
}

void Panorama2cubemap::setFocalLength(double focal_length_width, double focal_length_height) {
    focal_length_w = focal_length_width;
    focal_length_h = focal_length_height;
}

double Panorama2cubemap::getFocalLength() {
    return focal_length_h;
}

void Panorama2cubemap::setShift(double x, double y, double z) {
    shift = make_tuple(x, y, z);
}

void Panorama2cubemap::setShift(shift_coord input_shift) {
    shift = input_shift;
}

shift_coord Panorama2cubemap::getShift() {
    return shift;
}

void Panorama2cubemap::setCsvFile(std::string file_path, int pColumnNameIdx, int pRowNameIdx, char separatorParams) {
    csv_file = rapidcsv::Document(file_path, rapidcsv::LabelParams(pColumnNameIdx, pRowNameIdx),
                                  rapidcsv::SeparatorParams(separatorParams));
}

rapidcsv::Document Panorama2cubemap::getCsvFile() {
    return csv_file;
}

void Panorama2cubemap::createDescriptionFiles(std::string output_image_subpath1) {
    std::ostringstream oss;
    oss << output_image_subpath1 << "Yplus" << ".txt";
    createDescriptionFile(DestinationType::Yplus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Xplus" << ".txt";
    createDescriptionFile(DestinationType::Xplus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Yminus" << ".txt";
    createDescriptionFile(DestinationType::Yminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Xminus" << ".txt";
    createDescriptionFile(DestinationType::Xminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Zminus" << ".txt";
    createDescriptionFile(DestinationType::Zminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Zplus" << ".txt";
    createDescriptionFile(DestinationType::Zplus, oss.str());
}

std::string Panorama2cubemap::getFileName(std::string file_path) {
    int slash_index = getIndexBeforeChar(file_path, '/');
    int dot_index = getIndexBeforeChar(file_path, '.');
    int str_length = dot_index - slash_index - 1;
    return file_path.substr(slash_index + 1, str_length);
}

void Panorama2cubemap::createDescriptionFile(DestinationType type, std::string output_path) {
    double image_width = input_width / 4;
    double image_height = input_width / 4;
    double param;
    double radian;
    std::ofstream output_file(output_path);
    std::string image_name = getFileName(input_image_path);
    double projectedX = csv_file.GetCell<double>("projectedX[m]", image_name) + std::get<0>(shift);
    double projectedY = csv_file.GetCell<double>("projectedY[m]", image_name) + std::get<1>(shift);
    double projectedZ = csv_file.GetCell<double>("projectedZ[m]", image_name) + std::get<2>(shift);
    double heading = csv_file.GetCell<double>("heading[deg]", image_name);
    double pitch = csv_file.GetCell<double>("pitch[deg]", image_name);
    double roll = csv_file.GetCell<double>("roll[deg]", image_name);
    output_file << projectedX << " " << projectedY << " " << projectedZ << std::endl;

    Eigen::Matrix3d m;
    Eigen::Matrix3d rm;
    Eigen::AngleAxisd rollAngle;
    Eigen::AngleAxisd headingAngle;
    Eigen::AngleAxisd pitchAngle;

    double right_angle = degree_to_radian(90);

    switch (type) {
        case DestinationType::Yplus:
            param = right_angle;
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll) + param, Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) - right_angle, Eigen::Vector3d::UnitX());
            break;
        case DestinationType::Xplus:
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll), Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) - right_angle, Eigen::Vector3d::UnitX());
            break;
        case DestinationType::Yminus:
            param = -right_angle;
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll) + param, Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) - right_angle, Eigen::Vector3d::UnitX());
            break;
        case DestinationType::Xminus:
            param = 2 * right_angle;
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll) + param, Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) - right_angle, Eigen::Vector3d::UnitX());
            break;
        case DestinationType::Zminus:
            param = right_angle;
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll), Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) + param - right_angle, Eigen::Vector3d::UnitX());
            break;
        case DestinationType::Zplus:
            param = -right_angle;
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll), Eigen::Vector3d::UnitZ());
            headingAngle = Eigen::AngleAxisd(degree_to_radian(heading) + 2 * right_angle, Eigen::Vector3d::UnitY());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch) + param - right_angle, Eigen::Vector3d::UnitX());
            break;
        default:
            output_file.close();
            std::ostringstream oss;
            oss << "DestinationType type" << type << " is incorrect!\n";
            throw invalid_argument(oss.str());
    }
    Eigen::Quaternion<double> q = rollAngle * headingAngle * pitchAngle;
    m = q.matrix();
    output_file << m.coeff(0, 0) << " " << m.coeff(0, 1) << " " << m.coeff(0, 2) << std::endl;
    output_file << m.coeff(1, 0) << " " << m.coeff(1, 1) << " " << m.coeff(1, 2) << std::endl;
    output_file << m.coeff(2, 0) << " " << m.coeff(2, 1) << " " << m.coeff(2, 2) << std::endl;
    output_file << focal_length_w << std::endl;
    output_file << focal_length_h << std::endl;
    output_file << image_width << std::endl;
    output_file << image_height << std::endl;

    output_file.close();
}

void Panorama2cubemap::transform() {
    const int sqr = imagePanorama.cols / 4.0;
    const int output_width = sqr * 3;
    const int output_height = sqr * 2;

    // Placeholder image for the result
    vector <cv::Mat> destinations;
    for (int i = 0; i < 6; i++) {
        cv::Mat destination(sqr, sqr, CV_8UC3, cv::Scalar(255, 255, 255));
        destinations.push_back(destination);
    }

    auto begin = chrono::high_resolution_clock::now();

#pragma omp parallel for
    for (int j = 0; j < output_width; j++) {
        // #pragma omp parallel for
        for (int i = 0; i < output_height; i++) {
            DestinationType destImageType;
            float tx = 0.0;
            float ty = 0.0;
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;

            if (i < sqr + 1) {   // top half
                if (j < sqr + 1) { // top left box [Y+]
                    destImageType = DestinationType::Yplus;
                    tx = j;
                    ty = i;
                    x = tx - 0.5 * sqr;
                    y = 0.5 * sqr;
                    z = ty - 0.5 * sqr;
                } else if (j < 2 * sqr + 1) { // top middle [X+]
                    destImageType = DestinationType::Xplus;
                    tx = j - sqr;
                    ty = i;
                    x = 0.5 * sqr;
                    y = (tx - 0.5 * sqr) * -1;
                    z = ty - 0.5 * sqr;
                } else { // top right [Y-]
                    destImageType = DestinationType::Yminus;
                    tx = j - sqr * 2;
                    ty = i;
                    x = (tx - 0.5 * sqr) * -1;
                    y = -0.5 * sqr;
                    z = ty - 0.5 * sqr;
                }
            } else {             // bottom half
                if (j < sqr + 1) { // bottom left box [X-]
                    destImageType = DestinationType::Xminus;
                    tx = j;
                    ty = i - sqr;
                    x = int(-0.5 * sqr);
                    y = int(tx - 0.5 * sqr);
                    z = int(ty - 0.5 * sqr);
                } else if (j < 2 * sqr + 1) { // bottom middle [Z-]
                    destImageType = DestinationType::Zminus;
                    tx = j - sqr;
                    ty = i - sqr;
                    x = (ty - 0.5 * sqr) * -1;
                    y = (tx - 0.5 * sqr) * -1;
                    z = 0.5 * sqr; // was -0.5 might be due to phi being reversed
                } else { // bottom right [Z+]
                    destImageType = DestinationType::Zplus;
                    tx = j - sqr * 2;
                    ty = i - sqr;
                    x = ty - 0.5 * sqr;
                    y = (tx - 0.5 * sqr) * -1;
                    z = -0.5 * sqr; // was +0.5 might be due to phi being reversed
                }
            }

            // now find out the polar coordinates
            float rho = sqrt(x * x + y * y + z * z);
            float normTheta = getTheta(x, y) / (2 * M_PI); // /(2*M_PI) normalise theta
            float normPhi = (M_PI - acos(z / rho)) / M_PI; // /M_PI normalise phi

            // use this for coordinates
            float iX = normTheta * input_width;
            float iY = normPhi * input_height;

            // catch possible overflows
            if (iX >= input_width) {
                iX = iX - (input_width);
            }
            if (iY >= input_height) {
                iY = iY - (input_height);
            }

            destinations[destImageType].at<cv::Vec3b>(ty, tx) = imagePanorama.at<cv::Vec3b>(int(iY), int(iX));
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - begin;

    std::ostringstream oss;
    std::string output_image_subpath1;
    std::string output_image_subpath2;
    int dot_index = getIndexBeforeChar(output_image_path, '.');
    output_image_subpath1 = output_image_path.substr(0, dot_index);
    output_image_subpath2 = output_image_path.substr(dot_index, output_image_path.size());

    oss << output_image_subpath1 << "Yplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Yplus]);
    oss.str("");
    oss << output_image_subpath1 << "Xplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Xplus]);
    oss.str("");
    oss << output_image_subpath1 << "Yminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Yminus]);
    oss.str("");
    oss << output_image_subpath1 << "Xminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Xminus]);
    oss.str("");
    oss << output_image_subpath1 << "Zminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Zminus]);
    oss.str("");
    oss << output_image_subpath1 << "Zplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Zplus]);

    if (description_file_option) createDescriptionFiles(output_image_subpath1);

    cout << "Processing time: " << diff.count() << " s" << endl;
}

void Panorama2cubemap::transform_dir() {
    const boost::filesystem::path base_dir(dir_path);
    std::string extension(".jpg");
    std::vector <boost::filesystem::path> filenames;
    try {
        for (boost::filesystem::directory_iterator it(base_dir);
             it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(it->status()) &&
                boost::filesystem::extension(it->path()) == extension) {
                filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());

    for (int i = 0; i < filenames.size(); ++i) {
        string file_name = filenames[i].stem().string();
        if (!isdigit(file_name[file_name.length() - 1])) continue;
        input_image_path = filenames[i].string();

        int dot_index = getIndexBeforeChar(input_image_path, '.');

        output_image_path = input_image_path.substr(0, dot_index) + "_cubemap.jpg";
        std::cout << "input_image_path: " << input_image_path << " output_image_path: " << output_image_path
                  << std::endl;

        imagePanorama.release();
        setImagePanorama(input_image_path);

        const int sqr = imagePanorama.cols / 4.0;
        const int output_width = sqr * 3;
        const int output_height = sqr * 2;

        // Placeholder image for the result
        vector <cv::Mat> destinations;
        for (int i = 0; i < 6; i++) {
            cv::Mat destination(sqr, sqr, CV_8UC3, cv::Scalar(255, 255, 255));
            destinations.push_back(destination);
        }

        auto begin = chrono::high_resolution_clock::now();

#pragma omp parallel for
        for (int j = 0; j < output_width; j++) {
            // #pragma omp parallel for
            for (int i = 0; i < output_height; i++) {
                DestinationType destImageType;
                float tx = 0.0;
                float ty = 0.0;
                float x = 0.0;
                float y = 0.0;
                float z = 0.0;

                if (i < sqr + 1) {   // top half
                    if (j < sqr + 1) { // top left box [Y+]
                        destImageType = DestinationType::Yplus;
                        tx = j;
                        ty = i;
                        x = tx - 0.5 * sqr;
                        y = 0.5 * sqr;
                        z = ty - 0.5 * sqr;
                    } else if (j < 2 * sqr + 1) { // top middle [X+]
                        destImageType = DestinationType::Xplus;
                        tx = j - sqr;
                        ty = i;
                        x = 0.5 * sqr;
                        y = (tx - 0.5 * sqr) * -1;
                        z = ty - 0.5 * sqr;
                    } else { // top right [Y-]
                        destImageType = DestinationType::Yminus;
                        tx = j - sqr * 2;
                        ty = i;
                        x = (tx - 0.5 * sqr) * -1;
                        y = -0.5 * sqr;
                        z = ty - 0.5 * sqr;
                    }
                } else {             // bottom half
                    if (j < sqr + 1) { // bottom left box [X-]
                        destImageType = DestinationType::Xminus;
                        tx = j;
                        ty = i - sqr;
                        x = int(-0.5 * sqr);
                        y = int(tx - 0.5 * sqr);
                        z = int(ty - 0.5 * sqr);
                    } else if (j < 2 * sqr + 1) { // bottom middle [Z-]
                        destImageType = DestinationType::Zminus;
                        tx = j - sqr;
                        ty = i - sqr;
                        x = (ty - 0.5 * sqr) * -1;
                        y = (tx - 0.5 * sqr) * -1;
                        z = 0.5 * sqr; // was -0.5 might be due to phi being reversed
                    } else { // bottom right [Z+]
                        destImageType = DestinationType::Zplus;
                        tx = j - sqr * 2;
                        ty = i - sqr;
                        x = ty - 0.5 * sqr;
                        y = (tx - 0.5 * sqr) * -1;
                        z = -0.5 * sqr; // was +0.5 might be due to phi being reversed
                    }
                }

                // now find out the polar coordinates
                float rho = sqrt(x * x + y * y + z * z);
                float normTheta = getTheta(x, y) / (2 * M_PI); // /(2*M_PI) normalise theta
                float normPhi = (M_PI - acos(z / rho)) / M_PI; // /M_PI normalise phi

                // use this for coordinates
                float iX = normTheta * input_width;
                float iY = normPhi * input_height;

                // catch possible overflows
                if (iX >= input_width) {
                    iX = iX - (input_width);
                }
                if (iY >= input_height) {
                    iY = iY - (input_height);
                }

                destinations[destImageType].at<cv::Vec3b>(ty, tx) = imagePanorama.at<cv::Vec3b>(int(iY), int(iX));
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - begin;

        std::ostringstream oss;
        std::string output_image_subpath1;
        std::string output_image_subpath2;
        dot_index = getIndexBeforeChar(output_image_path, '.');
        output_image_subpath1 = output_image_path.substr(0, dot_index);
        output_image_subpath2 = output_image_path.substr(dot_index, output_image_path.size());

        oss << output_image_subpath1 << "Yplus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Yplus]);
        oss.str("");
        oss << output_image_subpath1 << "Xplus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Xplus]);
        oss.str("");
        oss << output_image_subpath1 << "Yminus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Yminus]);
        oss.str("");
        oss << output_image_subpath1 << "Xminus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Xminus]);
        oss.str("");
        oss << output_image_subpath1 << "Zminus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Zminus]);
        oss.str("");
        oss << output_image_subpath1 << "Zplus" << output_image_subpath2;
        std::cout << oss.str() << std::endl;
        cv::imwrite(oss.str(), destinations[DestinationType::Zplus]);

        if (description_file_option) createDescriptionFiles(output_image_subpath1);

        cout << "Processing time: " << diff.count() << " s" << endl;
    }
}
