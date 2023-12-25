#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Metrics/Metrics.h"
#include "Metrics/BlurEstimation.h"

using namespace std;

int main() {
    string camera_image_path = "../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_masked.png";
    string rendered_image_path = "../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_rendered_masked.png";

    cv::Mat camera_image = cv::imread(camera_image_path, cv::IMREAD_COLOR);
    cv::Mat rendered_image = cv::imread(rendered_image_path, cv::IMREAD_COLOR);

    // Уменьшаем разрешение изображений, чтобы минимизировать погрешность позы камеры
    cv::InterpolationFlags interpolation = cv::INTER_AREA;
    cv::Mat camera_image_shrinked_10 = cv::Mat();
    cv::Mat camera_image_shrinked_2 = cv::Mat();
    // Decimate the image by factor of 2 in each direction
    resize(camera_image, camera_image_shrinked_10, cv::Size(), 0.1, 0.1, interpolation);
    resize(camera_image, camera_image_shrinked_2, cv::Size(), 0.5, 0.5, interpolation);

    cv::Mat rendered_image_shrinked_10 = cv::Mat();
    cv::Mat rendered_image_shrinked_2 = cv::Mat();
    // Decimate the image by factor of 2 in each direction
    resize(rendered_image, rendered_image_shrinked_10, cv::Size(), 0.1, 0.1, interpolation);
    resize(rendered_image, rendered_image_shrinked_2, cv::Size(), 0.5, 0.5, interpolation);


    // Считаем ssim метрику для одного и того же изображения
    cout << "ssim метрика для одного и того же изображения: " << Metrics::SSIM(camera_image, camera_image, 8) << endl;
    // Считаем ssim метрику для изображения, полученного с камеры и для срендеренного изображения
    cout << "ssim метрика для изображения, полученного с камеры и для срендеренного изображения: " << Metrics::SSIM(camera_image, rendered_image, 8) << endl;

    // Считаем ssim метрику для уменьшенных изображений, для полученного с камеры и для срендеренного
    cout << "ssim метрика для уменьшенных x10 изображений, для полученного с камеры и для срендеренного: " << Metrics::SSIM(camera_image_shrinked_10, rendered_image_shrinked_10, 8) << endl;
    cout << "ssim метрика для уменьшенных x2 изображений, для полученного с камеры и для срендеренного: " << Metrics::SSIM(camera_image_shrinked_2, rendered_image_shrinked_2, 8) << endl;

    cv::imwrite("../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_shrinked_x10.jpg", camera_image_shrinked_10);
    cv::imwrite("../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_shrinked_x2.jpg", camera_image_shrinked_2);
    cv::imwrite("../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_rendered_shrinked_10.jpg", rendered_image_shrinked_10);
    cv::imwrite("../../data/rendered_images/render_scene_1/pano_000001_000034_cubemapXplus_rendered_shrinked_2.jpg", rendered_image_shrinked_2);

    // Считаем psnr метрику для одного и того же изображения
    cout << "psnr метрика для одного и того же изображения: " << Metrics::PSNR(camera_image, camera_image) << endl;
    // Считаем psnr метрику для изображения, полученного с камеры и для срендеренного изображения
    cout << "psnr метрика для изображения, полученного с камеры и для срендеренного изображения: " << Metrics::PSNR(camera_image, rendered_image) << endl;
    // Считаем psnr метрику для уменьшенных изображений, для полученного с камеры и для срендеренного
    cout << "psnr метрика для уменьшенных изображений x10, для полученного с камеры и для срендеренного: " << Metrics::PSNR(camera_image_shrinked_10, rendered_image_shrinked_10) << endl;
    cout << "psnr метрика для уменьшенных изображений x2, для полученного с камеры и для срендеренного: " << Metrics::PSNR(camera_image_shrinked_2, rendered_image_shrinked_2) << endl;

    // Считаем The Blur Effect метрику для изображения с камеры
    BlurEstimation estimater_camera_image(camera_image);
    cout << "The Blur Effect метрика для изображения с камеры: " << estimater_camera_image.estimate() << endl;
    // Считаем The Blur Effect метрику для срендеренного изображения
    BlurEstimation estimater_rendered_image(rendered_image);
    cout << "The Blur Effect метрика для срендеренного изображения: " << estimater_rendered_image.estimate() << endl;

    return 0;
}