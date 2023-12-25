#ifndef URBANRECONSTRUCTION_METRICS_H
#define URBANRECONSTRUCTION_METRICS_H

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/gp3.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace pcl;

namespace Metrics {
    /*
     * Чем ближе значение SSIM к 1, тем больше сходство между изображениями.
     * Эта метрика является полезным инструментом для оценки качества сжатия изображений,
     * а также для оценки эффективности различных алгоритмов обработки изображений.
     */
    double SSIM(cv::Mat image1, cv::Mat image2, int partition);
    /*
     * Диапазон значений метрики PSNR (Peak Signal-to-Noise Ratio) обычно составляет от 0 до бесконечности.
     * Значение метрики может меняться в зависимости от битности изображения.
     * Метрика, используется для измерения качества восстановленных изображений сравнением с исходным.
     * PSNR измеряет соотношение между максимально возможной мощностью сигнала и мощностью искажения,
     * вызванного ошибками восстановления. PSNR обычно выражается в децибелах (dB).
     */
    double PSNR(cv::Mat image1, cv::Mat image2);
    double PhotometricConsistencyRMSEnormalized(PolygonMesh mesh,
                                  string camera_pose_path_set_color,
                                  string camera_pose_path_check_color,
                                  cv::Mat image_real,
                                  cv::Mat image_rendered); // ml course
}


#endif //URBANRECONSTRUCTION_METRICS_H
