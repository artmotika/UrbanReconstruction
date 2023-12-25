#include "Metrics.h"

#define C1 (double) (0.01 * 255 * 0.01 * 255)
#define C2 (double) (0.03 * 255 * 0.03 * 255)

#define RED_GREY_CONSTANT 0.299
#define GREEN_GREY_CONSTANT 0.587
#define BLUE_GREY_CONSTANT 0.114

#define MAX_I 255

namespace Metrics {

    vector<vector<double>> meanBrightness(cv::Mat img, int partition) {
        int r = 0, g = 0, b = 0;
        vector<vector<double>> mean_brightness(partition, vector<double>(partition, 0.0));
        int width = 0, height = 0, width_prev = 0, height_prev = 0;
        int width_step = img.cols / partition;
        int width_step_remainder = img.cols % partition;
        int height_step = img.rows / partition;
        int height_step_remainder = img.rows % partition;
        for (int i = 0; i < partition; i++) {
            width += width_step;
            if (i == partition - 1) {
                width += width_step_remainder;
            }
            for (int j = 0; j < partition; j++) {
                height += height_step;
                if (j == partition - 1) {
                    height += height_step_remainder;
                }
                for (int x = width_prev; x < width; x++ ) {
                    for (int y = height_prev; y < height; y++ ) {
                        cv::Vec<unsigned char, 3> pixel_color = img.at<cv::Vec3b>(y, x);
                        r = pixel_color[0];
                        g = pixel_color[1];
                        b = pixel_color[2];
                        mean_brightness[i][j] = mean_brightness[i][j] + RED_GREY_CONSTANT * r + GREEN_GREY_CONSTANT * g + BLUE_GREY_CONSTANT * b;
                    }
                }
                height_prev = height;
            }
            height = 0;
            width_prev = width;
        }

        int n = width_step * height_step;
        int end = partition - 1;
        for (int i = 0; i < partition; i++) {
            for (int j = 0; j < partition; j++) {
                if (i != end && j != end) {
                    mean_brightness[i][j] = mean_brightness[i][j] / n;
                } else if (i == end && j != end) {
                    mean_brightness[i][j] = mean_brightness[i][j] /
                                            ((width_step + width_step_remainder) * height_step);
                } else if (i != end && j == end) {
                    mean_brightness[i][j] = mean_brightness[i][j] /
                                            (width_step * (height_step + height_step_remainder));
                } else {
                    mean_brightness[i][j] = mean_brightness[i][j] /
                                            ((width_step + width_step_remainder) * (height_step + height_step_remainder));
                }
            }
        }

        return mean_brightness;
    }

    vector<vector<double>> varianceBrightness(cv::Mat img, vector<vector<double>> mu, int partition) {
        int r = 0, g = 0, b = 0;
        vector<vector<double>> variance_brightness(partition, vector<double>(partition, 0.0));
        double curr_brightness = 0.0;
        int width = 0, height = 0, width_prev = 0, height_prev = 0;
        int width_step = img.cols / partition;
        int width_step_remainder = img.cols % partition;
        int height_step = img.rows / partition;
        int height_step_remainder = img.rows % partition;
        for (int i = 0; i < partition; i++) {
            width += width_step;
            if (i == partition - 1) {
                width += width_step_remainder;
            }
            for (int j = 0; j < partition; j++) {
                height += height_step;
                if (j == partition - 1) {
                    height += height_step_remainder;
                }
                for (int x = width_prev; x < width; x++) {
                    for (int y = height_prev; y < height; y++) {
                        cv::Vec<unsigned char, 3> pixel_color = img.at<cv::Vec3b>(y, x);
                        r = pixel_color[0];
                        g = pixel_color[1];
                        b = pixel_color[2];
                        curr_brightness = RED_GREY_CONSTANT * r + GREEN_GREY_CONSTANT * g + BLUE_GREY_CONSTANT * b;
                        variance_brightness[i][j] = variance_brightness[i][j] + pow(curr_brightness - mu[i][j], 2.0);
                    }
                }
                height_prev = height;
            }
            height = 0;
            width_prev = width;
        }

        int n = width_step * height_step;
        int end = partition - 1;
        for (int i = 0; i < partition; i++) {
            for (int j = 0; j < partition; j++) {
                if (i != end && j != end) {
                    variance_brightness[i][j] = variance_brightness[i][j] / n;
                } else if (i == end && j != end) {
                    variance_brightness[i][j] = variance_brightness[i][j] /
                                            ((width_step + width_step_remainder) * height_step);
                } else if (i != end && j == end) {
                    variance_brightness[i][j] = variance_brightness[i][j] /
                                            (width_step * (height_step + height_step_remainder));
                } else {
                    variance_brightness[i][j] = variance_brightness[i][j] /
                                            ((width_step + width_step_remainder) * (height_step + height_step_remainder));
                }
            }
        }

        return variance_brightness;
    }

    vector<vector<double>> covarianceBrightness(cv::Mat img1, cv::Mat img2, vector<vector<double>> mu1, vector<vector<double>> mu2, int partition) {
        int r1 = 0, g1 = 0, b1 = 0, r2 = 0, g2 = 0, b2 = 0;
        vector<vector<double>> covariance_brightness(partition, vector<double>(partition, 0.0));
        double curr_brightness1 = 0.0;
        double curr_brightness2 = 0.0;
        int width = 0, height = 0, width_prev = 0, height_prev = 0;
        int width_step = img1.cols / partition;
        int width_step_remainder = img1.cols % partition;
        int height_step = img1.rows / partition;
        int height_step_remainder = img1.rows % partition;
        for (int i = 0; i < partition; i++) {
            width += width_step;
            if (i == partition - 1) {
                width += width_step_remainder;
            }
            for (int j = 0; j < partition; j++) {
                height += height_step;
                if (j == partition - 1) {
                    height += height_step_remainder;
                }
                for (int x = width_prev; x < width; x++) {
                    for (int y = height_prev; y < height; y++) {
                        cv::Vec<unsigned char, 3> pixel_color1 = img1.at<cv::Vec3b>(y, x);
                        r1 = pixel_color1[0];
                        g1 = pixel_color1[1];
                        b1 = pixel_color1[2];
                        curr_brightness1 = RED_GREY_CONSTANT * r1 + GREEN_GREY_CONSTANT * g1 + BLUE_GREY_CONSTANT * b1;
                        cv::Vec<unsigned char, 3> pixel_color2 = img2.at<cv::Vec3b>(y, x);
                        r2 = pixel_color2[0];
                        g2 = pixel_color2[1];
                        b2 = pixel_color2[2];
                        curr_brightness2 = RED_GREY_CONSTANT * r2 + GREEN_GREY_CONSTANT * g2 + BLUE_GREY_CONSTANT * b2;
                        covariance_brightness[i][j] = covariance_brightness[i][j] + (curr_brightness1 - mu1[i][j]) * (curr_brightness2 - mu2[i][j]);
                    }
                }
                height_prev = height;
            }
            height = 0;
            width_prev = width;
        }

        int n = width_step * height_step;
        int end = partition - 1;
        for (int i = 0; i < partition; i++) {
            for (int j = 0; j < partition; j++) {
                if (i != end && j != end) {
                    covariance_brightness[i][j] = covariance_brightness[i][j] / n;
                } else if (i == end && j != end) {
                    covariance_brightness[i][j] = covariance_brightness[i][j] /
                                                ((width_step + width_step_remainder) * height_step);
                } else if (i != end && j == end) {
                    covariance_brightness[i][j] = covariance_brightness[i][j] /
                                                (width_step * (height_step + height_step_remainder));
                } else {
                    covariance_brightness[i][j] = covariance_brightness[i][j] /
                                                ((width_step + width_step_remainder) * (height_step + height_step_remainder));
                }
            }
        }

        return covariance_brightness;
    }

    /*
     * Чем ближе значение SSIM к 1, тем больше сходство между изображениями.
     * Эта метрика является полезным инструментом для оценки качества сжатия изображений,
     * а также для оценки эффективности различных алгоритмов обработки изображений.
     */
    double SSIM(cv::Mat image1, cv::Mat image2, int partition) {
        if (partition != 8 && partition != 16 && partition != 1) {
            throw invalid_argument("Partition must be equals 8 or 16 or 1 but it equals " + std::to_string(partition));
        }
        // mean of the brightness of the image1
        vector<vector<double>> mean1 = meanBrightness(image1, partition);
        // mean of the brightness of the image2
        vector<vector<double>> mean2 = meanBrightness(image2, partition);
        // variance of the brightness of the image1
        vector<vector<double>> variance1 = varianceBrightness(image1, mean1, partition);
        // variance of the brightness of the image2
        vector<vector<double>> variance2 = varianceBrightness(image2, mean2, partition);
        // covariance of the brightness between two images
        vector<vector<double>> covariance = covarianceBrightness(image1, image2, mean1, mean2, partition);

        double mean_ssim = 0.0;
        for (int i = 0; i < partition; i++) {
            for (int j = 0; j < partition; j++) {
                mean_ssim += ((2 * mean1[i][j] * mean2[i][j] + C1) * (2 * covariance[i][j] + C2)) /
                             ((pow(mean1[i][j], 2.0) + pow(mean2[i][j], 2.0) + C1) * (variance1[i][j] + variance2[i][j] + C2));
            }
        }

        return mean_ssim / (partition * partition);
    }

    double mseBrightness(cv::Mat img1, cv::Mat img2) {
        int r1 = 0, g1 = 0, b1 = 0, r2 = 0, g2 = 0, b2 = 0;
        double curr_brightness1 = 0.0;
        double curr_brightness2 = 0.0;
        double mse_brightness = 0.0;
        for (int x = 0; x < img1.cols; x++) {
            for (int y = 0; y < img1.rows; y++) {
                cv::Vec<unsigned char, 3> pixel_color1 = img1.at<cv::Vec3b>(y, x);
                r1 = pixel_color1[0];
                g1 = pixel_color1[1];
                b1 = pixel_color1[2];
                curr_brightness1 = RED_GREY_CONSTANT * r1 + GREEN_GREY_CONSTANT * g1 + BLUE_GREY_CONSTANT * b1;
                cv::Vec<unsigned char, 3> pixel_color2 = img2.at<cv::Vec3b>(y, x);
                r2 = pixel_color2[0];
                g2 = pixel_color2[1];
                b2 = pixel_color2[2];
                curr_brightness2 = RED_GREY_CONSTANT * r2 + GREEN_GREY_CONSTANT * g2 + BLUE_GREY_CONSTANT * b2;
                mse_brightness += pow((curr_brightness1 - curr_brightness2), 2.0);
            }
        }
        return mse_brightness / (img1.cols * img1.rows);
    }

    /*
     * Диапазон значений метрики PSNR (Peak Signal-to-Noise Ratio) обычно составляет от 0 до бесконечности.
     * Значение метрики может меняться в зависимости от битности изображения.
     * Метрика, используется для измерения качества восстановленных изображений сравнением с исходным.
     * PSNR измеряет соотношение между максимально возможной мощностью сигнала и мощностью искажения,
     * вызванного ошибками восстановления. PSNR обычно выражается в децибелах (dB).
     */
    double PSNR(cv::Mat image1, cv::Mat image2) {
        return 10 * log10(MAX_I * MAX_I / mseBrightness(image1, image2));
    }
}