#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <fstream>
#include <k4a/k4a.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

constexpr auto PBSTR = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
constexpr auto PBWIDTH = 60;

void printProgress(double percentage);

void create_xy_table(const k4a::calibration calibration, k4a::image xy_table);

bool generate_point_cloud(const k4a::image depth_image, const k4a::image xy_table, k4a::image point_cloud, int* point_count);

bool write_point_cloud(const char* file_name, const k4a::image point_cloud, int point_count);

cv::Mat get_mat(k4a::image image, bool deep_copy = true);

#endif UTILS_HPP