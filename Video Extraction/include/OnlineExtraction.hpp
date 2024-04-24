#ifndef ONLINEEXTRACTION_HPP
#define ONLINEEXTRACTION_HPP

#include <iostream>
#include <fstream>
#include <filesystem>
#include <k4a/k4a.hpp>
#include <opencv2/highgui.hpp>

#include "utils.hpp"
#include "MultiDeviceCapturer.hpp"

int onlineExtraction(int recording_duration, std::string base_path, int num_devices);

#endif ONLINEEXTRACTION_HPP