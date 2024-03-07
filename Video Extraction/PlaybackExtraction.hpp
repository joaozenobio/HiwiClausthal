#ifndef PLAYBACKEXTRACTION_HPP
#define PLAYBACKEXTRACTION_HPP

#include <iostream>
#include <fstream>
#include <filesystem>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <opencv2/highgui.hpp>
#include <nlohmann/json.hpp>

#include "utils.hpp"

int playback_extraction(std::string input_path);

#endif PLAYBACKEXTRACTION_HPP