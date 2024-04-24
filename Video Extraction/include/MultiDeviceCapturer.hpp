#ifndef MULTIDEVICECAPTURER_HPP
#define MULTIDEVICECAPTURER_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <k4a/k4a.hpp>

// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;

// This is the maximum difference between when we expected an image's timestamp to be and when it actually occurred.
constexpr std::chrono::microseconds MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP(100);

constexpr int64_t WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT = 60000;

class MultiDeviceCapturer
{
public:

    MultiDeviceCapturer(const std::vector<uint32_t>& device_indices, int32_t color_exposure_usec, int32_t powerline_freq);

    void start_devices(const k4a_device_configuration_t& master_config, const k4a_device_configuration_t& sub_config);

    std::vector<k4a::capture> get_synchronized_captures(const k4a_device_configuration_t& sub_config,
        bool compare_sub_depth_instead_of_color = false);

    const k4a::device& get_master_device() const;

    const k4a::device& get_subordinate_device_by_index(size_t i) const;

private:

    // Once the constuctor finishes, devices[0] will always be the master
    k4a::device master_device;
    std::vector<k4a::device> subordinate_devices;
};

k4a_device_configuration_t get_default_config();

k4a_device_configuration_t get_master_config();

k4a_device_configuration_t get_subordinate_config();

void log_lagging_time(const char* lagger, k4a::capture& master, k4a::capture& sub);

void log_synced_image_time(k4a::capture& master, k4a::capture& sub);

#endif MULTIDEVICECAPTURER_HPP