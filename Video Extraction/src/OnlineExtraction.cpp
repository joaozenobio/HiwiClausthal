#include "../include/OnlineExtraction.hpp"

namespace fs = std::filesystem;

// Extract online data from each camera sensor separately
int onlineExtraction(
    int recording_duration,     // Recording duration in seconds
    std::string base_path,      // Path to save data
    int num_devices) {          // Number of devices connected

    int32_t color_exposure_usec = 8000;  // somewhat reasonable default exposure time
    int32_t powerline_freq = 2;          // default to a 60 Hz powerline
    double calibration_timeout = 60.0; // default to timing out after 60s of trying to get calibrated

    std::string depth_path = "\\depth";
    std::string depth_images_path = depth_path + "\\images";
    std::string depth_raw_matrices_path = depth_path + "\\raw_matrices";
    std::string depth_point_cloud_path = depth_path + "\\point_clouds";
    std::string depth_timestamps_path = depth_path + "\\timestamps.txt";
    std::string color_path = "\\color";
    std::string color_images_path = color_path + "\\images";
    std::string color_timestamps_path = color_path + "\\timestamps.txt";
    std::string ir_path = "\\ir";
    std::string ir_images_path = ir_path + "\\images";
    std::string ir_raw_matrices_path = ir_path + "\\raw_matrices";
    std::string ir_timestamps_path = ir_path + "\\timestamps.txt";
    std::string imu_path = "\\imu.json";

    if (!fs::create_directories(base_path)) {
        std::cerr << "Error creating directory: " << base_path << std::endl;
        return false;
    }

    for (int i = 0; i < num_devices; i++)
    {
        std::string device_path = base_path + "\\" + std::to_string(i);
        if (!fs::create_directories(device_path)) {
            std::cerr << "Error creating directory: " << device_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + depth_path)) {
            std::cerr << "Error creating directory: " << depth_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + depth_images_path)) {
            std::cerr << "Error creating directory: " << depth_images_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + depth_raw_matrices_path)) {
            std::cerr << "Error creating directory: " << depth_raw_matrices_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + depth_point_cloud_path)) {
            std::cerr << "Error creating directory: " << depth_point_cloud_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + color_path)) {
            std::cerr << "Error creating directory: " << color_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + color_images_path)) {
            std::cerr << "Error creating directory: " << color_images_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + ir_path)) {
            std::cerr << "Error creating directory: " << ir_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + ir_images_path)) {
            std::cerr << "Error creating directory: " << ir_images_path << std::endl;
            return false;
        }

        if (!fs::create_directories(device_path + ir_raw_matrices_path)) {
            std::cerr << "Error creating directory: " << ir_raw_matrices_path << std::endl;
            return false;
        }
    }

    // Note that the order of indices in device_indices is not necessarily
    // preserved because MultiDeviceCapturer tries to find the master device based
    // on which one has sync out plugged in. Start with just { 0 }, and add
    // another if needed
    std::vector<uint32_t> device_indices;
    for (uint32_t i = 0; i < num_devices; i++)
    {
        device_indices.push_back(i);
    }

    // Set up a MultiDeviceCapturer to handle getting many synchronous captures
    MultiDeviceCapturer capturer(device_indices, color_exposure_usec, powerline_freq);

    // Create configurations for devices
    k4a_device_configuration_t main_config = get_master_config();
    k4a_device_configuration_t secondary_config = get_subordinate_config();

    // Construct all the things that we'll need whether or not we are running with 1 or more cameras
    k4a::calibration main_calibration = capturer.get_master_device().get_calibration(main_config.depth_mode,
        main_config.color_resolution);

    capturer.start_devices(main_config, secondary_config);

    k4a::transformation transformation(main_calibration);

    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() < recording_duration)
    {
        std::vector<k4a::capture> captures;
        captures = capturer.get_synchronized_captures(secondary_config, true);

        for (int i = 0; i < num_devices; i++) {

            std::string device_path = base_path + "\\" + std::to_string(i);

            std::ofstream depth_timestamps_file(device_path + depth_timestamps_path, std::ios::app);
            if (!depth_timestamps_file.is_open()) {
                std::cerr << "Error opening file: " << device_path + depth_timestamps_path << std::endl;
                return false;
            }

            std::ofstream color_timestamps_file(device_path + color_timestamps_path, std::ios::app);
            if (!color_timestamps_file.is_open()) {
                std::cerr << "Error opening file: " << device_path + color_timestamps_path << std::endl;
                return false;
            }

            std::ofstream ir_timestamps_file(device_path + ir_timestamps_path, std::ios::app);
            if (!ir_timestamps_file.is_open()) {
                std::cerr << "Error opening file: " << device_path + ir_timestamps_path << std::endl;
                return false;
            }

            k4a::image depth_image = captures[i].get_depth_image();
            k4a::image color_image = captures[i].get_color_image();
            k4a::image ir_image = captures[i].get_ir_image();

            if (depth_image.is_valid() && color_image.is_valid() && ir_image.is_valid())
            {
                int32_t color_image_width_pixels = color_image.get_width_pixels();
                int32_t color_image_height_pixels = color_image.get_height_pixels();

                k4a::image transformed_depth_image = k4a::image::create(
                    K4A_IMAGE_FORMAT_DEPTH16,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(uint16_t));
                transformation.depth_image_to_color_camera(depth_image, &transformed_depth_image);

                cv::Mat depth_image_opencv = get_mat(transformed_depth_image);

                uint32_t depth_image_timestamp = depth_image.get_device_timestamp().count();
                std::string depth_image_name = std::format("{:020}", depth_image_timestamp);

                cv::imwrite((device_path + depth_raw_matrices_path + "\\" + depth_image_name + ".jpg").c_str(), depth_image_opencv);

                // 3860mm is the max range of the depth sensor with NFOV_UNBINNED
                depth_image_opencv /= (3860.0 / 255.0);
                cv::imwrite((device_path + depth_images_path + "\\" + depth_image_name + ".jpg").c_str(), depth_image_opencv);
                
                /*
                k4a::image xy_table = k4a::image::create(
                    K4A_IMAGE_FORMAT_CUSTOM,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(k4a_float2_t));
                create_xy_table(main_calibration, xy_table);
                k4a::image point_cloud = k4a::image::create(
                    K4A_IMAGE_FORMAT_CUSTOM,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(k4a_float3_t));
                int point_count = 0;
                generate_point_cloud(transformed_depth_image, xy_table, point_cloud, &point_count);
                write_point_cloud((device_path + depth_point_cloud_path + "\\" + depth_image_name + ".ply").c_str(), point_cloud, point_count);
                */

                depth_timestamps_file << depth_image_timestamp << std::endl;

                transformed_depth_image.reset();
                //xy_table.reset();
                //point_cloud.reset();
                depth_timestamps_file.close();

                uint32_t color_image_timestamp = color_image.get_device_timestamp().count();
                std::string color_image_name = std::format("{:020}", color_image_timestamp);

                cv::Mat color_image_opencv = get_mat(color_image);

                cv::imwrite((device_path + color_images_path + "\\" + color_image_name + ".jpg").c_str(), color_image_opencv);

                color_timestamps_file << color_image_timestamp << std::endl;
                
                color_timestamps_file.close();

                int ir_image_width_pixels = ir_image.get_width_pixels();
                int ir_image_height_pixels = ir_image.get_height_pixels();
                int ir_image_stride_bytes = ir_image.get_stride_bytes();
                uint8_t* ir_image_buffer = ir_image.get_buffer();
                k4a::image custom_ir_image = k4a::image::create_from_buffer(
                    K4A_IMAGE_FORMAT_CUSTOM16,
                    ir_image_width_pixels,
                    ir_image_height_pixels,
                    ir_image_width_pixels * (int)sizeof(uint16_t),
                    ir_image_buffer,
                    ir_image_height_pixels * ir_image_stride_bytes,
                    NULL, // Memory leak?
                    NULL);

                k4a::image transformed_ir_image = k4a::image::create(
                    K4A_IMAGE_FORMAT_CUSTOM16,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(uint16_t));

                k4a::image transformed_depth_image_reference = k4a::image::create(
                    K4A_IMAGE_FORMAT_DEPTH16,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(uint16_t));

                transformation.depth_image_to_color_camera_custom(
                    depth_image,
                    custom_ir_image,
                    &transformed_depth_image_reference,
                    &transformed_ir_image,
                    K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
                    0);

                cv::Mat ir_image_opencv = get_mat(transformed_ir_image);

                uint32_t ir_image_timestamp = ir_image.get_device_timestamp().count();
                std::string ir_image_name = std::format("{:020}", ir_image_timestamp);

                cv::imwrite((device_path + ir_raw_matrices_path + "\\" + ir_image_name + ".jpg").c_str(), ir_image_opencv);

                // 1000 is the max range of the ir sensor
                ir_image_opencv /= (1000.0 / 255.0);
                cv::imwrite((device_path + ir_images_path + "\\" + ir_image_name + ".jpg").c_str(), ir_image_opencv);

                ir_timestamps_file << color_image_timestamp << std::endl;

                transformed_ir_image.reset();
                transformed_depth_image_reference.reset();
                ir_timestamps_file.close();
            }
            depth_image.reset();
            color_image.reset();
            ir_image.reset();
            captures[i].reset();
        }
    }
    transformation.destroy();

    return 0;
}