#include "PlaybackExtraction.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

int playback_extraction(std::string input_path) {

    auto start = std::chrono::high_resolution_clock::now();

    std::string base_path = input_path.substr(0, input_path.find("."));
    std::string depth_path = base_path + "\\depth";
    std::string depth_images_path = depth_path + "\\images";
    std::string depth_raw_matrices_path = depth_path + "\\raw_matrices";
    std::string depth_point_cloud_path = depth_path + "\\point_clouds";
    std::string depth_timestamps_path = depth_path + "\\timestamps.txt";
    std::string color_path = base_path + "\\color";
    std::string color_images_path = color_path + "\\images";
    std::string color_timestamps_path = color_path + "\\timestamps.txt";
    std::string ir_path = base_path + "\\ir";
    std::string imu_path = base_path + "\\imu.json";

    if (!fs::create_directories(base_path)) {
        std::cerr << "Error creating directory: " << base_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(depth_path)) {
        std::cerr << "Error creating directory: " << depth_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(depth_images_path)) {
        std::cerr << "Error creating directory: " << depth_images_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(depth_raw_matrices_path)) {
        std::cerr << "Error creating directory: " << depth_raw_matrices_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(depth_point_cloud_path)) {
        std::cerr << "Error creating directory: " << depth_point_cloud_path << std::endl;
        return 1;
    }

    std::ofstream depth_timestamps_file(depth_timestamps_path, std::ios::app);
    if (!depth_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << depth_timestamps_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(color_path)) {
        std::cerr << "Error creating directory: " << color_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(color_images_path)) {
        std::cerr << "Error creating directory: " << color_images_path << std::endl;
        return 1;
    }

    std::ofstream color_timestamps_file(color_timestamps_path, std::ios::app);
    if (!color_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << color_timestamps_path << std::endl;
        return 1;
    }

    if (!fs::create_directories(ir_path)) {
        std::cerr << "Error creating directory: " << ir_path << std::endl;
        return 1;
    }

    std::string ir_images_path = ir_path + "\\images";
    if (!fs::create_directories(ir_images_path)) {
        std::cerr << "Error creating directory: " << ir_images_path << std::endl;
        return 1;
    }

    std::string ir_raw_matrices_path = ir_path + "\\raw_matrices";
    if (!fs::create_directories(ir_raw_matrices_path)) {
        std::cerr << "Error creating directory: " << ir_raw_matrices_path << std::endl;
        return 1;
    }

    std::string ir_timestamps_path = ir_path + "\\timestamps.txt";
    std::ofstream ir_timestamps_file(ir_timestamps_path, std::ios::app);
    if (!ir_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << ir_timestamps_path << std::endl;
        return 1;
    }

    std::ofstream imu_file(imu_path, std::ios::app);
    if (!imu_file.is_open()) {
        std::cerr << "Error opening file: " << imu_path << std::endl;
        return 1;
    }

    k4a::playback playback = k4a::playback::open(input_path.c_str());

    k4a::calibration calibration = playback.get_calibration();

    k4a::transformation transformation(calibration);

    k4a::capture capture;

    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t ir_image = NULL;
    double recording_length = playback.get_recording_length().count();
    while (playback.get_next_capture(&capture))
    {
        k4a::image depth_image = capture.get_depth_image();
        k4a::image color_image = capture.get_color_image();
        k4a::image ir_image = capture.get_ir_image();

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

            int depth_image_timestamp = depth_image.get_device_timestamp().count();
            std::string depth_image_name = std::format("{:020}", depth_image_timestamp);

            cv::imwrite((depth_raw_matrices_path + "\\" + depth_image_name + ".jpg").c_str(), depth_image_opencv);

            // 3860mm is the max range of the depth sensor with NFOV_UNBINNED
            depth_image_opencv /= (3860.0 / 255.0);
            cv::imwrite((depth_images_path + "\\" + depth_image_name + ".jpg").c_str(), depth_image_opencv);
            /*
            k4a::image xy_table = k4a::image::create(
                K4A_IMAGE_FORMAT_CUSTOM,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(k4a_float2_t));
            create_xy_table(calibration, xy_table);
            k4a::image point_cloud = k4a::image::create(
                K4A_IMAGE_FORMAT_CUSTOM,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(k4a_float3_t));
            int point_count = 0;
            generate_point_cloud(transformed_depth_image, xy_table, point_cloud, &point_count);
            write_point_cloud((depth_point_cloud_path + "\\" + depth_image_name + ".ply").c_str(), point_cloud, point_count);
            */

            depth_timestamps_file << depth_image_timestamp << std::endl;

            transformed_depth_image.reset();
            //xy_table.reset();
            //point_cloud.reset();

            int color_image_timestamp = color_image.get_device_timestamp().count();
            std::string color_image_name = std::format("{:020}", color_image_timestamp);

            cv::Mat color_image_opencv = get_mat(color_image);

            cv::imwrite((color_images_path + "\\" + color_image_name + ".jpg").c_str(), color_image_opencv);

            color_timestamps_file << color_image_timestamp << std::endl;

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
                NULL, // memory leak?
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

            int ir_image_timestamp = ir_image.get_device_timestamp().count();
            std::string ir_image_name = std::format("{:020}", ir_image_timestamp);

            cv::imwrite((ir_raw_matrices_path + "\\" + ir_image_name + ".jpg").c_str(), ir_image_opencv);

            // 1000 is the max range of the ir sensor
            ir_image_opencv /= (1000.0 / 255.0);
            cv::imwrite((ir_images_path + "\\" + ir_image_name + ".jpg").c_str(), ir_image_opencv);

            ir_timestamps_file << color_image_timestamp << std::endl;

            custom_ir_image.reset();
            transformed_ir_image.reset();
            transformed_depth_image_reference.reset();

            printProgress(depth_image_timestamp / recording_length);
        }
        depth_image.reset();
        color_image.reset();
        ir_image.reset();
        capture.reset();
    }
    transformation.destroy();

    depth_timestamps_file.close();
    color_timestamps_file.close();
    ir_timestamps_file.close();

    k4a_imu_sample_t imu_sample;

    auto imu_data_array = json::array();

    while (playback.get_next_imu_sample(&imu_sample))
    {
        json imu_json = json::object();
        imu_json["data"] = json::object();
        imu_json["data"]["acc_sample"] = json::object();
        imu_json["data"]["acc_sample"]["x"] = imu_sample.acc_sample.xyz.x;
        imu_json["data"]["acc_sample"]["y"] = imu_sample.acc_sample.xyz.y;
        imu_json["data"]["acc_sample"]["z"] = imu_sample.acc_sample.xyz.z;
        imu_json["data"]["acc_timestamp_usec"] = imu_sample.acc_timestamp_usec;
        imu_json["data"]["gyro_sample"] = json::object();
        imu_json["data"]["gyro_sample"]["x"] = imu_sample.gyro_sample.xyz.x;
        imu_json["data"]["gyro_sample"]["y"] = imu_sample.gyro_sample.xyz.y;
        imu_json["data"]["gyro_sample"]["z"] = imu_sample.gyro_sample.xyz.z;
        imu_json["data"]["gyro_timestamp_usec"] = imu_sample.gyro_timestamp_usec;

        imu_data_array.push_back(imu_json);
    }

    json imu_data = json::object();
    imu_data["data"] = imu_data_array;

    imu_file << imu_data.dump(4) << std::endl;

    playback.close();
    imu_file.close();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << std::endl << input_path + " concluded in " << duration.count() << " seconds." << std::endl;

    return 0;
}