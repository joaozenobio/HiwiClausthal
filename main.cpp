// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/transformation
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/fastpointcloud
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen
// https://gist.github.com/UnaNancyOwen/9f16ce7ea4c2673fe08b4ce4804fc209

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <format>
#include <cstdlib>
#include <filesystem>
namespace fs = std::filesystem;

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

static bool create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    if (table_data == NULL)
    {
        std::cerr << "Failed to get xy_table image buffer" << std::endl;
        return false;
    }

    int width = k4a_image_get_width_pixels(xy_table);
    if (width == 0)
    {
        std::cerr << "Failed to get xy_table image width" << std::endl;
        return false;
    }
    int height = k4a_image_get_height_pixels(xy_table);
    if (height == 0)
    {
        std::cerr << "Failed to get xy_table image height" << std::endl;
        return false;
    }

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;
    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            if (K4A_RESULT_SUCCEEDED != k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid))
            {
                std::cerr << "Failed to transform 2D point to 3D" << std::endl;
                return false;
            }

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }

    return true;
}

static bool generate_point_cloud(const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int* point_count)
{
    int width = k4a_image_get_width_pixels(xy_table);
    if (width == 0)
    {
        std::cerr << "Failed to get xy_table image width" << std::endl;
        return false;
    }
    int height = k4a_image_get_height_pixels(xy_table);
    if (height == 0)
    {
        std::cerr << "Failed to get xy_table image height" << std::endl;
        return false;
    }

    uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    if (depth_data == NULL)
    {
        std::cerr << "Failed to get depth image buffer" << std::endl;
        return false;
    }

    k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    if (xy_table_data == NULL)
    {
        std::cerr << "Failed to get xy_table image buffer" << std::endl;
        return false;
    }

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);
    if (point_cloud_data == NULL)
    {
        std::cerr << "Failed to get point_cloud image buffer" << std::endl;
        return false;
    }

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }

    return true;
}

static bool write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
    int width = k4a_image_get_width_pixels(point_cloud);
    if (width == 0)
    {
        std::cerr << "Failed to get point_cloud image width" << std::endl;
        return false;
    }
    int height = k4a_image_get_height_pixels(point_cloud);
    if (height == 0)
    {
        std::cerr << "Failed to get point_cloud image height" << std::endl;
        return false;
    }

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);
    if (point_cloud_data == NULL)
    {
        std::cerr << "Failed to get point_cloud image buffer" << std::endl;
        return false;
    }

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
            << (float)point_cloud_data[i].xyz.z << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

cv::Mat get_mat(k4a_image_t src, bool deep_copy = true)
{
    assert(k4a_image_get_size(src) != 0);

    cv::Mat mat;
    const int32_t width = k4a_image_get_width_pixels(src);
    const int32_t height = k4a_image_get_height_pixels(src);

    const k4a_image_format_t format = k4a_image_get_format(src);
    switch (format)
    {
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG:
    {
        // NOTE: this is slower than other formats.
        std::vector<uint8_t> buffer(k4a_image_get_buffer(src), k4a_image_get_buffer(src) + k4a_image_get_size(src));
        mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
        cv::cvtColor(mat, mat, cv::COLOR_BGR2BGRA);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12:
    {
        cv::Mat nv12 = cv::Mat(height + height / 2, width, CV_8UC1, k4a_image_get_buffer(src)).clone();
        cv::cvtColor(nv12, mat, cv::COLOR_YUV2BGRA_NV12);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2:
    {
        cv::Mat yuy2 = cv::Mat(height, width, CV_8UC2, k4a_image_get_buffer(src)).clone();
        cv::cvtColor(yuy2, mat, cv::COLOR_YUV2BGRA_YUY2);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32:
    {
        mat = deep_copy ? cv::Mat(height, width, CV_8UC4, k4a_image_get_buffer(src)).clone()
            : cv::Mat(height, width, CV_8UC4, k4a_image_get_buffer(src));
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
    case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16:
    {
        mat = deep_copy ? cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(k4a_image_get_buffer(src))).clone()
            : cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(k4a_image_get_buffer(src)));
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8:
    {
        mat = cv::Mat(height, width, CV_8UC1, k4a_image_get_buffer(src)).clone();
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM:
    {
        // NOTE: This is opencv_viz module format (cv::viz::WCloud).
        const int16_t* buffer = reinterpret_cast<int16_t*>(k4a_image_get_buffer(src));
        mat = cv::Mat(height, width, CV_32FC3, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
        mat.forEach<cv::Vec3f>(
            [&](cv::Vec3f& point, const int32_t* position) {
                const int32_t index = (position[0] * width + position[1]) * 3;
                point = cv::Vec3f(buffer[index + 0], buffer[index + 1], buffer[index + 2]);
            }
        );
        break;
    }
    default:
        printf("Failed to convert this format\n");
        break;
    }

    return mat;
}

cv::Mat k4a_get_mat(k4a_image_t& src, bool deep_copy = true)
{
    k4a_image_reference(src);
    return get_mat(k4a_image_t(src), deep_copy);
}

// Check if the file was opened successfully
if (!outfile.is_open()) {
    cerr << "Error opening file: " << filename << endl;
    return 1;
}

// Get numbers from the user (or any other source)
int num;
cout << "Enter numbers to save (enter -1 to quit):" << endl;

while (cin >> num && num != -1) {
    // Write the number to the file
    outfile << num << endl;
}

// Close the file
outfile.close();

cout << "Numbers saved successfully to " << filename << endl;

int main(int argc, char** argv) {

    std::string input_path = "C:\\Users\\zenob\\Desktop\\dataset\\task1\\person1\\front.mkv";
    
    std::string base_path = input_path.substr(0, input_path.find("."));
    if (!fs::create_directories(base_path)) {
        std::cerr << "Error creating directory: " << base_path << std::endl;
        return 1;
    }
    std::string depth_path = base_path + "\\depth";
    if (!fs::create_directories(depth_path)) {
        std::cerr << "Error creating directory: " << depth_path << std::endl;
        return 1;
    }
    std::string color_path = base_path + "\\color";
    if (!fs::create_directories(color_path)) {
        std::cerr << "Error creating directory: " << color_path << std::endl;
        return 1;
    }
    std::string ir_path = base_path + "\\ir";
    if (!fs::create_directories(ir_path)) {
        std::cerr << "Error creating directory: " << ir_path << std::endl;
        return 1;
    }

    std::string depth_timestamps_path = depth_path + "\\timestamps.txt";
    std::ofstream depth_timestamps_file(depth_timestamps_path, std::ios::app);
    if (!depth_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << depth_timestamps_path << std::endl;
        return 1;
    }
    std::string color_timestamps_path = color_path + "\\timestamps.txt";
    std::ofstream color_timestamps_file(color_timestamps_path, std::ios::app);
    if (!color_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << depth_timestamps_path << std::endl;
        return 1;
    }
    std::string ir_timestamps_path = ir_path + "\\timestamps.txt";
    std::ofstream ir_timestamps_file(ir_timestamps_path, std::ios::app);
    if (!ir_timestamps_file.is_open()) {
        std::cerr << "Error opening file: " << depth_timestamps_path << std::endl;
        return 1;
    }
    std::string imu_path = base_path + "\\imu.json";
    std::ofstream imu_file(imu_path, std::ios::app);
    if (!imu_file.is_open()) {
        std::cerr << "Error opening file: " << depth_timestamps_path << std::endl;
        return 1;
    }

    k4a_playback_t playback = NULL;
    k4a_result_t result;
    result = k4a_playback_open(input_path.c_str(), &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        std::cerr << "Failed to open recording: " << input_path << std::endl;
        return 1;
    }

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        std::cerr << "Failed to get calibration" << std::endl;
        return 1;
    }

    k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&calibration);
    if (transformation == NULL)
    {
        std::cerr << "Failed to create transformation handle" << std::endl;
        return 1;
    }

    k4a_capture_t capture = NULL;
    k4a_stream_result_t stream_result;
    stream_result = k4a_playback_get_next_capture(playback, &capture);
    if (stream_result == K4A_STREAM_RESULT_FAILED || capture == NULL)
    {
        std::cerr << "Failed to fetch frame" << std::endl;
        return 1;
    }
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t ir_image = NULL;
    while (stream_result != K4A_STREAM_RESULT_EOF)
    {   
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0)
        {
            std::cout << "Failed to get depth image from capture" << std::endl;
        }

        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0)
        {
            std::cout << "Failed to get color image from capture" << std::endl;
        }

        ir_image = k4a_capture_get_ir_image(capture);
        if (ir_image == 0)
        {
            std::cout << "Failed to get ir image from capture" << std::endl;
        }

        if (depth_image && color_image && ir_image)
        {
            int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
            if (color_image_width_pixels == 0)
            {
                std::cerr << "Failed to get image width" << std::endl;
                return 1;
            }            
            int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
            if (color_image_height_pixels == 0)
            {
                std::cerr << "Failed to get image width" << std::endl;
                return 1;
            }

            k4a_image_t transformed_depth_image = NULL;
            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(uint16_t),
                &transformed_depth_image))
            {
                std::cerr << "Failed to create transformed depth image" << std::endl;
                return 1;
            }
            if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
            {
                std::cerr << "Failed to compute transformed depth image" << std::endl;
                return 1;
            }

            k4a_image_t xy_table = NULL;
            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(k4a_float2_t),
                &xy_table))
            {
                std::cerr << "Failed to create xy_table image" << std::endl;
                return 1;
            }
            if (!create_xy_table(&calibration, xy_table))
            {
                std::cerr << "Failed to create xy_table" << std::endl;
                return 1;
            }

            k4a_image_t point_cloud = NULL;
            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(k4a_float3_t),
                &point_cloud))
            {
                std::cerr << "Failed to create point_cloud image" << std::endl;
                return 1;
            }
            int point_count = 0;
            if (!generate_point_cloud(transformed_depth_image, xy_table, point_cloud, &point_count))
            {
                std::cerr << "Failed to generate point cloud" << std::endl;
                return 1;
            }

            int depth_image_timestamp = (int)k4a_image_get_device_timestamp_usec(depth_image);
            write_point_cloud((depth_directory_path + std::format("{:020}", depth_image_timestamp) + ".ply").c_str(), point_cloud, point_count);
            if (depth_image != NULL)
            {
                k4a_image_release(depth_image);
                depth_image = NULL;
            }
            if (transformed_depth_image != NULL)
            {
                k4a_image_release(transformed_depth_image);
                transformed_depth_image = NULL;
            }
            if (xy_table != NULL)
            {
                k4a_image_release(xy_table);
                xy_table = NULL;
            }
            if (point_cloud != NULL)
            {
                k4a_image_release(point_cloud);
                point_cloud = NULL;
            }

            int color_image_timestamp = (int)k4a_image_get_device_timestamp_usec(color_image);
            cv::Mat color_image_opencv = k4a_get_mat(color_image);
            cv::imwrite((color_images_path + std::to_string(color_image_timestamp) + ".png").c_str(), color_image_opencv);
            if (color_image != NULL)
            {
                k4a_image_release(color_image);
                color_image = NULL;
            }

            int ir_image_timestamp = (int)k4a_image_get_device_timestamp_usec(ir_image);
            cv::Mat ir_image_opencv = k4a_get_mat(ir_image);
            cv::imwrite((ir_images_path + std::format("{:020}", ir_image_timestamp) + ".png").c_str(), ir_image_opencv);
            if (ir_image != NULL)
            {
                k4a_image_release(ir_image);
                ir_image = NULL;
            }
        }
        else{
            if (depth_image != NULL)
            {
                k4a_image_release(depth_image);
                depth_image = NULL;
            }
            if (color_image != NULL)
            {
                k4a_image_release(color_image);
                color_image = NULL;
            }
            if (ir_image != NULL)
            {
                k4a_image_release(ir_image);
                ir_image = NULL;
            }
        }
        if (capture != NULL)
        {
            k4a_capture_release(capture);
            capture = NULL;
        }
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        if (stream_result == K4A_STREAM_RESULT_FAILED)
        {
            printf("Failed to fetch frame\n");
            return -1;
        }
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }

    std::cout << input_file + " concluded.";

    return 0;
}