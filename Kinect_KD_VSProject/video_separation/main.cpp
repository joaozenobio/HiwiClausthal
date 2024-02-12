// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/transformation
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/fastpointcloud
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen
// https://gist.github.com/UnaNancyOwen/9f16ce7ea4c2673fe08b4ce4804fc209

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

    int width = k4a_image_get_width_pixels(xy_table);
    int height = k4a_image_get_height_pixels(xy_table);

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);

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
}

static void generate_point_cloud(const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int* point_count)
{
    int width = k4a_image_get_width_pixels(xy_table);
    int height = k4a_image_get_height_pixels(xy_table);

    uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

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
}

static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
    int width = k4a_image_get_width_pixels(point_cloud);
    int height = k4a_image_get_height_pixels(point_cloud);

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

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
        printf("Failed to convert this format.");
        break;
    }

    return mat;
}

cv::Mat k4a_get_mat(k4a_image_t& src, bool deep_copy = true)
{
    k4a_image_reference(src);
    return get_mat(k4a_image_t(src), deep_copy);
}

int main(int argc, char** argv) {

    std::string input_file_name = "C:\\Users\\zenob\\Desktop\\front.mkv";
    std::string depth_image_output_directory = "C:\\Users\\zenob\\Desktop\\depth\\";
    std::string color_image_output_directory = "C:\\Users\\zenob\\Desktop\\color\\";
    std::string ir_image_output_directory = "C:\\Users\\zenob\\Desktop\\ir\\";
    k4a_playback_t playback = NULL;
    k4a_result_t result;
    result = k4a_playback_open(input_file_name.c_str(), &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_file_name.c_str());
        return -1;
    }

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        return -1;
    }

    k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&calibration);

    k4a_capture_t capture = NULL;
    k4a_stream_result_t stream_result;
    stream_result = k4a_playback_get_next_capture(playback, &capture);
    if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
    {
        printf("Failed to fetch frame\n");
        return -1;
    }

    k4a_image_t depth_image = NULL;
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        return -1;
    }

    k4a_image_t color_image = NULL;
    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        return -1;
    }

    k4a_image_t ir_image = NULL;
    ir_image = k4a_capture_get_ir_image(capture);
    if (ir_image == 0)
    {
        printf("Failed to get ir image from capture\n");
        return -1;
    }

    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return -1;
    }
    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return -1;
    }

    k4a_image_t xy_table = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(k4a_float2_t),
        &xy_table);
    create_xy_table(&calibration, xy_table);

    k4a_image_t point_cloud = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(k4a_float3_t),
        &point_cloud);
    int point_count = 0;
    generate_point_cloud(transformed_depth_image, xy_table, point_cloud, &point_count);
    int depth_image_timestamp = (int)k4a_image_get_device_timestamp_usec(depth_image);
    write_point_cloud((depth_image_output_directory + std::to_string(depth_image_timestamp) + ".ply").c_str(), point_cloud, point_count);

    int color_image_timestamp = (int)k4a_image_get_device_timestamp_usec(color_image);
    cv::Mat color_image_opencv = k4a_get_mat(color_image);
    cv::imwrite((color_image_output_directory + std::to_string(color_image_timestamp) + ".png").c_str(), color_image_opencv);

    int ir_image_timestamp = (int)k4a_image_get_device_timestamp_usec(ir_image);
    cv::Mat ir_image_opencv = k4a_get_mat(ir_image);
    cv::imwrite((ir_image_output_directory + std::to_string(ir_image_timestamp) + ".png").c_str(), ir_image_opencv);

    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }

    return 0;
}