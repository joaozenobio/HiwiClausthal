#include "utils.hpp"

void printProgress(double percentage) {
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

void create_xy_table(const k4a::calibration calibration, k4a::image xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)xy_table.get_buffer();

    int width = xy_table.get_width_pixels();

    int height = xy_table.get_height_pixels();

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;
    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            valid = calibration.convert_2d_to_3d(p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray);

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

bool generate_point_cloud(
    const k4a::image depth_image,
    const k4a::image xy_table,
    k4a::image point_cloud,
    int* point_count)
{
    uint32_t width = xy_table.get_width_pixels();
    uint32_t height = xy_table.get_height_pixels();
    uint16_t* depth_data = (uint16_t*)(void*)depth_image.get_buffer();
    k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)xy_table.get_buffer();
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)point_cloud.get_buffer();

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

bool write_point_cloud(const char* file_name, const k4a::image point_cloud, int point_count)
{
    uint32_t image_width = point_cloud.get_width_pixels();
    uint32_t image_height = point_cloud.get_height_pixels();

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)point_cloud.get_buffer();
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
    for (int i = 0; i < image_width * image_height; i++)
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

cv::Mat get_mat(k4a::image image, bool deep_copy)
{
    cv::Mat mat;

    size_t image_size = image.get_size();
    auto image_buffer = image.get_buffer();
    uint32_t image_width = image.get_width_pixels();
    uint32_t image_height = image.get_height_pixels();

    const k4a_image_format_t format = image.get_format();
    switch (format)
    {
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG:
    {
        // NOTE: this is slower than other formats.
        std::vector<uint8_t> buffer(image_buffer, image_buffer + image_size);
        mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
        cv::cvtColor(mat, mat, cv::COLOR_BGR2BGRA);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_NV12:
    {
        cv::Mat nv12 = cv::Mat(image_height + image_height / 2, image_width, CV_8UC1, image_buffer).clone();
        cv::cvtColor(nv12, mat, cv::COLOR_YUV2BGRA_NV12);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_YUY2:
    {
        cv::Mat yuy2 = cv::Mat(image_height, image_width, CV_8UC2, image_buffer).clone();
        cv::cvtColor(yuy2, mat, cv::COLOR_YUV2BGRA_YUY2);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32:
    {
        mat = deep_copy ? cv::Mat(image_height, image_width, CV_8UC4, image_buffer).clone()
            : cv::Mat(image_height, image_width, CV_8UC4, image_buffer);
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16:
    case k4a_image_format_t::K4A_IMAGE_FORMAT_IR16:
    {
        mat = deep_copy ? cv::Mat(image_height, image_width, CV_16UC1, reinterpret_cast<uint16_t*>(image_buffer)).clone()
            : cv::Mat(image_height, image_width, CV_16UC1, reinterpret_cast<uint16_t*>(image_buffer));
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8:
    {
        mat = cv::Mat(image_height, image_width, CV_8UC1, image_buffer).clone();
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM16:
    {
        mat = cv::Mat(image_height, image_width, CV_16UC1, image_buffer).clone();
        break;
    }
    case k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM:
    {
        // NOTE: This is opencv_viz module format (cv::viz::WCloud).
        const int16_t* buffer = reinterpret_cast<int16_t*>(image_buffer);
        mat = cv::Mat(image_height, image_width, CV_32FC3, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
        mat.forEach<cv::Vec3f>(
            [&](cv::Vec3f& point, const int32_t* position) {
                const int32_t index = (position[0] * image_width + position[1]) * 3;
                point = cv::Vec3f(buffer[index + 0], buffer[index + 1], buffer[index + 2]);
            }
        );
        break;
    }
    default:
        std::cerr << "Failed to convert k4a_image_t to cv::Mat" << std::endl;
        break;
    }

    return mat;
}