#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "util.h"
#include "json.h"

int RLEDecompress(const uint8_t *input_data, int size, uint8_t *output_data, int &buf_len, uint16_t align)
{
    if (!input_data || size == 0)
    {
        return 0;
    }

    const uint8_t *ptr = input_data;
    ptr += 4;

    auto height =
        static_cast<uint16_t>(static_cast<uint16_t>(*ptr++ << 8) & 0XFF00);
    height |= static_cast<uint16_t>(*ptr++);
    auto width =
        static_cast<uint16_t>(static_cast<uint16_t>(*ptr++ << 8) & 0xFF00);
    width |= static_cast<uint16_t>(*ptr++);

    if (height <= 0 || width <= 0)
    {
        return 1;
    }

    uint16_t step = width;
    uint16_t padding_value = 0;
    if (align > 0)
    {
        step = (width + align - 1) / align * align;
        padding_value = step - width;
    }

    if (buf_len < height * step)
    {
        return 2;
    }

    buf_len = height * step;
    int x = 0;
    int y = 0;
    int rle_cnt = (size - 8) / 3;
    uint8_t *p_im_data = output_data;
    int offset = 0;
    for (int i = 0; i < rle_cnt; ++i)
    {
        uint8_t label = *ptr++;
        uint16_t cnt = *reinterpret_cast<const uint16_t *>(ptr);
        ptr += 2;

        x += cnt;
        if (x > width)
        {
            return 1;
        }

        if (y > height)
        {
            return 1;
        }

        offset += static_cast<int>(cnt);
        if (offset > buf_len)
        {
            return 1;
        }

        for (int j = 0; j < cnt; ++j)
        {
            *p_im_data++ = label;
        }

        if (x == width)
        {
            y++;
            x = 0;
            p_im_data += padding_value;
            offset += padding_value;
            if (offset > buf_len)
            {
                return 1;
            }
        }
    }

    return 0;
}

bool DecodeParsing(const uint8_t *input_data, int input_size, int img_height, int img_width, cv::Mat &img)
{

    int img_length = img_width * img_height;
    auto *decode_data = new unsigned char[img_length];
    memset(decode_data, 255, img_length);

    int ret = RLEDecompress(input_data, input_size, decode_data, img_length, 1);
    img = cv::Mat(img_height, img_width, CV_8UC1, decode_data).clone();

    delete[] decode_data;
    decode_data = nullptr;

    if (ret != 0)
    {
        return false;
    }

    return true;
}

std::string join_filepath(const std::string &dir_, const std::string &filename)
{
    std::filesystem::path dir(dir_);
    std::filesystem::path file(filename);
    std::filesystem::path full_path = dir / file;
    return full_path.c_str();
}

// 数据压缩参数，0为不压缩，如果9，压缩时间将较长，1的性价比较高
std::vector<int> _compression_params = {cv::IMWRITE_PNG_COMPRESSION, 1};

// 拉伸图像，并对数据范围进行裁切
void inv_crop(cv::Mat &seg, int rows = 2160, int cols = 3840, uchar border_val = 255)
{
    float s = cols * 1.f / seg.cols;
    cv::Mat tmp(rows, cols, seg.type());
    tmp.setTo(border_val);
    cv::resize(seg, seg, cv::Size(), s, s, cv::INTER_NEAREST);
    seg.copyTo(tmp.rowRange(0, seg.rows));
    seg = tmp * 10;
}

void PerceptionRawMsgLaneParsing2Image(std::vector<uint8_t> seg_data, int input_size, int img_height,
                                       int img_width, int64_t frame_ts, cv::Mat &seg_image)
{
    uint8_t *input_data = new uint8_t[seg_data.size()];
    if (!seg_data.empty())
    {
        memcpy(input_data, &seg_data[0], seg_data.size() * sizeof(uint8_t));
    }
    bool decode = DecodeParsing(input_data, input_size, img_height, img_width, seg_image);
    if (decode)
    {
        inv_crop(seg_image);
    }
}

void perception_data_to_png(std::string &perception_data_path, cv::Mat &seg_image)
{
    std::ifstream json_file(perception_data_path);
    nlohmann::json json;
    json_file >> json;
    std::vector<uint8_t> data;
    for (auto one : json["parsing_raw"][0]["data"])
    {
        data.push_back(one);
    }
    int w = json["parsing_raw"][0]["img_desc"]["width"];
    int h = json["parsing_raw"][0]["img_desc"]["height"];
    uint64_t stamp_ms = json["img_desc"][0]["time_stamp"];
    float left = json["parsing_raw"][0]["img_desc"]["roi_map"][0]["model_roi"]["left"];
    float top = json["parsing_raw"][0]["img_desc"]["roi_map"][0]["model_roi"]["top"];
    PerceptionRawMsgLaneParsing2Image(data, data.size(), h, w, stamp_ms, seg_image);
    cv::Mat t_mat =cv::Mat::zeros(2, 3, CV_32FC1);
    t_mat.at<float>(0, 0) = 1;
    t_mat.at<float>(0, 2) = left; //水平平移量
    t_mat.at<float>(1, 1) = 1;
    t_mat.at<float>(1, 2) = top; //竖直平移量
    cv::warpAffine(seg_image, seg_image, t_mat, seg_image.size());      //对齐png和seg
}
