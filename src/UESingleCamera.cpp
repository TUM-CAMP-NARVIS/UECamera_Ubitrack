#include "UESingleCamera.h"

#include <fstream>
#include <filesystem>
#include <regex>

#if __cplusplus < 201703L // If the version of C++ is less than 17
// It was still in the experimental:: namespace
namespace fs = std::experimental::filesystem;
#else
namespace fs = std::filesystem;
#endif

using json = nlohmann::json;

UESingleCamera::UESingleCamera(const std::string& _data_source_path)
    : IUESingleCamera(_data_source_path)
    , basePath(_data_source_path)
{
    std::vector<std::string> allFiles;
    for (const auto & entry : fs::directory_iterator(basePath))
    {
        allFiles.push_back(entry.path().u8string());
    }

    std::regex rex;
    std::smatch matches;
    rex = R"###(image_number_([0-9]*)\.raw8)###";

    for (auto file : allFiles)
    {
        if (std::regex_search(file, matches, rex))
        {
            std::string numberPath = matches[0];
            int id = std::stoi(numberPath);
            entryList[id] = file;
        }
    }

    extrinsic_parameter = set_extrinsic_matrix("color_image");
}

std::unique_ptr<cv::Mat> UESingleCamera::getImage(int32_t id)
{
    std::string filePath = entryList[id];
    int32_t fileSize;
    std::unique_ptr<char[]> buffer = readFile(filePath, fileSize);

    std::unique_ptr<cv::Mat> img = std::make_unique<cv::Mat>(metadata["color_image"]["height"], metadata["color_image"]["width"], CV_8UC4);
    std::memcpy(img->data, buffer.get(), fileSize);

    return img;
}

