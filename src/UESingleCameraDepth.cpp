#include "UESingleCameraDepth.h"
#include "nlohmann/json.hpp"
#include <thread>
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

UESingleCameraDepth::UESingleCameraDepth(const std::string& _data_source_path)
	: IUESingleCamera(_data_source_path)
{
	std::vector<std::string> allFiles;
	for (const auto & entry : fs::directory_iterator(basePath))
	{
		allFiles.push_back(entry.path().u8string());
	}

	std::regex rex;
	std::smatch matches;
	rex = R"###(image_number_([0-9]*)\.raw32f)###";
	
	for (auto file : allFiles)
	{
		if (std::regex_search(file, matches, rex))
		{
			std::string numberPath = matches[0];
			int id = std::stoi(numberPath);
			entryList[id] = file;
		}
	}

    extrinsic_parameter = set_extrinsic_matrix("depth_image");
}

std::unique_ptr<char[]> readFile(std::string filename, int32_t& _fileSize)
{
	// open the file:
	std::ifstream file(filename, std::ios::binary);
	file.unsetf(std::ios::skipws);

	// get its size:
	int32_t fileSize;

	file.seekg(0, std::ios::end);
	fileSize = static_cast<int32_t>(file.tellg());
	file.seekg(0, std::ios::beg);

	std::vector<float> test;
	test.resize(fileSize / sizeof(float));

	std::unique_ptr<char[]> buffer(new char[fileSize]);

	// read the data:
	file.read(buffer.get(), fileSize);

	file.close();
	_fileSize = fileSize;

	return buffer;
}


std::unique_ptr<cv::Mat> UESingleCameraDepth::getImage(int32_t id)
{
	std::string filePath = entryList[id];
    int32_t fileSize;
    std::unique_ptr<char[]> buffer = readFile(filePath, fileSize);

    std::vector<float> fImageValues;
    fImageValues.resize(fileSize / 4);
    for (uint64_t index = 0; index < fImageValues.size(); ++index)
    {
        fImageValues[index] = *reinterpret_cast<float*>(&buffer[index * 4]);
    }

    std::unique_ptr<cv::Mat> img = std::make_unique<cv::Mat>(metadata["depth_image"]["height"], metadata["depth_image"]["width"], CV_32FC1);
    std::memcpy(img->data, fImageValues.data(), fImageValues.size() * sizeof(fImageValues[0]));

	return img;
}

