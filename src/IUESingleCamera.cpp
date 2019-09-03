#include "IUESingleCamera.h"
#include <fstream>
#include <cmath>

// Public functions

IUESingleCamera::IUESingleCamera(const std::string& _data_source_path)
    : basePath(_data_source_path)
{	
	if (basePath.back() != '/')
	{
		basePath.append("/");
	}

	metadata = read_metadata(basePath);
    frames_per_second = metadata["fps"];
    field_of_view = metadata["fov"];
    resolution = set_resolution();
    intrinsic_parameter = set_intrinsic_matrix();
}

bool IUESingleCamera::camera_get_calibration(Eigen::Matrix4d& _extrinsic_parameter, Eigen::Matrix3d& _intrinsic_parameter, Eigen::Vector2i& _resolution)
{
    _extrinsic_parameter = extrinsic_parameter;
    _intrinsic_parameter = intrinsic_parameter;
    _resolution = resolution;
    return true;
}

float IUESingleCamera::get_frames_per_second()
{
	return frames_per_second;
}

Eigen::Matrix4d IUESingleCamera::get_extrinsic_matrix()
{
    return extrinsic_parameter;
}

Eigen::Matrix3d IUESingleCamera::get_intrinsic_matrix()
{
    return intrinsic_parameter;
}

Eigen::Vector2i IUESingleCamera::get_resolution()
{
    return resolution;
}

// Protected functions

json IUESingleCamera::read_metadata(const std::string& path)
{
    // read metadata file
    std::ifstream infile(path + "Metadata.json");
    return json::parse(infile);
}

Eigen::Matrix4d IUESingleCamera::set_extrinsic_matrix(std::string camera_identifier)
{
    Eigen::Matrix4d result;
    double yaw = metadata[camera_identifier]["rot_yaw"];
    double yaw_cos = std::cos(PI * yaw / 180.0);
    double yaw_sin = std::sin(PI * yaw / 180.0);

    double pitch = metadata[camera_identifier]["rot_pitch"];
    double pitch_cos = std::cos(PI * pitch / 180.0);
    double pitch_sin = std::sin(PI * pitch / 180.0);

    double roll = metadata[camera_identifier]["rot_roll"];
    double roll_cos = std::cos(PI * roll / 180.0);
    double roll_sin = std::sin(PI * roll / 180.0);

    double x = metadata[camera_identifier]["pos_x"];
    double y = metadata[camera_identifier]["pos_y"];
    double z = metadata[camera_identifier]["pos_z"];

    result(0, 0) = yaw_cos * pitch_cos;
    result(0, 1) = yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos;
    result(0, 2) = yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin;
    result(0, 3) = x;

    result(1, 0) = yaw_sin * pitch_cos;
    result(1, 1) = yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos;
    result(1, 2) = yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin;
    result(1, 3) = y;

    result(2, 0) = -pitch_sin;
    result(2, 1) = pitch_cos * roll_sin;
    result(2, 2) = pitch_cos * roll_cos;
    result(2, 3) = z;
    
    result(3, 0) = 0;
    result(3, 1) = 0;
    result(3, 2) = 0;
    result(3, 3) = 1;

    return result;
}

Eigen::Matrix3d IUESingleCamera::set_intrinsic_matrix()
{
    Eigen::Matrix3d result;
    double focal_length_X = (resolution(0) / 2) / std::tan(PI * (field_of_view / 2.0) / 180.0);
    double focal_length_Y = (resolution(1) / 2) / std::tan(PI * (field_of_view / 2.0) / 180.0);

    result(0, 0) = focal_length_X;
    result(0, 1) = 0.;
    result(0, 2) = 0.;

    result(1, 0) = 0.;
    result(1, 1) = focal_length_Y;
    result(1, 2) = 0.;

    result(2, 0) = 0.;
    result(2, 1) = 0.;
    result(2, 2) = 1.;

    return result;
}

Eigen::Vector2i IUESingleCamera::set_resolution()
{
    std::string sWidth = metadata["width"];
    std::string sHeight = metadata["height"];
    Eigen::Vector2i result;
    result(0) = std::stoi(sWidth);
    result(1) = std::stoi(sHeight);

    return result;
}

std::unique_ptr<char[]> IUESingleCamera::readFile(std::string filename, int32_t& _fileSize)
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