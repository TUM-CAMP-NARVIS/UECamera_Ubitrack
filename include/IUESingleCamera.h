//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_APP_IUESINGLECAMERA_H
#define BASIC_FACADE_APP_IUESINGLECAMERA_H

// include Ubitrack BasicFacade
#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include "nlohmann/json.hpp"

using json = nlohmann::json;
class IUESingleCamera {
public:
	explicit IUESingleCamera(const std::string& _components_path);
    virtual ~IUESingleCamera() = default;

	// Setup functions
    virtual bool camera_get_calibration(Eigen::Matrix4d& _extrinsic_parameter, Eigen::Matrix3d& _intrinsic_parameter, Eigen::Vector2i& _resolution);
	float get_frames_per_second();

	// Continuous function, called to retrieve an OpenCV image containing the image data (8bit RGB for color, or 32bit float for depth)
	virtual std::unique_ptr<cv::Mat> getImage(int32_t id) = 0;
    virtual Eigen::Matrix4d get_extrinsic_matrix();
    virtual Eigen::Matrix3d get_intrinsic_matrix();
    virtual Eigen::Vector2i get_resolution();

protected:
    Eigen::Vector2i set_resolution();
	Eigen::Matrix4d set_extrinsic_matrix(std::string camera_identifier);
    Eigen::Matrix3d set_intrinsic_matrix();
    
    std::unique_ptr<char[]> readFile(std::string filename, int32_t& _fileSize);

	json read_metadata(const std::string& path);
	json metadata;

    std::string basePath;

    Eigen::Vector2i resolution;
	Eigen::Matrix4d extrinsic_parameter;
    Eigen::Matrix3d intrinsic_parameter;

    float field_of_view;
    float frames_per_second;

    const double PI = 3.14159265358979323846;
};

#endif  // BASIC_FACADE_APP_IUESINGLECAMERA_H
