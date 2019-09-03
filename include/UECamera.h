//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_APP_UECAMERA_H
#define BASIC_FACADE_APP_UECAMERA_H

// include Ubitrack BasicFacade
#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>
#include <Eigen/Core>

#include <mutex>
#include <stdint.h>

#include "UESingleCamera.h"
#include "UESingleCameraDepth.h"

// In the UE4 simulation, each virtual camera (Screencapture) consists of 2 cameras with independent poses, but 
// same view axis and rotations. One is color camera, one is depth camera

typedef uint64_t TimestampT;

class UECamera {
public:
    explicit UECamera(const std::string& _components_path);
    virtual ~UECamera() = default;

	bool initialize(const std::string& _utql_filename);
	bool teardown();

    // first camera input
    // naming already reflects future extensions to stereo camera setup
    bool camera_color_get_model(TimestampT ts, double near, double far,
            Eigen::Matrix4d& projection_matrix, Eigen::Matrix3d& intrinsics_matrix, Eigen::Vector2i& resolution);
    bool camera_color_get_pose(TimestampT ts, Eigen::Matrix4d& pose);
    bool camera_color_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >& img);

	bool camera_depth_get_model(TimestampT ts, double near, double far,
		Eigen::Matrix4d& projection_matrix, Eigen::Matrix3d& intrinsics_matrix, Eigen::Vector2i& resolution);
	bool camera_depth_get_pose(TimestampT ts, Eigen::Matrix4d& pose);
	bool camera_depth_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >& img);

    // handlers for push sinks

    void receive_camera_color_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& pose);
	void receive_camera_depth_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& pose);

private:
    std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>            m_pushsink_camera_color_image;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >> m_pullsink_camera_color_model;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>             m_pullsink_camera_color_pose;

    std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>            m_pushsink_camera_depth_image;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >> m_pullsink_camera_depth_model;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>             m_pullsink_camera_depth_pose;

    std::unique_ptr<UESingleCamera> camera_color;
    std::unique_ptr<UESingleCameraDepth> camera_depth;

    std::mutex m_textureAccessMutex_color;
    std::mutex m_textureAccessMutex_depth;
    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> m_current_camera_color_image;
    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> m_current_camera_depth_image;

    TimestampT start_point;
    int32_t currentID_color;
    int32_t currentID_depth;

    std::unique_ptr<Ubitrack::Facade::BasicFacade> m_utFacade;
};





#endif  // BASIC_FACADE_APP_UECAMERA_H
