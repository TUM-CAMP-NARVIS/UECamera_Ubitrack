#include "../include/UECamera.h"

#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>

#include <chrono>

UECamera::UECamera(const std::string & _components_path)
    : currentID_color(0)
    , currentID_depth(0)
    , m_utFacade(new Ubitrack::Facade::BasicFacade(_components_path.c_str()))
{
    camera_color = std::make_unique<UESingleCamera>(_components_path);
    camera_depth = std::make_unique<UESingleCameraDepth>(_components_path);
    start_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}

bool UECamera::initialize(const std::string & _utql_filename)
{
    // create sinks/sources

    // RGB Camera
    m_pushsink_camera_color_image = std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>(
        m_utFacade->getPushSink<Ubitrack::Facade::BasicImageMeasurement>("camera_color_image")
        );

    m_pullsink_camera_color_model= std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >>(
        m_utFacade->getPullSink<Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >("camera_color_intrinsics")
        );

    m_pullsink_camera_color_pose = std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>(
        m_utFacade->getPullSink<Ubitrack::Facade::BasicPoseMeasurement>("camera_color_pose")
        );

    if (m_pushsink_camera_color_image) {
        m_pushsink_camera_color_image->registerCallback(std::bind(&UECamera::receive_camera_color_image, this, std::placeholders::_1));
    }

    // Depth camera
    m_pushsink_camera_depth_image = std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>(
        m_utFacade->getPushSink<Ubitrack::Facade::BasicImageMeasurement>("camera_depth_image")
        );

    m_pullsink_camera_depth_model = std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >>(
        m_utFacade->getPullSink<Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >("camera_depth_intrinsics")
        );

    m_pullsink_camera_depth_pose = std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>(
        m_utFacade->getPullSink<Ubitrack::Facade::BasicPoseMeasurement>("camera_depth_pose")
        );

    if (m_pushsink_camera_depth_image) {
        m_pushsink_camera_depth_image->registerCallback(std::bind(&UECamera::receive_camera_depth_image, this, std::placeholders::_1));
    }

    return true;
}

bool UECamera::teardown()
{
    if (m_pushsink_camera_color_image) {
        m_pushsink_camera_color_image->unregisterCallback();
    }

    if (m_pushsink_camera_depth_image) {
        m_pushsink_camera_depth_image->unregisterCallback();
    }

    m_pushsink_camera_color_image.release();
    m_pullsink_camera_color_model.release();
    m_pullsink_camera_color_pose.release();

    m_pushsink_camera_depth_image.release();
    m_pullsink_camera_depth_model.release();
    m_pullsink_camera_depth_pose.release();

    return true;
}

bool UECamera::camera_color_get_model(TimestampT ts, double, double, Eigen::Matrix4d & projection_matrix, Eigen::Matrix3d & intrinsics_matrix, Eigen::Vector2i & resolution)
{
    camera_color->camera_get_calibration(projection_matrix, intrinsics_matrix, resolution);
    return true;
}

bool UECamera::camera_depth_get_model(TimestampT ts, double, double, Eigen::Matrix4d & projection_matrix, Eigen::Matrix3d & intrinsics_matrix, Eigen::Vector2i & resolution)
{
    camera_depth->camera_get_calibration(projection_matrix, intrinsics_matrix, resolution);
    return true;
}

bool UECamera::camera_color_get_pose(TimestampT, Eigen::Matrix4d & pose)
{
    pose = camera_color->get_extrinsic_matrix();
    return true;
}

bool UECamera::camera_depth_get_pose(TimestampT ts, Eigen::Matrix4d & pose)
{
    pose = camera_depth->get_extrinsic_matrix();
    return true;
}

bool UECamera::camera_color_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& img)
{
    std::unique_ptr<cv::Mat> ptr_image = camera_color->getImage(currentID_color);

    /// Measurement is in nanoseconds, 10^9 (written as (10^3)^3 for readability
    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> colorImage = std::make_shared < Ubitrack::Facade::BasicImageMeasurement>(
        static_cast<Ubitrack::Measurement::Timestamp>(start_point + currentID_color * 1000 * 1000 * 1000 / camera_depth->get_frames_per_second()),
        ptr_image->cols,
        ptr_image->rows,
        ptr_image->depth(),
        ptr_image->channels(),
        ptr_image->data,
        Ubitrack::Facade::BasicImageMeasurement::DEPTH,
        true);

    img = colorImage;
    currentID_color++;

    return true;
}

bool UECamera::camera_depth_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& img)
{
    std::unique_ptr<cv::Mat> ptr_image = camera_depth->getImage(currentID_depth);

    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> depthImage = std::make_shared < Ubitrack::Facade::BasicImageMeasurement>(
        static_cast<Ubitrack::Measurement::Timestamp>(start_point + currentID_depth * 1000000000 / camera_depth->get_frames_per_second()),
        ptr_image->cols,
        ptr_image->rows,
        ptr_image->depth(),
        ptr_image->channels(),
        ptr_image->data,
        Ubitrack::Facade::BasicImageMeasurement::DEPTH,
        true);

    img = depthImage;
    currentID_depth++;

    return true;
}

void UECamera::receive_camera_color_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& image)
{
    {
        std::unique_lock<std::mutex> ul(m_textureAccessMutex_color);
        m_current_camera_color_image = image;
    }
}

void UECamera::receive_camera_depth_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& image)
{
    {
        std::unique_lock<std::mutex> ul(m_textureAccessMutex_depth);
        m_current_camera_depth_image = image;
    }
}
