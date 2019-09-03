#ifndef BASIC_FACADE_APP_UESINGLECAMERA_H
#define BASIC_FACADE_APP_UESINGLECAMERA_H

#include <opencv2/core.hpp>
#include <IUESingleCamera.h>

class UESingleCamera : public IUESingleCamera {
public:
    explicit UESingleCamera(const std::string& _data_source_path);
    ~UESingleCamera() = default;

    std::unique_ptr<cv::Mat> getImage(int32_t id) override;
private:
    std::map<int, std::string> entryList;
    std::string basePath;
    float field_of_view;
    float frames_per_second;
};

#endif  // BASIC_FACADE_APP_UESINGLECAMERA_H
