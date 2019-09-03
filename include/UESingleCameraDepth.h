#ifndef BASIC_FACADE_APP_UESINGLECAMERADEPTH_H
#define BASIC_FACADE_APP_UESINGLECAMERADEPTH_H

// include Ubitrack BasicFacade
#include "IUESingleCamera.h"
#include <opencv2/core.hpp>
#include <map>
#include <memory>

// In the UE4 simulation, each virtual camera (Screencapture) consists of 2 cameras with independent poses, but 
// same view axis and rotations. One is color camera, one is depth camera.

// This code implements a singular of those cameras

class UESingleCameraDepth : public IUESingleCamera {
public:
	explicit UESingleCameraDepth(const std::string& _data_source_path);
	~UESingleCameraDepth() = default;

	std::unique_ptr<cv::Mat> getImage(int32_t id) override;
private:
	std::map<int, std::string> entryList;
};

#endif  // BASIC_FACADE_APP_UESINGLECAMERADEPTH_H
