/*
CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with
Multiple Generic Cameras and Odometry
Copyright (c) 2013 Lionel Heng

This software is licensed under CC-BY-SA. For more information, visit
http://http://creativecommons.org/licenses/by-sa/2.0/

If you use this software for an academic publication, please cite this paper:
Lionel Heng, Bo Li, and Marc Pollefeys, CamOdoCal: Automatic Intrinsic and
Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry,
In Proc. IEEE/RSJ International Conference on Intelligent Robots
and Systems (IROS), 2013.
*/

#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "defraction_map_finder/Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
