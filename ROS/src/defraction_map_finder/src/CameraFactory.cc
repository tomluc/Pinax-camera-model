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

#include "defraction_map_finder/CameraFactory.h"

#include <boost/algorithm/string.hpp>

//#include "ceres/ceres.h"
#include "defraction_map_finder/CataCamera.h"
#include "defraction_map_finder/EquidistantCamera.h"
#include "defraction_map_finder/PinholeCamera.h"
#include <iostream>
//#include "camodocal/camera_models/EquidistantCamera.h"
//#include "camodocal/camera_models/PinholeCamera.h"

namespace camodocal
{

boost::shared_ptr<CameraFactory> CameraFactory::m_instance (new CameraFactory);

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
        
    }
    case Camera::MEI:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera ;
        
    }
    }
   return CameraPtr(); 
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return CameraPtr();
    }

    Camera::ModelType modelType = Camera::MEI;
    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (boost::iequals(sModelType, "kannala_brandt"))
        {
            modelType = Camera::KANNALA_BRANDT;
        }
        else if (boost::iequals(sModelType, "mei"))
        {
            modelType = Camera::MEI;
        }
        else if (boost::iequals(sModelType, "pinhole"))
        {
            modelType = Camera::PINHOLE;
        }
        else
        {
            std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
            return CameraPtr();
        }
    }

    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    }

    return CameraPtr();
}

}

