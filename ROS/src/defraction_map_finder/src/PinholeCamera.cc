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

#include "defraction_map_finder/PinholeCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//include "../gpl/gpl.h"

namespace camodocal
{

PinholeCamera::Parameters::Parameters()
 : Camera::Parameters(PINHOLE)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_fx(0.0)
 , m_fy(0.0)
 , m_cx(0.0)
 , m_cy(0.0)
{

}

PinholeCamera::Parameters::Parameters(const std::string& cameraName,
                                      int w, int h,
                                      double k1, double k2,
                                      double p1, double p2,
                                      double fx, double fy,
                                      double cx, double cy)
 : Camera::Parameters(PINHOLE, cameraName, w, h)
 , m_k1(k1)
 , m_k2(k2)
 , m_p1(p1)
 , m_p2(p2)
 , m_fx(fx)
 , m_fy(fy)
 , m_cx(cx)
 , m_cy(cy)
{
}

double&
PinholeCamera::Parameters::k1(void)
{
    return m_k1;
}

double&
PinholeCamera::Parameters::k2(void)
{
    return m_k2;
}

double&
PinholeCamera::Parameters::p1(void)
{
    return m_p1;
}

double&
PinholeCamera::Parameters::p2(void)
{
    return m_p2;
}

double&
PinholeCamera::Parameters::fx(void)
{
    return m_fx;
}

double&
PinholeCamera::Parameters::fy(void)
{
    return m_fy;
}

double&
PinholeCamera::Parameters::cx(void)
{
    return m_cx;
}

double&
PinholeCamera::Parameters::cy(void)
{
    return m_cy;
}

double
PinholeCamera::Parameters::k1(void) const
{
    return m_k1;
}

double
PinholeCamera::Parameters::k2(void) const
{
    return m_k2;
}

double
PinholeCamera::Parameters::p1(void) const
{
    return m_p1;
}

double
PinholeCamera::Parameters::p2(void) const
{
    return m_p2;
}

double
PinholeCamera::Parameters::fx(void) const
{
    return m_fx;
}

double
PinholeCamera::Parameters::fy(void) const
{
    return m_fy;
}

double
PinholeCamera::Parameters::cx(void) const
{
    return m_cx;
}

double
PinholeCamera::Parameters::cy(void) const
{
    return m_cy;
}

bool
PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("PINHOLE") != 0)
        {
            return false;
        }
    }

    m_modelType = PINHOLE;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    m_k1 = static_cast<double>(n["k1"]);
    m_k2 = static_cast<double>(n["k2"]);
    m_p1 = static_cast<double>(n["p1"]);
    m_p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    m_fx = static_cast<double>(n["fx"]);
    m_fy = static_cast<double>(n["fy"]);
    m_cx = static_cast<double>(n["cx"]);
    m_cy = static_cast<double>(n["cy"]);

    return true;
}

void
PinholeCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "PINHOLE";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    fs << "distortion_parameters";
    fs << "{" << "k1" << m_k1
              << "k2" << m_k2
              << "p1" << m_p1
              << "p2" << m_p2 << "}";

    // projection: fx, fy, cx, cy
    fs << "projection_parameters";
    fs << "{" << "fx" << m_fx
              << "fy" << m_fy
              << "cx" << m_cx
              << "cy" << m_cy << "}";

    fs.release();
}

PinholeCamera::Parameters&
PinholeCamera::Parameters::operator=(const PinholeCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_k1 = other.m_k1;
        m_k2 = other.m_k2;
        m_p1 = other.m_p1;
        m_p2 = other.m_p2;
        m_fx = other.m_fx;
        m_fy = other.m_fy;
        m_cx = other.m_cx;
        m_cy = other.m_cy;
    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const PinholeCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type " << "PINHOLE" << std::endl;
    out << "   camera_name " << params.m_cameraName << std::endl;
    out << "   image_width " << params.m_imageWidth << std::endl;
    out << "  image_height " << params.m_imageHeight << std::endl;

    // radial distortion: k1, k2
    // tangential distortion: p1, p2
    out << "Distortion Parameters" << std::endl;
    out << "            k1 " << params.m_k1 << std::endl
        << "            k2 " << params.m_k2 << std::endl
        << "            p1 " << params.m_p1 << std::endl
        << "            p2 " << params.m_p2 << std::endl;

    // projection: fx, fy, cx, cy
    out << "Projection Parameters" << std::endl;
    out << "            fx " << params.m_fx << std::endl
        << "            fy " << params.m_fy << std::endl
        << "            cx " << params.m_cx << std::endl
        << "            cy " << params.m_cy << std::endl;

    return out;
}

PinholeCamera::PinholeCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{

}

PinholeCamera::PinholeCamera(const std::string& cameraName,
                             int imageWidth, int imageHeight,
                             double k1, double k2, double p1, double p2,
                             double fx, double fy, double cx, double cy)
 : mParameters(cameraName, imageWidth, imageHeight,
               k1, k2, p1, p2, fx, fy, cx, cy)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

PinholeCamera::PinholeCamera(const PinholeCamera::Parameters& params)
 : mParameters(params)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

Camera::ModelType
PinholeCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
PinholeCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
PinholeCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
PinholeCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}


/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}


/**
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */


/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

/**
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate Jacobian
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */

const PinholeCamera::Parameters&
PinholeCamera::getParameters(void) const
{
    return mParameters;
}

void
PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)
{
    mParameters = parameters;

    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}


}
