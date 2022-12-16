#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__PINHOLE_CAMERA_MODEL_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__PINHOLE_CAMERA_MODEL_HPP

#include <stdexcept>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Eigen>

namespace jir_refractive_image_geometry {

class Exception : public std::runtime_error
{
public:
	Exception(const std::string& description) : std::runtime_error(description) {}
};

enum DistortionState { NONE, CALIBRATED, UNKNOWN };


template < typename Derived >
class PinholeCameraModelParametersBase {
public:
	typedef typename Derived::Scalar Scalar;

	size_t numberOfDistortionCoeffs() const { return Derived::numberOfDistortionCoeffs(); }

	/// Focal length in x
	const Scalar& fx() const { return Derived::fx(); }
	/// Focal length in y
	const Scalar& fy() const { return Derived::fy(); }
	/// Principal point in x
	const Scalar& cx() const { return Derived::cx(); }
	/// Principal point in y
	const Scalar& cy() const { return Derived::cy(); }
	/// Distortion Parameters
	const Scalar& distortionCoeff(size_t i) const { return Derived::distortionCoeff(i); }

	/// Focal length in x
	Scalar& fx() { return Derived::fx(); }
	/// Focal length in y
	Scalar& fy() { return Derived::fy(); }
	/// Principal point in x
	Scalar& cx() { return Derived::cx(); }
	/// Principal point in y
	Scalar& cy() { return Derived::cy(); }
	/// Distortion Parameters
	Scalar& distortionCoeff(size_t i) { return Derived::distortionCoeff(i); }
};

template < typename Scalar_ >
class PlumbBobPinholeCameraModelParametersVector : public PinholeCameraModelParametersBase< PinholeCameraModelParametersPinholeCameraModelParametersVector<Scalar_> {
public:
	typedef Scalar_ Scalar;
	typedef Eigen::Matrix<Scalar, 4+num_distortion_coeffs, 1> ParameterVector;

	ParameterVector values;

	size_t length() const { return 4+num_distortion_coeffs; }
	Scalar* data() { return values.data(); }

	PinholeCameraModelParameters()
}

template < typename Scalar >
class PinholeCameraModel {
public:
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
	typedef Eigen::Matrix<Scalar,3,3> Matrix33;
	typedef Eigen::Matrix<Scalar,3,4> Matrix34;

protected:
	

	Vector   D_;		// Unaffected by binning, ROI
	Matrix33 R_;		// Unaffected by binning, ROI
	Matrix33 K_;		// Describe current image (includes binning, ROI)
	Matrix34 P_;		// Describe current image (includes binning, ROI)
	Matrix33 K_full_;	// Describe full-res image, needed for full maps
	Matrix34 P_full_;	// Describe full-res image, needed for full maps

	struct CalibCache {

	};


	sensor_msgs::CameraInfo cam_info_;
public:
	virtual ~PinholeCameraModel() {}

	PinholeCameraModel() {}

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfo& msg) {

	}

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo message.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

	}

	/**
	* \brief Returns true if the camera has been initialized
	*/
	bool initialized() const { return (bool)cache_; }



	/* Trivial inline functions */
	inline std::string tfFrame() const
	{
	  assert( initialized() );
	  return cam_info_.header.frame_id;
	}

	inline ros::Time stamp() const
	{
	  assert( initialized() );
	  return cam_info_.header.stamp;
	}

	inline const sensor_msgs::CameraInfo& cameraInfo() const  { return cam_info_; }
	inline const Matrix33& intrinsicMatrix() const  { return K_; }
	inline const Vector& distortionCoeffs() const { return D_; }
	inline const Matrix33& rotationMatrix() const   { return R_; }
	inline const Matrix34& projectionMatrix() const { return P_; }
	inline const Matrix33& fullIntrinsicMatrix() const  { return K_full_; }
	inline const Matrix34& fullProjectionMatrix() const { return P_full_; }

	inline Scalar fx() const { return P_(0,0); }
	inline Scalar fy() const { return P_(1,1); }
	inline Scalar cx() const { return P_(0,2); }
	inline Scalar cy() const { return P_(1,2); }
	inline Scalar Tx() const { return P_(0,3); }
	inline Scalar Ty() const { return P_(1,3); }

	/**
	 * \brief Returns the number of columns in each bin.
	 */
	inline uint32_t binningX() const { return cam_info_.binning_x; }
	/**
	 * \brief Returns the number of rows in each bin.
	 */
	inline uint32_t binningY() const { return cam_info_.binning_y; }

	/**
	 * \brief Compute delta u, given Z and delta X in Cartesian space.
	 *
	 * For given Z, this is the inverse of getDeltaX().
	 *
	 * \param deltaX Delta X, in Cartesian space
	 * \param Z      Z (depth), in Cartesian space
	 */
	inline Scalar getDeltaU(Scalar deltaX, Scalar Z) const
	{
	  assert( initialized() );
	  return fx() * deltaX / Z;
	}

	/**
	 * \brief Compute delta v, given Z and delta Y in Cartesian space.
	 *
	 * For given Z, this is the inverse of getDeltaY().
	 *
	 * \param deltaY Delta Y, in Cartesian space
	 * \param Z      Z (depth), in Cartesian space
	 */
	inline Scalar getDeltaV(Scalar deltaY, Scalar Z) const
	{
	  assert( initialized() );
	  return fy() * deltaY / Z;
	}

	/**
	 * \brief Compute delta X, given Z in Cartesian space and delta u in pixels.
	 *
	 * For given Z, this is the inverse of getDeltaU().
	 *
	 * \param deltaU Delta u, in pixels
	 * \param Z      Z (depth), in Cartesian space
	 */
	inline Scalar getDeltaX(Scalar deltaU, Scalar Z) const
	{
	  assert( initialized() );
	  return Z * deltaU / fx();
	}

	/**
	 * \brief Compute delta Y, given Z in Cartesian space and delta v in pixels.
	 *
	 * For given Z, this is the inverse of getDeltaV().
	 *
	 * \param deltaV Delta v, in pixels
	 * \param Z      Z (depth), in Cartesian space
	 */
	inline Scalar getDeltaY(Scalar deltaV, Scalar Z) const
	{
	  assert( initialized() );
	  return Z * deltaV / fy();
	}
};


}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__PINHOLE_CAMERA_MODEL_HPP
