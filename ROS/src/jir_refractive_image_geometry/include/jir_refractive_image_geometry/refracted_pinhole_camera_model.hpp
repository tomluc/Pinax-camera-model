#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTED_PINHOLE_CAMERA_MODEL_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTED_PINHOLE_CAMERA_MODEL_HPP

#include <cmath>

#include <image_geometry/pinhole_camera_model.h>

#include <jir_refractive_image_geometry_msgs/PlanarRefractionInfo.h>

#include <jir_refractive_image_geometry/ray.hpp>
#include <jir_refractive_image_geometry/ray_operations.hpp>
#include <jir_refractive_image_geometry/refraction_operations.hpp>

namespace jir_refractive_image_geometry {

class RefractedPinholeCameraModel : public image_geometry::PinholeCameraModel {
public:
	RefractedPinholeCameraModel() : image_geometry::PinholeCameraModel(), planarRefractionInfoIsSet(false) {}

	inline bool fromPlanarRefractionInfo( const jir_refractive_image_geometry_msgs::PlanarRefractionInfoConstPtr& info) {
		return fromPlanarRefractionInfo(*info);
	}
	inline bool fromPlanarRefractionInfo( const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& info) {
		planarRefractionInfoIsSet = true;
		planarRefractionInfo = info;
	}

	template < typename Derived >
	Ray< typename Eigen::internal::traits<Derived>::Scalar > projectPixelTo3dRay(const Eigen::MatrixBase<Derived>& p) const {
		return projectPixelTo3dRay< typename Eigen::internal::traits<Derived>::Scalar >(p(0), p(1));
	}

	template < typename Scalar >
	Ray<Scalar> projectPixelTo3dRay(Scalar u_rectified, Scalar v_rectified) const
	{
		Scalar r0_x = (u_rectified - Scalar(cx()) - Scalar(Tx()) ) / Scalar(fx());
		Scalar r0_y = (v_rectified - Scalar(cy()) - Scalar(Ty()) ) / Scalar(fy());
		
		Ray<Scalar> r_out(cam_info_.header.frame_id, r0_x, r0_y, Scalar(1));

		// maybe transform ray into refraction info frame

		if( planarRefractionInfoIsSet )
		{
			Ray<Scalar> r0 = r_out;
			refract(r0, planarRefractionInfo, r_out);
		}
		
		return r_out;
	}

	template < typename Derived >
	Eigen::Matrix< typename Eigen::internal::traits<Derived>::Scalar, 2, 1 > project3dToPixel(const Eigen::MatrixBase<Derived>& xyz_) const {
		typedef typename Eigen::internal::traits<Derived>::Scalar Scalar;

		Matrix<Scalar,3,1> xyz = xyz_;

		if( planarRefractionInfoIsSet )
		{
			// find point on inside of refraction interface corresponding to beam
			Matrix<Scalar,3,1> xyz_in = xyz;
			point_on_inside_of_refractive_plane_analytic(xyz_in, planarRefractionInfo, xyz); // tolerance 1e-10,  max 100 iterations
		}

		Matrix< Scalar, 2, 1 > p;
		p(0) = ( Scalar(fx()) * xyz(0) + Scalar(Tx()) ) / xyz(2) + Scalar(cx());
		p(1) = ( Scalar(fy()) * xyz(1) + Scalar(Ty()) ) / xyz(2) + Scalar(cy());

		return p;
	}


protected:
	jir_refractive_image_geometry_msgs::PlanarRefractionInfo planarRefractionInfo;
	bool planarRefractionInfoIsSet;


//	struct gsl_roots_workspace;
//	boost::shared_ptr<gsl_roots_workspace> gsl_roots;
};

}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTED_PINHOLE_CAMERA_MODEL_HPP
