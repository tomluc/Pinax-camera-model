#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY_MSGS__HELPERS_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY_MSGS__HELPERS_HPP

#include <eigen3/Eigen/Eigen>

#include <jir_refractive_image_geometry_msgs/PlanarRefractionInfo.h>

namespace jir_refractive_image_geometry_msgs {

inline void transform(const PlanarRefractionInfo& in, const Eigen::Affine3d& tr_in_out, PlanarRefractionInfo& out) {
	out = in;

	Eigen::Map< const Eigen::Vector3d > normal_in  ( &in.normal[0] );
	Eigen::Map< Eigen::Vector3d > normal_out ( &out.normal[0] );

	normal_out = tr_in_out.rotation() * normal_in;

	out.d_0 = normal_out.dot( tr_in_out * ( in.d_0 * normal_in ));
}

}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY_MSGS__HELPERS_HPP
