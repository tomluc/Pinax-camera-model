#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_HPP

#include <string>
#include <eigen3/Eigen/Eigen>

namespace jir_refractive_image_geometry {

template < typename Scalar_ >
class Ray {
public:
	typedef Scalar_ Scalar;
	typedef Eigen::Matrix<Scalar,3,1> Vector;

	std::string frame_id;
	
	bool was_totally_reflected;
	Vector offset;
	Vector direction;

	Ray() : was_totally_reflected(false), offset(0,0,0), direction(0,0,1) {}

	Ray(const std::string& f_id, Scalar x, Scalar y, Scalar z) : frame_id(f_id), was_totally_reflected(false), offset(0,0,0), direction(x,y,z) {
		direction.normalize();
	}

	Ray(const Ray<Scalar>& o) : frame_id(o.frame_id), was_totally_reflected(o.was_totally_reflected), offset(o.offset), direction(o.direction) {
		direction.normalize();
	}

	Vector operator()(Scalar d) const {
		return offset + direction * d;
	}
};

typedef Ray<double> Rayd;

}


#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_HPP
