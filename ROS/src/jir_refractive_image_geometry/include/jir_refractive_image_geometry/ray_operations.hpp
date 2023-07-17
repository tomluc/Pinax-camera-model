#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_OPERATIONS_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_OPERATIONS_HPP

#include <jir_refractive_image_geometry/ray.hpp>

namespace jir_refractive_image_geometry {
using namespace Eigen;

template < typename Scalar, int Mode, int Options >
Ray<Scalar> transformRay( const Ray<Scalar>& r_in, const Eigen::Transform<Scalar,3,Mode,Options>& tr_in_out, const std::string& new_frame_id = "") {
	Ray<Scalar> out;

	out.offset = tr_in_out * r_in.offset;
	out.direction = tr_in_out.rotation() * r_in.direction;

	out.frame_id = new_frame_id;

	return out;
}

template < typename Scalar, typename Derived1, typename Derived3 >
bool intersect(const Ray<Scalar>& r, const Eigen::MatrixBase<Derived1>& plane_normal, const Scalar& plane_distance, Eigen::MatrixBase<Derived3>& point) {
	Scalar nd = plane_normal.dot( r.direction );
	if(std::fabs(nd) < 1e-10) return false;

	Scalar t = ( -plane_distance - plane_normal.dot( r.offset ) ) / nd;

	if( t < 1e-10 ) return false;

	point = r.offset + t*r.direction;
	return true;
}

template < typename Scalar, typename Derived1, typename Derived2 >
void closest_points(const Ray<Scalar>& r1, const Ray<Scalar>& r2, MatrixBase<Derived1>& p_on_r1, MatrixBase<Derived2>& p_on_r2) {
	Matrix<Scalar,3,1> m = r1.offset - r2.offset;

	Scalar d11 = r1.direction.dot(r1.direction);
	Scalar d12 = r1.direction.dot(r2.direction);
	Scalar d22 = r2.direction.dot(r2.direction);
	Scalar d1m = r1.direction.dot(m);
	Scalar d2m = r2.direction.dot(m);

	Scalar d   = d11*d22 - d12*d12;
	Scalar x1  = (d12*d2m - d22*d1m)/d;
	Scalar x2  = (d11*d2m - d12*d1m)/d;

	p_on_r1 = r1.offset;
	p_on_r2 = r2.offset;

	if( x1 > 0 ) p_on_r1 += x1*r1.direction;
	if( x2 > 0 ) p_on_r2 += x2*r2.direction;
}

template < typename Derived >
void triangulate(const Ray< typename internal::traits<Derived>::Scalar >& r1, const Ray< typename internal::traits<Derived>::Scalar >& r2, MatrixBase<Derived>& p) {
	Matrix< typename internal::traits<Derived>::Scalar ,3,1> p1, p2;
	closest_points(r1,r2,p1,p2);
	p = (p1+p2)/( typename internal::traits<Derived>::Scalar(2) );
}

template < typename Scalar >
void distance(const Ray<Scalar>& r1, const Ray<Scalar>& r2, Scalar& dist) {
	Matrix<Scalar,3,1> p1, p2;
	closest_points(r1,r2,p1,p2);
	dist = (p1-p2).norm();
}

template < typename Derived >
void distance(const Ray< typename internal::traits<Derived>::Scalar >& r, const MatrixBase<Derived>& p, typename internal::traits<Derived>::Scalar& dist) {
	typename internal::traits<Derived>::Scalar len = (p-r.offset).dot(r.direction);
	if( len > typename internal::traits<Derived>::Scalar(0) ) {
		dist = (p - (len*r.direction + r.offset)).norm();
	} else {
		dist = (p-r.offset).norm();
	}
}

template < typename Derived >
void square_distance(const Ray< typename internal::traits<Derived>::Scalar >& r, const MatrixBase<Derived>& p, typename internal::traits<Derived>::Scalar& dist) {
	typename internal::traits<Derived>::Scalar len = (p-r.offset).dot(r.direction);
	if( len > typename internal::traits<Derived>::Scalar(0) ) {
		dist = (p - (len*r.direction + r.offset)).squaredNorm();
	} else {
		dist = (p-r.offset).squaredNorm();
	}
}

}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__RAY_OPERATIONS_HPP
