#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTION_OPERATIONS_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTION_OPERATIONS_HPP

#include <jir_refractive_image_geometry_msgs/PlanarRefractionInfo.h>

#include <jir_refractive_image_geometry/ray.hpp>
#include <jir_refractive_image_geometry/ray_operations.hpp>

#include <jir_refractive_image_geometry/polynomial.hpp>
#include <jir_refractive_image_geometry/polynomial_roots_gsl.hpp>

namespace jir_refractive_image_geometry {
using namespace Eigen;
using namespace std;

template < typename Derived >
void refract_at_interface(const MatrixBase<Derived>& l, const MatrixBase<Derived>& n, const typename internal::traits<Derived>::Scalar& n1, const typename internal::traits<Derived>::Scalar& n2, MatrixBase<Derived>& out_v) {
	typedef typename internal::traits<Derived>::Scalar Scalar;

	// from Wikipedia 

	Scalar c = -n.dot(l);
	Scalar r = n1/n2;

	Scalar s = Scalar(1.0) - r*r * ( Scalar(1.0) - c*c );

	if( s > Scalar(0.0) ) { // (partial) refraction
		out_v = r * l + (r*c - sqrt(s)) * n;
	} else { // total reflection
		out_v = l + Scalar(2.0) * c * n;
	}
}

template< typename Scalar >
void refract(const Ray<Scalar>& input, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& params, Ray<Scalar>& output) {
	if( input.frame_id != params.header.frame_id ) {
		throw std::runtime_error("Ray and PlanarRefractionInfo are not in the same coordinate frame!");
	}

	output = input;
	Matrix<Scalar,3,1> plane_normal( Scalar(params.normal[0]), Scalar(params.normal[1]), Scalar(params.normal[2]) );
	Scalar plane_d_0( params.d_0 );
	Scalar plane_d_1 = Scalar(params.d_0) + Scalar(params.d_1);

	Scalar mu1 = Scalar(params.n_glass);
	Scalar mu2 = Scalar(params.n_water);


	Matrix<Scalar,3,1> new_offset;
	if( intersect(output, plane_normal, plane_d_0, new_offset)) {
		output.offset = new_offset;

		refract_at_interface(output.direction, plane_normal, 1, mu1, output.direction);
	}

	if( intersect(output, plane_normal, plane_d_1, new_offset)) {
		output.offset = new_offset;

		refract_at_interface(output.direction, plane_normal, mu1, mu2, output.direction);
	}

}

template< typename Derived >
void point_on_inside_of_refractive_plane_analytic(const MatrixBase<Derived>& in_outside_point, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& params, MatrixBase<Derived>& out_inside_point) {
	typedef typename internal::traits<Derived>::Scalar Scalar;
	
	Matrix<Scalar,3,1> plane_normal( Scalar(params.normal[0]), Scalar(params.normal[1]), Scalar(params.normal[2]) );
	Scalar plane_d_0( params.d_0 );
	Scalar plane_d_1( params.d_1 );

	//plane_d_1 += plane_d_0;

	Scalar mu1 = Scalar(params.n_glass);
	Scalar mu2 = Scalar(params.n_water);

	//plane_normal = -plane_normal;

	Matrix<Scalar, 3,1> POR = plane_normal.cross(in_outside_point).normalized();

	Matrix<Scalar, 3,1> z1 = -plane_normal;
	Matrix<Scalar, 3,1> z2 = POR.cross(z1).normalized();

	Scalar u_y = in_outside_point.transpose() * z1;
	Scalar u_x = in_outside_point.transpose() * z2;

	/*
	cout << "POR: " << POR.transpose() << endl;
	cout << "z1: " << z1.transpose() << endl;
	cout << "z2: " << z2.transpose() << endl;
	cout << "v: " << u_y << endl;
	cout << "u: " << u_x << endl;
	//*/
	
	// build poly, from supplementary.pdf
	vector<Scalar> k1 = linear<Scalar>( plane_d_0+plane_d_1-u_y,   0 );
	vector<Scalar> k2 = linear<Scalar>(                      -1, u_x );
	vector<Scalar> k3 = linear<Scalar>(              -plane_d_1,   0 );
	vector<Scalar> D1 = quadratic<Scalar>( mu1*mu1-1, 0, plane_d_0*plane_d_0 * mu1*mu1 );
	vector<Scalar> D2 = quadratic<Scalar>( mu2*mu2-1, 0, plane_d_0*plane_d_0 * mu2*mu2 );

	vector<Scalar> t1 = pow( 
							sub( 
								add( 
									mult( 
										pow(k1,2), 
										D1 
									), 
									mult( 
										pow(k3,2), 
										D2 
									) 
								),
								mult( 
									mult( 
										pow(k2,2), 
										D1 
									), 
									D2 
								) 
							)
						, 2) ;
	vector<Scalar> t2 = mult( 
							Scalar(4), 
							mult(
								mult( 
									pow(k1,2), 
									pow(k3,2)
								),
								mult(
									D1,
									D2
								)
							)
						);

	vector<Scalar> p = sub( t1, t2 );

	/*
	cout << "degree of p: " << degree(p) << endl;

	cout << endl << endl;
	cout << "P:" << endl;
	for(size_t i=p.size(); i>0; i--) {
		cout << "x^" << i-1 << " -> " << p[i-1] << endl;
	}
	cout << endl << endl;
	//*/

	vector<Scalar> roots = real_roots(p);

	//cout << "found " << roots.size() << " roots!" << endl;


	Scalar error(1e100);
	for(size_t i=0; i<roots.size(); i++) {
		//cout << "root: " << roots[i] << endl;

		Matrix<Scalar, 3,1> candidate = roots[i] * z2 + plane_d_0 * z1;

		//cout << "Candiate: " << candidate.normalized().transpose() << endl;

		Ray<Scalar> r_inside, r_outside;
		r_inside.frame_id = params.header.frame_id;
		r_inside.direction = candidate.normalized();

		refract(r_inside, params, r_outside);

		Scalar dist;
		square_distance(r_outside, in_outside_point, dist);

		//cout << "ERROR: " << dist << endl;

		if( error > dist ) {
			error = dist;
			out_inside_point = candidate;
		}
	}

}


namespace point_on_inside_of_refractive_plane_iterative__DETAIL {

template < typename Derived >
typename internal::traits<Derived>::Scalar distance(typename internal::traits<Derived>::Scalar x, const MatrixBase<Derived>& p, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& params) {
	typedef typename internal::traits<Derived>::Scalar Scalar;

	Matrix<Scalar,3,1> plane_normal( Scalar(params.normal[0]), Scalar(params.normal[1]), Scalar(params.normal[2]) );
	Scalar plane_d_0( params.d_0 );
	Scalar plane_d_1( params.d_1 );
	

	Scalar mu1 = Scalar(params.n_glass);
	Scalar mu2 = Scalar(params.n_water);

	//plane_normal = -plane_normal;

	Matrix<Scalar, 3,1> POR = plane_normal.cross(p).normalized();

	if( abs(plane_normal.dot(p.normalized())) > (1.0-1e-8) ) {
		// plane normal and p are parallel, choose any POR
		POR[0]=1; POR[1]=1;
		POR[2] = -(plane_normal[0]+plane_normal[1])/plane_normal[2];
		POR.normalize();

		//cout << "Check: " << POR.dot(plane_normal) << endl;
	}

	Matrix<Scalar, 3,1> z1 = -plane_normal;
	Matrix<Scalar, 3,1> z2 = POR.cross(z1).normalized();

	Ray<Scalar> r_inside;
	r_inside.frame_id = params.header.frame_id;
	r_inside.direction = x*z2 + plane_d_0*z1;
	r_inside.direction.normalize();


	Ray<Scalar> r_outside;
	refract(r_inside, params, r_outside);

	Scalar dist;
	square_distance(r_outside, p, dist);
	return dist;	
}

template < typename Derived >
void write_3d_point(typename internal::traits<Derived>::Scalar x, const MatrixBase<Derived>& in_outside_point, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& params, MatrixBase<Derived>& out_inside_point) {
	typedef typename internal::traits<Derived>::Scalar Scalar;

	Matrix<Scalar,3,1> plane_normal( Scalar(params.normal[0]), Scalar(params.normal[1]), Scalar(params.normal[2]) );
	Scalar plane_d_0( params.d_0 );
	Scalar plane_d_1( params.d_1 );

	plane_d_1 += plane_d_0;

	Scalar mu1 = Scalar(params.n_glass);
	Scalar mu2 = Scalar(params.n_water);

	//plane_normal = -plane_normal;

	Matrix<Scalar, 3,1> POR = plane_normal.cross(in_outside_point).normalized();

	if( abs(plane_normal.dot(in_outside_point.normalized())) > (1.0-1e-9) ) {
		// plane normal and p are parallel, choose any POR
		POR[0]=1; POR[1]=1;
		POR[2] = -(plane_normal[0]+plane_normal[1])/plane_normal[2];
		POR.normalize();

		//cout << "Check: " << POR.dot(plane_normal) << endl;
	}

	Matrix<Scalar, 3,1> z1 = -plane_normal;
	Matrix<Scalar, 3,1> z2 = POR.cross(z1).normalized();

	out_inside_point = x*z2 + plane_d_0*z1;
}

}

template< typename Derived >
typename boost::enable_if< boost::is_same< typename internal::traits<Derived>::Scalar, double>, void>::type point_on_inside_of_refractive_plane_iterative(const MatrixBase<Derived>& in_outside_point, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& params, MatrixBase<Derived>& out_inside_point, typename internal::traits<Derived>::Scalar tol = 1e-10, size_t max_iterations = 100) {
	using namespace point_on_inside_of_refractive_plane_iterative__DETAIL;
	typedef typename internal::traits<Derived>::Scalar Scalar;


	Matrix<Scalar,3,1> plane_normal( Scalar(params.normal[0]), Scalar(params.normal[1]), Scalar(params.normal[2]) );
	if( abs(plane_normal.dot(in_outside_point.normalized())) < 1e-9 ) {
		// orthogonal, does not intersect refraction plane!
		out_inside_point = in_outside_point;
		return;
	}

	

	Scalar x(0);
	Scalar h(1e-4);

	Scalar f = distance(x, in_outside_point, params);

	cout << "f("<< x << ") = " << f << endl;

	if( f > tol ) {
		for( size_t i=0; i < max_iterations; i++ ) {
			Scalar f_ph = distance(x+h, in_outside_point, params);
			//Scalar f_p2h = distance(x+2*h, in_outside_point, params);
			Scalar f_mh = distance(x-h, in_outside_point, params);
			//Scalar f_m2h = distance(x-2*h, in_outside_point, params);
			

			/*
			Scalar f_p  = ( 1./12. * f_m2h - 2./3. * f_mh            + 2./3. * f_ph  -1./12. * f_p2h ) / (h);
			Scalar f_pp = (-1./12. * f_m2h + 4./3. * f_mh -5./2. * f + 4./3. * f_ph  -1./12. * f_p2h ) / (h*h);
			//*/

			//*  2 accuracy
			Scalar f_p   = (-.5 * f_mh         + .5 * f_ph ) / (h);
			Scalar f_pp  = (      f_mh -2. * f +      f_ph ) / (h*h);
			//Scalar f_ppp = (-.5*f_m2h + f_mh - f_ph + .5*f_p2h ) / (h*h*h);
			//*/

			cout << "f_p:   " << f_p << endl;
			cout << "f_pp:  " << f_pp << endl;
			//cout << "f_ppp: " << f_ppp << endl;

			if( f_pp <= 0) {
				x +=  -f/f_p;
			} else {
				x += -f_p/f_pp;
			}
			

			f = distance(x, in_outside_point, params);

			cout << "f("<< x << ") = " << f << endl;

			if( f < tol ) {
				cout << "needed " << i << " iterations" << endl;
				break;
			}
		}
	}

	write_3d_point(x, in_outside_point, params, out_inside_point);
}

}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTION_OPERATIONS_HPP
