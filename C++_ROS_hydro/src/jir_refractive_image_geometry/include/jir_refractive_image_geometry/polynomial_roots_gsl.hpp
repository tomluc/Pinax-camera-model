#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_ROOTS_GSL_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_ROOTS_GSL_HPP

#include <complex>

#include <gsl/gsl_poly.h>

namespace jir_refractive_image_geometry {
using namespace std;

template< typename Scalar>
inline vector<Scalar> real_roots(const vector<Scalar>& v);

template<> 
inline vector<double> real_roots<double>(const vector<double>& v) {
	size_t deg = degree(v);
	gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc ( deg+1 );

	vector< complex<double> > roots(deg);

	gsl_poly_complex_solve(&v[0], deg+1, w, (double*)&roots[0]);

	gsl_poly_complex_workspace_free (w);

	vector<double> ret; ret.reserve(deg);
	for(size_t i=0; i<roots.size(); i++) {
		if( abs( imag(roots[i]) ) < 1e-10 ) {
			ret.push_back( real(roots[i]) );
		}
	}

	return ret;
}

}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_ROOTS_GSL_HPP
