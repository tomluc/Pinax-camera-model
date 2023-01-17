#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTIVE_PLANE_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTIVE_PLANE_HPP

#include <string>
#include <Eigen/Eigen>


namespace Eigen {
namespace internal {

template<typename _Scalar, int _Options>
struct traits<jir_refractive_image_geometry::TwoLayerRefraction<_Scalar,_Options> > {
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,7,1> ParametersType;
};

template<typename _Scalar, int _Options>
struct traits<Map<jir_refractive_image_geometry::TwoLayerRefraction<_Scalar>, _Options> >
    : traits<jir_refractive_image_geometry::TwoLayerRefraction<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<Matrix<Scalar,7,1>,_Options> ParametersType;
};

template<typename _Scalar, int _Options>
struct traits<Map<const jir_refractive_image_geometry::TwoLayerRefraction<_Scalar>, _Options> >
    : traits<const jir_refractive_image_geometry::TwoLayerRefraction<_Scalar, _Options> > {
  typedef _Scalar Scalar;
  typedef Map<const Matrix<Scalar,7,1>,_Options> ParametersType;
};

}
}

namespace jir_refractive_image_geometry {
using namespace Eigen;

template < typename Derived >
class TwoLayerRefractionBase {
public:
	typedef typename internal::traits<Derived>::Scalar Scalar;

	typedef typename internal::traits<Derived>::ParametersType & ParametersReference;
	typedef const typename internal::traits<Derived>::ParametersType & ConstParametersReference;

	typedef Matrix<Scalar, 3,1> Vector;

	std::string frame_id;

	Vector normal() const {
		return Derived::normal();
	}

	

};

}

#endif //JIR_REFRACTIVE_IMAGE_GEOMETRY__REFRACTIVE_PLANE_HPP
