#ifndef JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_HPP
#define JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_HPP

#include <vector>
#include <string>
#include <sstream>

namespace jir_refractive_image_geometry {
using namespace std;

template < typename Scalar >
vector<Scalar> cubic(Scalar x3, Scalar x2, Scalar x1, Scalar x0) {
	vector<Scalar> v;

	v.push_back(x0);
	v.push_back(x1);
	v.push_back(x2);
	v.push_back(x3);

	return v;
}

template < typename Scalar >
vector<Scalar> quadratic(Scalar x2, Scalar x1, Scalar x0) {
	vector<Scalar> v;

	v.push_back(x0);
	v.push_back(x1);
	v.push_back(x2);

	return v;
}

template < typename Scalar >
vector<Scalar> linear(Scalar x1, Scalar x0) {
	vector<Scalar> v;
	
	v.push_back(x0);
	v.push_back(x1);

	return v;
}

template < typename Scalar >
vector<Scalar> add(const vector<Scalar>& v1, const vector<Scalar>& v2) {
	vector<Scalar> ret( max(v1.size(), v2.size() ), Scalar(0.) );
	for(size_t i=0; i<v1.size(); i++) {
		ret[i] = v1[i];
	}
	for(size_t i=0; i<v2.size(); i++) {
		ret[i] += v2[i];
	}
	return ret;
}

template < typename Scalar >
vector<Scalar> sub(const vector<Scalar>& v1, const vector<Scalar>& v2) {
	vector<Scalar> ret( max(v1.size(), v2.size() ), Scalar(0.) );
	for(size_t i=0; i<v1.size(); i++) {
		ret[i] = v1[i];
	}
	for(size_t i=0; i<v2.size(); i++) {
		ret[i] -= v2[i];
	}
	return ret;
}

template < typename Scalar >
size_t degree(const vector<Scalar>& v) {
	Scalar zero(0);

	size_t deg = v.size()-1;
	while(v[deg] == zero && deg > 0) {
		deg--;
	}

	return deg;
}

template < typename Scalar >
string pretty_string(const vector<Scalar>& v, const std::string& var) {
	ostringstream out;
	pretty_print(out, v, var);
	return out.str();
}

template < typename Scalar >
void pretty_print(std::ostream& out, const vector<Scalar>& v, const std::string& var) {
	bool first = true;
	Scalar zero(0);

	for(size_t i=v.size(); i>0; i--) { // because size_t is unsigned, and after 0 comes max
		if( v[i-1] == zero ) continue;

		if( !first ) {
			out << " ";
			if( v[i-1] >= zero) {
				out << "+";
			}
		}
		out << v[i-1];
		if( i > 1 ) {
			out << " " << var << "^{" << (i-1) << "}";
		}
		first = false;
	}
}

template < typename Scalar >
vector<Scalar> mult(Scalar s, const vector<Scalar>& v) {
	vector<Scalar> ret(v);
	for(size_t i=0; i<ret.size(); i++) {
		ret[i] *= s;
	}
	return ret;
}


template < typename Scalar >
vector<Scalar> mult(const vector<Scalar>& v1, const vector<Scalar>& v2) {
	size_t new_degree = degree(v1)+degree(v2);
	//cout << "new deg: " << new_degree << endl;

	vector<Scalar> ret(new_degree+1,Scalar(0));

	for(size_t i=0; i<v1.size(); i++) {
		for(size_t j=0; j<v2.size(); j++) {
			//cout << "adding (" <<v1[i]<<"*"<<v2[j]<<"="<<(v1[i]*v2[j])<<") to r[" << (i+j) <<"] = " << ret[i+j] <<endl;
			ret[i+j] += v1[i]*v2[j];
			//cout << "r["<<i+j<<"]="<<ret[i+j] << endl;
		}
	}

	/*
	cout << "result:" << endl;
	for(size_t i=0; i<ret.size(); i++) {
		cout << "r["<<i<<"]=" << ret[i] <<endl;
	}
	//*/

	return ret;
}


template < typename Scalar >
vector<Scalar> pow(const vector<Scalar>& coeff, unsigned int power) {
	if( power == 0 ) { // hehe
		return vector<Scalar>(1,Scalar(1));
	}
	if( power == 1 ) {
		return coeff;
	}

	vector<Scalar> ret( coeff );
	ret.reserve( degree(coeff)*power + 1);

	vector<Scalar> tmp;
	tmp.reserve( degree(coeff)*power + 1);

	unsigned int count=1;

	while( count < power ) {
		tmp.resize( degree(coeff) + degree(ret) + 1, Scalar(0) );

		for(size_t i=0; i<coeff.size(); i++) {
			for(size_t j=0; j<ret.size(); j++) {
				tmp[i+j] += coeff[i]*ret[j];
			}
		}

		ret = tmp;
		tmp.clear();

		count++;
	}

	return ret;
}



}

#endif // JIR_REFRACTIVE_IMAGE_GEOMETRY__POLYNOMIAL_HPP
