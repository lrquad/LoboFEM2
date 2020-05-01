#pragma once
#include <iostream>
#include <type_traits>
#include <complex>
#include <cstring>
namespace lobo
{
	template<class TYPE, class Scalar> class LoboComplex;

	//template<> class LoboComplex<float>;
	//template<> class LoboComplex<double>;


#define LoboComplexType LoboComplex<TYPE,Scalar>
#define TemplateHead template<class TYPE,class Scalar>
#define LoboComplexS LoboComplex<double, double>
#define LoboComplexD LoboComplex<LoboComplexS, double>
#define LoboComplexT LoboComplex<LoboComplexD, double>


	TemplateHead
	TYPE sqaure_norm(const LoboComplexType& a);


	TemplateHead
	TYPE norm(const LoboComplexType& a);

	


	inline double log(const double&a)
	{
		return std::log(a);
	}

	inline float log(const float&a)
	{
		return std::log(a);
	}


	TemplateHead
	LoboComplexType log(const LoboComplexType&a);

	inline double exp(const double& a)
	{
		return std::exp(a);
	}

	inline float exp(const float& a)
	{
		return std::exp(a);
	}

	TemplateHead
	LoboComplexType exp(const LoboComplexType& a);

	inline double pow(const double& a, const double b)
	{
		return std::pow(a, b);
	}

	inline float pow(const float& a, const float b)
	{
		return std::pow(a, b);
	}

	TemplateHead
	LoboComplexType pow(const LoboComplexType& a, Scalar b);

	TemplateHead
		double powS(const LoboComplex<double, double>& a, double b);

	TemplateHead
		double normS(const LoboComplex<double, double>& a);

	TemplateHead
		LoboComplex<double, double> logS(const LoboComplex<double, double>&a);


	

	TemplateHead
	LoboComplexType polar(const TYPE& rho, const TYPE& theta);

	inline double cos(const double& a)
	{
		return std::cos(a);
	}

	inline float cos(const float& a)
	{
		return std::cos(a);
	}

	inline double sin(const double& a)
	{
		return std::sin(a);
	}

	inline float sin(const float& a)
	{
		return std::sin(a);
	}


	TemplateHead
	LoboComplexType cos(const LoboComplexType& a);

	TemplateHead
	LoboComplexType sin(const LoboComplexType& a);


	inline double cosh(const double&a)
	{
		return std::cosh(a);
	}

	inline float cosh(const float&a)
	{
		return std::cosh(a);
	}

	inline double sinh(const double&a)
	{
		return std::sinh(a);
	}

	inline float sinh(const float&a)
	{
		return std::sinh(a);
	}

	inline double tanh(const double&a)
	{
		return std::tanh(a);
	}

	inline float tanh(const float&a)
	{
		return std::tanh(a);
	}

	TemplateHead
	LoboComplexType cosh(const LoboComplexType& a);

	TemplateHead
	LoboComplexType sinh(const LoboComplexType& a);

	TemplateHead
		LoboComplexType tanh(const LoboComplexType& a);


	inline double atan2(const double&b, const double&a)
	{
		return std::atan2(b, a);
	}

	inline float atan2(const float&b, const float&a)
	{
		return std::atan2(b, a);
	}

	TemplateHead
	LoboComplexType atan2(const LoboComplexType&b, const LoboComplexType &a);


	template<class TYPE, class Scalar>
	inline LoboComplex<double, double> atanh(const LoboComplex<double, double>&a);

	template<class TYPE, class Scalar>
	inline LoboComplex<float, float> atanh(const LoboComplex<float, float>&a);

	TemplateHead
	LoboComplexType atanh(const LoboComplexType&a);


	inline double atan(const double& a)
	{
		return std::atan(a);
	}

	inline float atan(const float& a)
	{
		return std::atan(a);
	}

	TemplateHead
	LoboComplexType atan(const LoboComplexType&a);


	inline double sqrt(const double& a)
	{
		return std::sqrt(a);
	}

	inline float sqrt(const float& a)
	{
		return std::sqrt(a);
	}

	TemplateHead
	LoboComplexType sqrt(const LoboComplexType&a);

	inline bool largerReal(const double& a, const double& b)
	{
		return a > b;
	}

	inline bool largerReal(const float& a, const float& b)
	{
		return a > b;
	}

	TemplateHead
		bool largerReal(const LoboComplexType&a, const Scalar &b);

	TemplateHead
		bool largerReal(const LoboComplexType&a, const LoboComplexType &b);


	inline bool smallerReal(const double& a, const double& b)
	{
		return a < b;
	}

	inline bool smallerReal(const float& a, const float& b)
	{
		return a < b;
	}

	inline void setALLZero(double& a)
	{
		a = 0.0;
	}

	inline void setALLZero(float& a)
	{
		a = 0.0;
	}

	TemplateHead
		void setALLZero(LoboComplexType&a);


	TemplateHead
		bool smallerReal(const LoboComplexType&a, const Scalar &b);

	TemplateHead
		bool smallerReal(const LoboComplexType&a, const LoboComplexType &b);



	inline double abs(const double& a)
	{
		return std::abs(a);
	}

	inline float abs(const float& a)
	{
		return std::abs(a);
	}

	TemplateHead
		LoboComplexType abs(const LoboComplexType&a);

	/*template<class TYPE, class COMPLEX, class Scalar>
	double sqrt(const double& a)
	{
		return std::sqrt(a);
	}*/

	//now for top level
	inline double multi(const double& a, const double& b)
	{
		return a*b;
	}

	inline float multi(const float& a, const float& b)
	{
		return a*b;
	}

	inline void multiScalr(const double& a, const double& b ,  double& c)
	{
		c = a*b;
	}

	inline void multiScalr(const float& a, const float& b, float& c)
	{
		c = a*b;
	}

	TemplateHead
		LoboComplexType multi(const LoboComplexType&a, const LoboComplexType&b);

	TemplateHead
		void multiScalr(const LoboComplexType&a, const Scalar&b, LoboComplexS&c);

	TemplateHead
		void multi(const LoboComplexS&a, const LoboComplexS&b, LoboComplexS&c);

	TemplateHead
		void multi(const LoboComplexD&a, const LoboComplexD&b, LoboComplexD&c);

	TemplateHead
		void multi(const LoboComplexT&a, const LoboComplexT&b, LoboComplexT&c);

	TemplateHead
		void divide(const LoboComplexS&a, const LoboComplexS&b, LoboComplexS&c);

	TemplateHead
		void divide(const LoboComplexD&a, const LoboComplexD&b, LoboComplexD&c);

	TemplateHead
		void divide(const LoboComplexT&a, const LoboComplexT&b, LoboComplexT&c);
	

	TemplateHead
		void add(const LoboComplexType&a, const LoboComplexType&b, LoboComplexType&c);

	TemplateHead
		void add_a(LoboComplexType&a, const LoboComplexType&b);
	
	inline void add_a( double& a, const double& b)
	{
		a += b;
	}
	
	inline void add(const double& a, const double& b, double&c)
	{
		c = a + b;
	}

	TemplateHead
		void powS(const LoboComplexS& a, double b, LoboComplexS&c);
	TemplateHead
		void powS(const LoboComplexD& a, double b, LoboComplexD&c)
	{

	}
	TemplateHead
		void powS(const LoboComplexT& a, double b, LoboComplexT&c)
	{

	}


	TemplateHead
	class LoboComplex
	{
	public:

		/*LoboComplex(const TYPE& real = TYPE(), const TYPE& image = TYPE()):real_(real),image_(image)
		{

		}*/
		LoboComplex(TYPE real, TYPE image);
		LoboComplex(TYPE real);
		LoboComplex();

		LoboComplex(const LoboComplexType& c);
		~LoboComplex();

		void setZero();




		LoboComplexType& operator+=(const Scalar& a)
		{
			real_ += a;
			return *this;
		}

		LoboComplexType& operator-=(const Scalar& a)
		{
			real_ -= a;
			return *this;
		}

		

		LoboComplexType& operator*=(const Scalar&);
		LoboComplexType& operator/=(const Scalar&);




		LoboComplexType& operator= (const LoboComplexType& rhs);
		LoboComplexType& operator= (const Scalar& rhs);

		
		LoboComplexType& operator+=(const LoboComplexType& rhs);
		LoboComplexType& operator-=(const LoboComplexType& rhs);

		LoboComplexType& operator*=(const LoboComplexType& rhs);

		LoboComplexType& operator/=(const LoboComplexType& rhs);

		/*friend LoboComplexType operator+(const LoboComplexType& lhs, const LoboComplexType& rhs);*/


		TYPE real_;
		TYPE image_;

	};

	TemplateHead
	void lobo::LoboComplexType::setZero()
	{
		lobo::setALLZero(*this);
	}
	
	TemplateHead
		void add(const LoboComplexType&a, const LoboComplexType&b, LoboComplexType&c)
	{
		//c.real_ = a.real_ + b.real_;
		add(a.real_, b.real_, c.real_);
		add(a.image_, b.image_, c.image_);
	}

	TemplateHead
		void add_a(LoboComplexType&a, const LoboComplexType&b)
	{
		//a.real_ += b.real_;
		//a.image_ += b.image_;

		add_a(a.real_, b.real_);
		add_a(a.image_, b.image_);
	}

	TemplateHead
		LoboComplexType multi(const LoboComplexType&a, const LoboComplexType&b)
	{
		LoboComplexType r;
		r.real_ = lobo::multi(a.real_, b.real_);
		r.image_ = lobo::multi(a.real_, b.image_) + lobo::multi(a.image_, b.real_);
		return r;
	}

	TemplateHead
	void multi(const double& a, const double& b, double&c)
	{
		c = a*b;
	}

	TemplateHead
		void multi(const LoboComplexS&a, const LoboComplexS&b, LoboComplexS&c)
	{
		c.real_ = a.real_*b.real_;
		c.image_ = a.real_*b.image_ + a.image_*b.real_;
	}

	TemplateHead
		void multi(const LoboComplexD&a, const LoboComplexD&b, LoboComplexD&c)
	{
		c.real_.real_ = a.real_.real_*b.real_.real_;
		c.real_.image_ = a.real_.image_*b.real_.real_ + a.real_.real_ * b.real_.image_;
		c.image_.real_ = a.image_.real_*b.real_.real_ + a.real_.real_ * b.image_.real_;
		c.image_.image_ = a.image_.image_*b.real_.real_ + a.real_.image_*b.image_.real_ + a.image_.real_*b.real_.image_ + a.real_.real_*b.image_.image_;
	}

	

	TemplateHead
		void multi(const LoboComplexT&a, const LoboComplexT&b, LoboComplexT&c)
	{

	}

	TemplateHead
		void divide(const LoboComplexS&a, const LoboComplexS&b, LoboComplexS&c)
	{
		const double r = a.real_*b.real_;
		const double n = b.real_*b.real_;
		c.image_ = (a.image_*b.real_ - a.real_*b.image_) / n;
		c.real_ = r / n;
	}

	TemplateHead
		void divide(const LoboComplexD&a, const LoboComplexD&b, LoboComplexD&c)
	{

	}

	TemplateHead
		void divide(const LoboComplexT&a, const LoboComplexT&b, LoboComplexT&c)
	{

	}

	TemplateHead
		void multiScalr(const LoboComplexType&a, const Scalar&b, LoboComplexS&c)
	{
		multiScalr(a.real_, b, c.real_);
		multiScalr(a.image_, b, c.image_);
	}

	TemplateHead
		void powS(const LoboComplex<double, double>& a, double b, LoboComplex<double, double>&c)
	{
		c.real_ = std::pow(a.real_, b);
		c.image_ = c.real_ / (a.real_ / b)*a.image_;
	}

	TemplateHead
		double powS(const LoboComplex<double, double>& a, double b)
	{
		/*LoboComplexType r = logS<double,double>(a);

		double rho = exp(b*r.real_);
		double theta = b*r.image_;*/
		double p = pow(a.real_, b);
		double image_ = p / (a.real_ / b)*a.image_;
		return image_;
	}


	TemplateHead
		inline bool operator==(const LoboComplexType& lhs, const Scalar& rhs)
	{
		return (lhs.image_ == rhs) && (lhs.real_ == rhs);
	}

	TemplateHead
		inline bool operator<(const LoboComplexType& lhs, const Scalar& rhs)
	{
		return (lhs.image_ < rhs) && (lhs.real_ < rhs);
	}

	TemplateHead
		inline bool operator>(const LoboComplexType& lhs, const Scalar& rhs)
	{
		return (lhs.image_ > rhs) && (lhs.real_ > rhs);
	}

	TemplateHead
	inline LoboComplexType operator+(const LoboComplexType& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = lhs;
		r += rhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator+(const LoboComplexType& lhs, const Scalar& rhs)
	{
		LoboComplexType r = lhs;
		r += rhs;
		return r;
	}

	/*template<class TYPE, class COMPLEX>
	inline LoboComplexType operator+(const TYPE& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = rhs;
		r += lhs;
		return r;
	}*/

	TemplateHead
	inline LoboComplexType operator+(const Scalar& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r;
		r = lhs;
		return r+rhs;
	}

	TemplateHead
	inline LoboComplexType operator-(const LoboComplexType& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = lhs;
		r -= rhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator-(const LoboComplexType& lhs, const Scalar& rhs)
	{
		LoboComplexType r = lhs;
		r -= rhs;
		return r;
	}

	/*template<class TYPE, class COMPLEX>
	inline LoboComplexType operator-(const TYPE& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r(lhs, -rhs.image_);
		r -= rhs.real_;
		return r;
	}*/

	TemplateHead
	inline LoboComplexType operator-(const Scalar& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = lhs;

		return r-rhs;
	}


	TemplateHead
	inline LoboComplexType operator*(const LoboComplexType& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = lhs;
		r *= rhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator*(const LoboComplexType& lhs, const Scalar& rhs)
	{
		LoboComplexType r = lhs;
		r *= rhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator*(const Scalar& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = rhs;
		r *= lhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator/(const LoboComplexType& lhs, const LoboComplexType& rhs)
	{
		LoboComplexType r = lhs;
		r /= rhs;
		return r;
	}

	/*template<class TYPE, class COMPLEX>
	inline LoboComplexType operator/(const LoboComplexType& lhs, const TYPE& rhs)
	{
		LoboComplexType r = lhs;
		r /= rhs;
		return r;
	}*/

	TemplateHead
	inline LoboComplexType operator/(const LoboComplexType& lhs, const Scalar& rhs)
	{
		LoboComplexType r = lhs;
		r /= rhs;
		return r;
	}

	TemplateHead
	inline LoboComplexType operator/(const Scalar& lhs, const LoboComplexType& rhs)
	{
		//To be continue;
		LoboComplexType result;
		result = lhs;
		result /= rhs;
		return result;
	}

	/*TemplateHead
		inline bool operator>(const LoboComplexType& lhs, const Scalar& rhs)
	{
		return lhs.real_ > rhs;
	}*/

	TemplateHead
	inline LoboComplexType operator+(const LoboComplexType& x)
	{
		return x;
	}

	TemplateHead
	inline LoboComplexType operator-(const LoboComplexType& x)
	{
		return LoboComplexType(-x.real_,-x.image_);
	}
	

	TemplateHead
	LoboComplexType& lobo::LoboComplexType::operator/=(const LoboComplexType& rhs)
	{
		const TYPE r = real_*rhs.real_ + image_*rhs.image_;
		const TYPE n = sqaure_norm(rhs);
		image_ = (image_*rhs.real_ - real_*rhs.image_) / n;
		real_ = r / n;
		return *this;
	}

	TemplateHead
	LoboComplexType& lobo::LoboComplexType::operator*=(const LoboComplexType& rhs)
	{
		const TYPE r = real_*rhs.real_ - image_*rhs.image_;
		image_ = image_*rhs.real_ + real_*rhs.image_;
		real_ = r;
		return *this;
	}

	

	TemplateHead
	LoboComplexType& lobo::LoboComplexType::operator-=(const LoboComplexType& rhs)
	{
		real_ -= rhs.real_;
		image_ -= rhs.image_;
		return *this;
	}

	TemplateHead
	LoboComplexType& lobo::LoboComplexType::operator+=(const LoboComplexType& rhs)
	{
		real_ += rhs.real_;
		image_ += rhs.image_;
		return *this;
	}

	TemplateHead
	lobo::LoboComplexType& lobo::LoboComplexType::operator=(const LoboComplexType& rhs)
	{
		real_ = rhs.real_;
		image_ = rhs.image_;
		return *this;
	}

	TemplateHead
	lobo::LoboComplexType& lobo::LoboComplexType::operator=(const Scalar& rhs)
	{
		real_ = rhs;
		return *this;
	}

	

	TemplateHead
	lobo::LoboComplexType& lobo::LoboComplexType::operator*=(const Scalar& t)
	{
		real_ *= t;
		image_ *= t;
		return *this;
	}

	TemplateHead
	lobo::LoboComplexType& lobo::LoboComplexType::operator /= (const Scalar& t)
	{
		real_ /= t;
		image_ /= t;

		return *this;
	}

	///  Insertion operator for complex values.
	TemplateHead
	std::ostream& operator<<(std::ostream& os, const LoboComplexType& __x)
	{
		os << '(' << __x.real_ << ',' << __x.image_ << ')';
		return os;
	}

	TemplateHead
	LoboComplexType::~LoboComplex()
	{

	}

	TemplateHead
		LoboComplexType::LoboComplex(const LoboComplexType& c) :real_(c.real_), image_(c.image_)
	{
		//real_ = c.real_;
		//image_ = c.image_;
	}

	TemplateHead
	LoboComplexType::LoboComplex(TYPE real /*= 0.0*/, TYPE image /*= 0.0*/)
	{
		real_ = real;
		image_ = image;
	}


	TemplateHead
	LoboComplexType::LoboComplex()
	{
		//real_ = 0.0;
		//image_ = 0.0;
		memset(this, 0, sizeof(*this));
	}

	TemplateHead
	LoboComplexType::LoboComplex(TYPE real)
	{
		real_ = real;
		image_ = 0.0;
	}



	TemplateHead
	TYPE sqaure_norm(const LoboComplexType& a)
	{
		TYPE r = a.real_*a.real_ + a.image_*a.image_;
		return r;
	}

	

	TemplateHead
	TYPE norm(const LoboComplexType& a)
	{
		TYPE r = a.real_*a.real_ + a.image_*a.image_;
		
		//return std::sqrt(r);
		return sqrt(r);
	}
	

	TemplateHead
	LoboComplexType log(const LoboComplexType&a)
	{
		LoboComplexType result;

		TYPE r = norm(a);

		result.image_ = atan2(a.image_,a.real_);
		
		result.real_ = log(r);

		return result;
	}
	
	TemplateHead
	LoboComplexType exp(const LoboComplexType& a)
	{
		LoboComplexType r;

		r.real_ = exp(a.real_)*cos(a.image_);
		r.image_ = exp(a.real_)*sin(a.image_);

		return r;
	}


	TemplateHead
	LoboComplexType pow(const LoboComplexType& a, Scalar b)
	{
		LoboComplexType r = log(a);
		r = polar<TYPE,Scalar>(exp(b*r.real_), b*r.image_);
		return r;
	}

	

	TemplateHead
		double normS(const LoboComplex<double, double>& a)
	{
		return a.real_;
	}

	TemplateHead
		LoboComplex<double, double> logS(const LoboComplex<double, double>&a)
	{
		LoboComplexType result;

		double r = normS<double,double>(a);

		result.image_ = atan2(a.image_, a.real_);

		result.real_ = log(r);

		return result;
	}

	

	TemplateHead
	LoboComplexType polar(const TYPE& rho, const TYPE& theta)
	{
		LoboComplexType r;
		r.real_ = rho*cos(theta);
		r.image_ = rho*sin(theta);
		return r;
	}

	TemplateHead
	LoboComplexType cos(const LoboComplexType& a)
	{
		LoboComplexType r;
		r.real_ = cosh(-a.image_)*cos(a.real_);
		r.image_ = sinh(-a.image_)*sin(a.real_);

		return r;
	}

	TemplateHead
	LoboComplexType sin(const LoboComplexType& a)
	{
		LoboComplexType r;
		r.real_ = cosh(-a.image_)*sin(a.real_);
		r.image_ = -sinh(-a.image_)*cos(a.real_);
		return r;
	}

	TemplateHead
		LoboComplexType cosh(const LoboComplexType& a)
	{
		LoboComplexType r;

		r.real_ = cosh(a.real_)*cos(a.image_);
		r.image_ = sinh(a.real_)*sin(a.image_);
		
		return r;
	}

	TemplateHead
		LoboComplexType sinh(const LoboComplexType& a)
	{
		LoboComplexType r;
		r.real_ = sinh(a.real_)*cos(a.image_);
		r.image_ = cosh(a.real_)*sin(a.image_);
		return r;
	}

	TemplateHead
		LoboComplexType tanh(const LoboComplexType& a)
	{
		LoboComplexType r;
		r = sinh(a) / cosh(a);
		return r;
	}

	TemplateHead
	LoboComplexType atan2(const LoboComplexType&b, const LoboComplexType &a)
	{
		LoboComplexType r = a*a + b*b;

		r = sqrt(r);
		
		if (largerReal(a, (Scalar)0.0))
		{
			r += a;
			r = b / r;
		}
		else
		{
			r -= a;
			r = r / b;
		}
		
		r = atan(r);

		r *= 2.0;

		return r;
	}

	TemplateHead
	LoboComplexType atan(const LoboComplexType&a)
	{
		LoboComplexType r(-a.image_, a.real_);
		r = atanh(r);
		
		return LoboComplexType(r.image_,-r.real_);
	}


	TemplateHead
	LoboComplexType atanh(const LoboComplexType&a)
	{
		LoboComplexType i1;
		i1.setZero();
		i1.real_ = 1;
		return (log(i1 + a) - log(i1 - a))*(Scalar)0.5;
	}

	template<>
	inline LoboComplex<double, double> atanh(const LoboComplex<double, double>&a)
	{
		std::complex<double> c(a.real_, a.image_);
		c = std::atanh(c);
		return LoboComplex<double, double>(c.real(), c.imag());
	}

	template<>
		inline LoboComplex<float, float> atanh(const LoboComplex<float, float>&a)
	{
		std::complex<float> c(a.real_, a.image_);
		c = std::atanh(c);
		return LoboComplex<float, float>(c.real(), c.imag());
	}


	TemplateHead
	LoboComplexType sqrt(const LoboComplexType&a)
	{
		LoboComplexType result = a;
		TYPE r = norm(a); //|z|
		TYPE sqrt_r;
		sqrt_r = sqrt(r);
		//sqrt_r = sqrt<TYPE,Scalar>(r);
		
		result.real_ += r; //z+r
		TYPE zrnorm = norm(result); //|z+r|
		//std::cout <<"zrnom "<< zrnorm << std::endl;

		if (zrnorm < (Scalar)1e-200 && zrnorm > (Scalar)-1e-200)
		{
			result *= sqrt_r;
			return result;
		}

		TYPE scale = sqrt_r / zrnorm;

		result *= scale;
		
		return result;
	}

	TemplateHead
		bool largerReal(const LoboComplexType&a, const Scalar &b)
	{
		return largerReal(a.real_, b);
	}

	TemplateHead
		bool largerReal(const LoboComplexType&a, const LoboComplexType &b)
	{
		return largerReal(a.real_, b.real_);
	}


	TemplateHead
		bool smallerReal(const LoboComplexType&a, const Scalar &b)
	{
		return smallerReal(a.real_, b);

	}

	TemplateHead
		bool smallerReal(const LoboComplexType&a, const LoboComplexType &b)
	{
		return smallerReal(a.real_, b.real_);
	}


	TemplateHead
		void setALLZero(LoboComplexType&a)
	{
		lobo::setALLZero(a.real_);
		lobo::setALLZero(a.image_);
	}


	TemplateHead
		LoboComplexType abs(const LoboComplexType&a)
	{
		LoboComplexType b = a;
		if (smallerReal(a, (Scalar)0.0))
		{
			b = a*(Scalar)(-1.0);
			return b;
		}
		else {
			return b;
		}
	}


//#define LOBO_MAKE_TYPEDEFS(Type,TypeSuffix)\
//typedef LoboComplex<Type, Type> LoboComplex##TypeSuffix;\
//typedef LoboComplex<Type, LoboComplex##TypeSuffix> LoboComplexDual##TypeSuffix;\
//typedef LoboComplex<Type, LoboComplexDual##TypeSuffix> LoboComplexTri##TypeSuffix;
//
//	LOBO_MAKE_TYPEDEFS(float, f);
//	LOBO_MAKE_TYPEDEFS(double, d);

//#undef LOBO_MAKE_TYPEDEFS

}

#define LOBO_MAKE_TYPEDEFS(Type,TypeSuffix)\
typedef lobo::LoboComplex<Type,Type> LoboComplex##TypeSuffix;\
typedef lobo::LoboComplex<LoboComplex##TypeSuffix,Type> LoboComplexDual##TypeSuffix;\
typedef lobo::LoboComplex<LoboComplexDual##TypeSuffix,Type> LoboComplexTri##TypeSuffix;
