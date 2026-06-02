
#ifndef BUTTERWORTHFILTER_H_
#define BUTTERWORTHFILTER_H_

// 定义滤波器的阶数
#define ORDER 2

class ButterWorthFIlter
{
private:

	double _delay_element_1{0.0f}; // buffered sample -1
	double _delay_element_2{0.0f}; // buffered sample -2

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	float _cutoff_freq{0.f};
	float _sample_freq{0.f};


public:

    ButterWorthFIlter() ;

	ButterWorthFIlter(float sample_freq, float cutoff_freq);

     ~ButterWorthFIlter();

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    double filter(const double &sample);


	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Return the sample frequency
	float get_sample_freq() const { return _sample_freq; }



	// Reset the filter state to this value
	double reset(const double &sample);

	void disable();

	int fil_count;




};



#endif

