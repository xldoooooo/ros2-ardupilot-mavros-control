#include "ButterWorthFIlter.hpp"
#include "math.h"


ButterWorthFIlter::ButterWorthFIlter()
{
}

ButterWorthFIlter::ButterWorthFIlter(float sample_freq, float cutoff_freq){
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
        fil_count = 0;

}

ButterWorthFIlter::~ButterWorthFIlter()
{
}



// Change filter parameters
void ButterWorthFIlter::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
    // if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2)) {

    //     disable();
    //     return;
    // }

    // reset delay elements on filter change
    _delay_element_1 = 0.0f;
    _delay_element_2 = 0.0f;

    _cutoff_freq =  cutoff_freq; //math::max(cutoff_freq, sample_freq * 0.001f);
    _sample_freq = sample_freq;

    const float fr = _sample_freq / _cutoff_freq;
    const float ohm = tan(M_PI / fr);
    const float c = 1.f + 2.f * cos(M_PI / 4.f) * ohm + ohm * ohm;

    _b0 = ohm * ohm / c;
    _b1 = 2.f * _b0;
    _b2 = _b0;

    _a1 = 2.f * (ohm * ohm - 1.f) / c;
    _a2 = (1.f - 2.f * cos(M_PI / 4.f) * ohm + ohm * ohm) / c;

}


/**
 * Add a new raw value to the filter
 *
 * @return retrieve the filtered result
 */
double ButterWorthFIlter::filter(const double &sample)
{
    // Direct Form II implementation
    double delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};

    const double output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    if(fil_count < 5){
        fil_count++;
        return sample;
    }

    return output;
}

// Reset the filter state to this value
double ButterWorthFIlter::reset(const double &sample)
{
    const double input =  sample ;

    if (fabsf(1 + _a1 + _a2) > __FLT_EPSILON__) {
        _delay_element_1 = _delay_element_2 = input / (1 + _a1 + _a2);

    } else {
        _delay_element_1 = _delay_element_2 = input;
    }

    return filter(input);
}

void ButterWorthFIlter::disable()
{
    // no filtering
    _sample_freq = 0.f;
    _cutoff_freq = 0.f;

    _delay_element_1 = 0.0f;
    _delay_element_2 = 0.0f;

    _b0 = 1.f;
    _b1 = 0.f;
    _b2 = 0.f;

    _a1 = 0.f;
    _a2 = 0.f;
}


