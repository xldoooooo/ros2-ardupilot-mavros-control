#include <cmath>
#include "PIDControl.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd)
    : _dt(dt), _max(max), _min(min), _Kp(Kp), _Ki(Ki), _Kd(Kd)
    , _pre_error(0), _integral(0), _count(0)
{
    init_filter(100, 10);
}

// ---------------------------------------------------------------------------
// Filter helpers
// ---------------------------------------------------------------------------
void PID::init_filter(double fs, double fc) {
    double nyq = fs / 2.0;
    double wn  = fc / nyq;
    double sqrt2 = std::sqrt(2.0);

    b[0] = 1.0;   b[1] = 2.0;   b[2] = 1.0;
    a[0] = 1.0 + sqrt2 + 1.0;
    a[1] = -2.0 * (1.0 - 1.0);
    a[2] = 1.0 - sqrt2 + 1.0;

    for (int i = 0; i < ORDER + 1; i++) {
        b[i] /= a[0];
        if (i > 0) a[i] /= a[0];
    }
}

double PID::filter(double input) {
    for (int i = ORDER; i > 0; i--)
        state[i] = state[i - 1];
    state[0] = input;

    double output = b[0] * state[0] + b[1] * state[1] + b[2] * state[2]
                  - a[1] * state[ORDER + 1] - a[2] * state[ORDER + 2];

    state[ORDER + 1] = output;
    state[ORDER + 2] = state[ORDER + 1];
    return output;
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------
void PID::reinitiate() {
    _pre_error = 0;
    _integral  = 0;
}

void PID::setPID(double newkp, double newki, double newkd) {
    _Kp = newkp;
    _Ki = newki;
    _Kd = newkd;
}

void PID::setLimit(double newmax, double newmin) {
    _max = newmax;
    _min = newmin;
}

void PID::setDt(double dt) {
    _dt = dt;
}

std::list<double> PID::getPID() {
    return {_Kp, _Ki, _Kd};
}

// ---------------------------------------------------------------------------
// Core calculations
// ---------------------------------------------------------------------------
double PID::calculate(double currentpoint, double setpoint) {
    double error = setpoint - currentpoint;

    // Proportional
    double Pout = _Kp * error;

    // Integral
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative (skip on first call to avoid dt-spike)
    if (_count == 0) {
        _pre_error = error;
        _count++;
    }
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    if (output > _max)      output = _max;
    else if (output < _min) output = _min;

    _pre_error = error;
    return output;
}

double PID::calculate_LPF(double currentpoint, double setpoint) {
    static double Dout_fil = 0;
    Dout_fil += (1.0 / (1.0 + 1.0 / (10.0 * 6.28 * _dt))) * (currentpoint - Dout_fil);
    return Dout_fil;
}

double PID::calculate_BUTTER(double currentpoint, double setpoint) {
    return filter(currentpoint);
}
