#ifndef _PID_H_
#define _PID_H_

#include <list>

class PID {
public:
    // Kp - proportional gain
    // Ki - Integral gain
    // Kd - derivative gain
    // dt - loop interval time (s)
    // max - maximum output clamp
    // min - minimum output clamp
    PID(double dt, double max, double min, double Kp, double Ki, double Kd);
    ~PID() = default;

    void reinitiate();
    void setPID(double newkp, double newki, double newkd);
    void setLimit(double newmax, double newmin);
    void setDt(double dt);
    std::list<double> getPID();

    double calculate(double currentpoint, double setpoint);
    double calculate_LPF(double currentpoint, double setpoint);
    double calculate_BUTTER(double currentpoint, double setpoint);

private:
    static constexpr int ORDER = 2;

    void init_filter(double fs, double fc);
    double filter(double input);

    double _dt;
    double _max, _min;
    double _Kp, _Ki, _Kd;
    double _pre_error;
    double _integral;
    int _count;

    double b[ORDER + 1];
    double a[ORDER + 1];
    double state[ORDER + 2] = {};
};

#endif
