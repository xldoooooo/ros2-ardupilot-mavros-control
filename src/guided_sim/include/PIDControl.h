#ifndef _PID_H_
#define _PID_H_

#include <list>

#ifndef EXPORT_API

#define EXPORT_API __attribute__((visibility("default")))

#endif

class PIDImpl;  // forward declaration (pimpl)
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Ki, double Kd);
        void reinitiate();
        void setPID(double newkp, double newki, double newkd);
        void setLimit(double newmax, double newmin);
        std::list<double> getPID();
        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double currentpoint, double setpoint);
        double calculate_LPF( double currentpoint, double setpoint);
        double calculate_BUTTER( double currentpoint, double setpoint);
        
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif