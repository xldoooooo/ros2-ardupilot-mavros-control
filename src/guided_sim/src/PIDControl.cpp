#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <list>
#include <cmath>
#include "PIDControl.h"
#include <stdio.h>

//using namespace std;

// 定义滤波器的阶数
#define ORDER 2


class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Ki, double Kd );
        ~PIDImpl();
        double calculate( double currentpoint, double setpoint);
        void reinitiate();
        void setPID(double newkp, double newki, double newkd);
        void setLimit(double newmax, double newmin);
        double calculate_LPF( double currentpoint, double setpoint);
        double calculate_BUTTER( double currentpoint, double setpoint);

        void init_filter(double fs, double fc);
        double filter(double input, double *state);
        
        std::list<double> getPID();

    private:
        double _dt;             //采样时间
        double _max;            //返回值输出的最大值限制
        double _min;            //返回值输出的最小值限制
        double _Kp;             //设置的P值
        double _Kd;             //设置的d值
        double _Ki;             //设置的I值
        double _pre_error;      //上次的误差，用于D的计算
        double _integral;       //累计积分误差，用于I的计算
        int _count;

        // 定义滤波器的系数
        double b[ORDER + 1]; // 分子系数
        double a[ORDER + 1]; // 分母系数

        // 创建状态数组
        double state[ORDER + 2] = {0};


};



// 初始化滤波器系数
void PIDImpl::init_filter(double fs, double fc) {
    double nyq = fs / 2.0;
    double wn = fc / nyq;

    // 计算巴特沃斯滤波器的系数（这里是二阶）
    double sqrt2 = sqrt(2.0);
    b[0] = 1.0;
    b[1] = 2.0;
    b[2] = 1.0;
    a[0] = 1.0 + sqrt2 + 1.0;
    a[1] = -2.0 * (1.0 - 1.0);
    a[2] = 1.0 - sqrt2 + 1.0;

    // 归一化系数
    for (int i = 0; i < ORDER + 1; i++) {
        b[i] /= a[0];
        if (i > 0)
            a[i] /= a[0];
    }
}

// 滤波函数
double PIDImpl::filter(double input, double *state) {
    double output;

    // 更新状态
    for (int i = ORDER; i > 0; i--) {
        state[i] = state[i - 1];
    }
    state[0] = input;

    // 计算输出
    output = b[0] * state[0] + b[1] * state[1] + b[2] * state[2]
             - a[1] * state[ORDER + 1] - a[2] * state[ORDER + 2];

    // 更新状态
    state[ORDER + 1] = output;
    state[ORDER + 2] = state[ORDER + 1];

    return output;
}


PID::PID( double dt, double max, double min, double Kp, double Ki, double Kd )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Ki,Kd);

}
void PID::reinitiate()
{
    pimpl->reinitiate();
}

void PID::setPID(double newkp, double newki, double newkd)
{
    pimpl->setPID(newkp, newki, newkd);
}
void PID::setLimit(double newmax, double newmin)
{
    pimpl->setLimit(newmax, newmin);
}

std::list<double> PID::getPID()
{
    return pimpl->getPID();
}


double PID::calculate( double currentpoint, double setpoint)
{
    return pimpl->calculate(currentpoint,setpoint);
}

double PID::calculate_LPF( double currentpoint, double setpoint)
{
    return pimpl->calculate_LPF(currentpoint,setpoint);
}

double PID::calculate_BUTTER( double currentpoint, double setpoint)
{
    return pimpl->calculate_BUTTER(currentpoint,setpoint);
}

PID::~PID() 
{
    delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Ki, double Kd ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Ki(Ki),
    _Kd(Kd),
    _pre_error(0),
    _integral(0),
    _count(0)
{
        // 初始化滤波器
    init_filter(100, 10);
}
void PIDImpl::reinitiate()
{
    _pre_error=0;
    _integral = 0;
}

void PIDImpl::setPID(double newkp, double newki, double newkd)
{
    _Kp = newkp;
    _Ki = newki;
    _Kd = newkd;
}

void PIDImpl::setLimit(double newmax, double newmin)
{
    _max = newmax;
    _min = newmin;
}

std::list<double> PIDImpl::getPID()
{
    return {_Kp, _Ki, _Kd};
} 

double PIDImpl::calculate( double currentpoint, double setpoint)
{
    // Calculate error
    double error = setpoint - currentpoint;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    if(_count == 0){
        _pre_error = error;
        _count++;
    }
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;
    //printf("PID： Get position [Pout= %f, Iout= %f, Dout=%f,  dt= %f, error=%f , _pre_error= %f, _integral=%f, Kp=%f , Ki= %f, Kd=%f,] \r\n", Pout, Iout, Dout, _dt,  error, _pre_error, _integral,_Kp,_Ki,_Kd);
    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}


#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
double PIDImpl::calculate_LPF( double currentpoint, double setpoint)
{
    static double Dout_fil = 0;

    ((Dout_fil) = Dout_fil + ( 1 / ( 1 + 1 / ( (10) *6.28f *(_dt) ) ) ) *( (currentpoint) - (Dout_fil) ));


    return Dout_fil;
}



double PIDImpl::calculate_BUTTER( double currentpoint, double setpoint)
{

    double Dout_fil;

    Dout_fil = filter(currentpoint, state);


    return Dout_fil;
}


PIDImpl::~PIDImpl()
{
}

#endif