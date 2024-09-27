#include "PID.h"

#include <math.h>

#include <iostream>

#include "calc.h"
#include "geometry.h"
#include "my-timer.h"
#include "vex.h"
using namespace std;

/**
 * 本文件定义PID控制器
 * 需完成以下内容并在注释中完整解释：
 * PID::update()
 * DirPID::update()
 * PosPID::update()
 * 
 * 样例使用方法：
 * PID pid;
 * pid.setTarget(100);
 * pid.setErrorTolerance(1);
 * pid.setCoefficient(1, 0.1, 0.1);
 * while(!pid.taragetArrived()){
 *      pid.update(curr_value);
 *      output = pid.getOutput();
 *      // do something with output
 * }
 * 
 */

PID::PID() : first_time(true), arrived(false), I_max(20), I_range(50), jump_time(50), D_tol(10) { my_timer.reset(); }

void PID::setFirstTime() { first_time = true; }

void PID::setCoefficient(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
void PID::setTarget(double _target) { target = _target; }
void PID::setIMax(double _IMax) { I_max = _IMax; }
void PID::setIRange(double _IRange) { I_range = _IRange; }
void PID::setErrorTolerance(double _errorTol) { error_tol = _errorTol; }
void PID::setDTolerance(double _DTol) { D_tol = _DTol; }
void PID::setJumpTime(double _jumpTime) { jump_time = _jumpTime; }
void PID::setArrived(bool _arrived) { arrived = _arrived; }
bool PID::targetArrived() { return arrived; }
double PID::getOutput() { return output; }

void PID::update(double input) {
    error_curt = target - input;  // calculate current error
    
    if (first_time) { // refresh error_prev and error_int for the first time
        first_time = false;
        error_prev = error_curt;
        error_int = 0;
    }

    P = kp * error_curt; // calculate the contribution of P (proportional unit)

    if (fabs(P) >= I_range) { // I only starts to increase when P < I_range
        error_int = 0;
    } else {
        error_int += error_curt; // do integration
        if (ki * fabs(error_int) > fabs(I_max)) {
            error_int = sign(error_int) * I_max / ki; // avoid abs(I) > abs(I_max)
        }
    }
    I = ki * error_int; // calculate the contribution of I (integral unit)

    error_dev = error_curt - error_prev; // calculate error_dev
    error_prev = error_curt; // refresh error_prev for the next calculation of error_dev
    D = kd * error_dev; // calculate the contribution of D (derivative unit)
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void PosPID::setTarget(Point p) { target_point = p; }

void PosPID::update(Point input) {
    Vector err = target_point - input;
    error_curt = err.mod();  // calculate current error
    
    if (first_time) { // refresh error_prev and error_int for the first time
        first_time = false;
        error_prev = error_curt;
        error_int = 0;
    }

    P = kp * error_curt; // calculate the contribution of P (proportional unit)

    if (fabs(P) >= I_range) { // I only starts to increase when P < I_range
        error_int = 0;
    } else {
        error_int += error_curt; // do integration
        if (ki * fabs(error_int) > fabs(I_max)) {
            error_int = sign(error_int) * I_max / ki; // avoid abs(I) > abs(I_max)
        }
    }
    I = ki * error_int; // calculate the contribution of I (integral unit)

    error_dev = error_curt - error_prev; // calculate error_dev
    error_prev = error_curt; // refresh error_prev for the next calculation of error_dev
    D = kd * error_dev; // calculate the contribution of D (derivative unit)
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void DirPID::update(double input) {
    error_curt = degNormalize(target - input);  // calculate current error

    if (first_time) { // refresh error_prev and error_int for the first time
        first_time = false;
        error_prev = error_curt;
        error_int = 0;
    }

    P = kp * error_curt; // calculate the contribution of P (proportional unit)

    if (fabs(P) >= I_range) { // I only starts to increase when P < I_range
        error_int = 0;
    } else {
        error_int += error_curt; // do integration
        if (ki * fabs(error_int) > fabs(I_max)) {
            error_int = sign(error_int) * I_max / ki; // avoid abs(I) > abs(I_max)
        }
    }
    I = ki * error_int; // calculate the contribution of I (integral unit)

    error_dev = error_curt - error_prev; // calculate error_dev
    error_prev = error_curt; // refresh error_prev for the next calculation of error_dev
    D = kd * error_dev; // calculate the contribution of D (derivative unit)
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}