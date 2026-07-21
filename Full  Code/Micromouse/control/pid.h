/**
 * @file pid.h
 * @brief Generic PID controller class.
 */

#ifndef PID_H
#define PID_H

/**
 * @brief PID controller state and parameters.
 */
class PID {
public:
    /**
     * @brief Constructor.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param out_min Minimum output value
     * @param out_max Maximum output value
     */
    PID(float kp, float ki, float kd, float out_min, float out_max);

    /**
     * @brief Compute the PID output.
     * @param setpoint Target value
     * @param measurement Current measured value
     * @param dt Time step in seconds
     * @return Control output
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief Reset the integral and derivative state.
     */
    void reset();

    /**
     * @brief Update PID gains dynamically.
     */
    void set_gains(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _out_min, _out_max;
    
    float _integral;
    float _prev_error;
    float _prev_measurement;
};

#endif /* PID_H */
