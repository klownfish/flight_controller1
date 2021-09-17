#include "BasicLinearAlgebra/BasicLinearAlgebra.h"


class Kalman2 {
    BLA::Matrix<2, 1> state;
    BLA::Matrix<2, 2> process_covariance;
    BLA::Matrix<2, 2> measurement_err;
    BLA::Matrix<2, 2> process_noise; 
    float second_derivative = 0;

public:
    Kalman2(float measure_err_value, float measure_err_deriv, float process_noise_value, float process_noise_deriv, 
            float x = 0, float Dx = 0,  float expected_change = 0, float process_err_value = 1, float process_err_deriv = 1) {
        state(0, 0) = x;
        state(1, 0) = Dx;
        measurement_err(0, 0) = measure_err_value;
        measurement_err(1, 1) = measure_err_deriv;
        process_noise(0, 0) = process_noise_value;
        process_noise(1, 1) = process_noise_deriv;
        process_covariance(0, 0) = process_err_value;
        process_covariance(1, 1) = process_err_deriv;
        second_derivative = expected_change;
    }

    float get_value() {return state(0, 0);}
    float get_derivative() {return state(1, 0);}

    void set_process_noise(float process_noise_value, float process_noise_deriv) {
        process_noise(0, 0) = process_noise_value;
        process_noise(1, 1) = process_noise_deriv;
    }

    void set_measurement_error(float measure_err_value, float measure_err_deriv) {
        measurement_err(0, 0) = measure_err_value;
        measurement_err(1, 1) = measure_err_deriv;        
    }

    void set_second_derivative(float a) {
        second_derivative = a;
    }

    void set_state(float val, float deriv) {
        state(0, 0) = val;
        state(1, 0) = deriv;
    }

    void get_process_error(float* value, float* deriv) {
        *value = process_covariance(0, 0);
        *deriv = process_covariance(1, 0);
    }

    void insert(float value, float deriv, float dt) {
        BLA::Matrix<2,1> measurement = {value, deriv};

        BLA::Matrix<2, 2> process_update = {1, dt,
                                            0, 1};
        BLA::Matrix<2,2> process_update_T = ~process_update;
        BLA::Matrix<2, 1> process_change = {0.5 * dt * dt, 
                                            dt};             
        // predict
        BLA::Matrix<2, 1> prediction = process_update * state + process_change * second_derivative;

        //update error
        process_covariance = process_update * process_covariance * process_update_T + process_noise;

        //calculate gain
        BLA::Matrix<2,2> gain = process_covariance * (process_covariance + measurement_err).Inverse();

        //update state
        state = prediction + gain * (measurement - prediction);

        //update error
        process_covariance = (BLA::Matrix<2, 2> {1,0,0,1} - gain) * process_covariance;
    }
};

class Kalman1 {
    float state;
    float process_err;
    float process_noise;
    float measurement_err;
    float derivative;

public:
    Kalman1(float measure_err, float _process_noise, float value = 0, float process_error = 1) {
        state = value;
        measurement_err = measure_err;
        process_noise = _process_noise;
        process_err = process_error;
    }

    void insert(float value, float dt = 0) {
        //predict
        float prediction = state + dt * derivative;
        //iupdate process error
        process_err = process_err + process_noise;
        //calculate gain
        float gain = process_err / (process_err + measurement_err);
        //set new state
        state = prediction + gain * (value - prediction);
        //set new process error
        process_err = (1 - gain) * process_err + process_noise;
    }

    float get_value() {return state;}
    float get_process_error() {return process_err;}

    void set_derivative(float val) {derivative = val;}
    void set_value(float val) {state = val;}
    void set_process_noise(float val) {process_noise = val;}
    void set_measurement_err(float val) {measurement_err = val;}
};