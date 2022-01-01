#ifndef FUCKTARD
#define FUCKTARD

#include <stdint.h>
#include <math.h>
#include "turbomath.h"
//#include "KlimaD3.h"
//#include "rocket_defines.h"
#define AMOUNT_OF_ENGINES 2

#define GRAVITY 9.824

#define GYRO_BIAS_MEASURE_NOISE 0.1
#define GYRO_BIAS_PROCESS_NOISE 0.001

#define ACCEL_MEASURE_NOISE_VERTICAL 0.1
#define ACCEL_PROCESS_NOISE_VERTICAL 0.5

#define ACCEL_MEASURE_NOISE_HORIZONTAL 0.1
#define ACCEL_PROCESS_NOISE_HORIZONTAL 0.5

#define GYRO_MEASURE_NOISE 0.1
#define GYRO_PROCESS_NOISE 0.5

#define GRAVITY_MEASURE_NOISE 0.1
#define GRAVITY_PROCESS_NOISE 0.5

#define ALTITUDE_MEASURE_NOISE 0.1
#define ALTITUDE_PROCESS_NOISE 0.5

class ComplimentaryFilter {
    float value;
    float gain;
public:
    ComplimentaryFilter(float value_gain) {
        gain = value_gain;
    }
    void insert_value(float new_value) {
        value = (1 - value) * gain + new_value * gain;
    }
    void insert_derivative(float deriv, float dt) {
        value += deriv * dt;
    }
    float get_value() {
        return value;
    }

    void force_set(float new_value) {
        value = new_value;
    }

};

class Kalman {
    float state;
    float process_err;
    float process_noise;
    float measurement_err;
    float derivative;

public:
    Kalman(){};
    Kalman(float measure_err, float _process_noise, float value = 0, float process_error = 1) {
        state = value;
        measurement_err = measure_err;
        process_noise = _process_noise;
        process_err = process_error;
    }

    void insert(float value, float dt = 0) {
        //predict
        float prediction = state + dt * derivative;
        //update process error
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

class KalmanVector {
    Kalman x;
    Kalman y;
    Kalman z;

public:
    KalmanVector(turbomath::Vector measure_err, turbomath::Vector process_noise) {
        x = Kalman(measure_err.x, process_noise.x);
        y = Kalman(measure_err.y, process_noise.y);
        z = Kalman(measure_err.z, process_noise.z);
    }


    turbomath::Vector get_value() {
        return turbomath::Vector{x.get_value(), y.get_value(), z.get_value()};
    }

    turbomath::Vector get_process_error() {
        return turbomath::Vector{x.get_process_error(), y.get_process_error(), z.get_process_error()};
    }

    void insert(turbomath::Vector val, float dt = 0) {
        x.insert(val.x, dt);
        y.insert(val.y, dt);
        z.insert(val.z, dt);
    }

    void set_derivative(turbomath::Vector val) {
        x.set_derivative(val.x);
        y.set_derivative(val.y);
        z.set_derivative(val.z);
    }
};

class KalmanQuaternion {
    Kalman w;
    Kalman x;
    Kalman y;
    Kalman z;

public:
    KalmanQuaternion(turbomath::Quaternion measure_err, turbomath::Quaternion process_noise) {
        w = Kalman(measure_err.w, process_noise.w);
        x = Kalman(measure_err.x, process_noise.x);
        y = Kalman(measure_err.y, process_noise.y);
        z = Kalman(measure_err.z, process_noise.z);
    }

    turbomath::Quaternion get_value() {
        return turbomath::Quaternion{w.get_value(), x.get_value(), y.get_value(), z.get_value()};
    }

    turbomath::Quaternion get_process_error() {
        return turbomath::Quaternion{w.get_process_error(), x.get_process_error(), y.get_process_error(), z.get_process_error()};
    }

    void insert(turbomath::Quaternion val, float dt = 0) {
        w.insert(val.w, dt);
        x.insert(val.x, dt);
        y.insert(val.y, dt);
        z.insert(val.z, dt);
    }

    void set_derivative(turbomath::Quaternion val) {
        w.set_derivative(val.w);
        x.set_derivative(val.x);
        y.set_derivative(val.y);
        z.set_derivative(val.z);
    }
};

class Estimator {
public:
    void update(uint32_t time);
    void insert_acceleration(float ax, float ay, float az, uint32_t time);
    void insert_gyro(float gx, float gy, float gz, uint32_t time);
    void insert_pressure(float pres, uint32_t time);

    void set_stationary(float roll, float pitch, float yaw);
    void set_moving();
    void ignition_started();

    float get_thrust();
    bool has_launched();

    uint8_t get_calibration(uint8_t* buf);
    void set_calibration(uint8_t* buf, uint8_t len);
    turbomath::Quaternion get_heading();
    turbomath::Vector get_local_rotation();
    turbomath::Vector get_local_acceleration();
    float get_z_velocity();
    float get_altitude();
    bool apogee_reached();

private:
    bool stationary = false;
    bool ignited = false;
    
    uint32_t last_acceleration_update = 0;
    uint32_t last_pressure_update = 0;
    uint32_t last_gyro_update = 0;
    uint32_t last_main_update = 0;
    uint32_t engine_started = 0;

    turbomath::Quaternion imu_offset;

    turbomath::Vector gravity_vector {0, 0, GRAVITY};
    turbomath::Quaternion stationary_heading {0, 0, 0};

    turbomath::Quaternion heading {0, 0, 0};

    Kalman altitude {ALTITUDE_MEASURE_NOISE, ALTITUDE_PROCESS_NOISE};

    KalmanVector measured_gravity_vector{{
        GRAVITY_MEASURE_NOISE,
        GRAVITY_MEASURE_NOISE,
        GRAVITY_MEASURE_NOISE
    },{
        GRAVITY_PROCESS_NOISE,
        GRAVITY_PROCESS_NOISE,
        GRAVITY_PROCESS_NOISE
    }};

    KalmanVector local_accel {{
        ACCEL_MEASURE_NOISE_HORIZONTAL,
        ACCEL_MEASURE_NOISE_HORIZONTAL,
        ACCEL_MEASURE_NOISE_VERTICAL
    },{
        ACCEL_PROCESS_NOISE_HORIZONTAL,
        ACCEL_PROCESS_NOISE_HORIZONTAL,
        ACCEL_PROCESS_NOISE_VERTICAL
    }};

    KalmanVector gyro_bias {{
        GYRO_BIAS_MEASURE_NOISE,
        GYRO_BIAS_MEASURE_NOISE, 
        GYRO_BIAS_MEASURE_NOISE
    },{
        GYRO_BIAS_PROCESS_NOISE,
        GYRO_BIAS_PROCESS_NOISE,
        GYRO_BIAS_PROCESS_NOISE,
    }};

    KalmanVector local_gyro {{
        GYRO_MEASURE_NOISE,
        GYRO_MEASURE_NOISE,
        GYRO_MEASURE_NOISE,
    },{
        GYRO_PROCESS_NOISE,
        GYRO_PROCESS_NOISE,
        GYRO_PROCESS_NOISE
    }};
};

#endif