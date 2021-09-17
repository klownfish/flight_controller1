#pragma once

#include "turbomath/turbomath.h"
#include <math.h>
#include <Arduino.h>

#define HIGH_GAIN_FOR 300
#define HIGH_GAIN_MULIPLIER 0.1

#define GYRO_BIAS_PROCESS_NOISE 0.0000001
#define ACC_OFFSET_WEIGHT 0.01

#define GYRO_PROCESS_NOISE 0.5
#define GYRO_MEASUREMENT_NOISE 0.3

#define ACCEL_PROCESS_NOISE 3
#define ACCEL_MEASUREMENT_NOISE 0.5

#define ALTITUDE_MEASUREMENT_NOISE 0.5
#define ALTITUDE_PROCESS_NOISE 0.9

#define GRAVITY 9.82

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

class KalmanVector {
    turbomath::Vector state = {0, 0, 0};
    turbomath::Vector process_err = {1, 1, 1};
    turbomath::Vector process_noise;
    turbomath::Vector measurement_noise;
    turbomath::Vector derivative = {0, 0, 0};

public:
    KalmanVector(turbomath::Vector measure_noise, turbomath::Vector _process_noise) {
        measurement_noise = measure_noise;
        process_noise = _process_noise;
    }

    void insert(turbomath::Vector measurement, float dt = 0) {
        //predict
        turbomath::Vector prediction = state + dt * derivative;
        //iupdate process error
        process_err = process_err + process_noise;
        //calculate gain
        turbomath::Vector gain = {
            process_err.x / (process_err.x + measurement_noise.x),
            process_err.y / (process_err.y + measurement_noise.y),
            process_err.z / (process_err.z + measurement_noise.z)
        };

        //set new state
        state =  {
            prediction.x + gain.x * (measurement.x - prediction.x),
            prediction.y + gain.y * (measurement.y - prediction.y),
            prediction.z + gain.z * (measurement.z - prediction.z)
        };

        //set new process error
        process_err =  {
            (1 - gain.x) * process_err.x + process_noise.x,
            (1 - gain.y) * process_err.y + process_noise.y,
            (1 - gain.z) * process_err.z + process_noise.z
        };
    }

    turbomath::Vector get_value() {return state;}
    turbomath::Vector get_process_error() {return process_err;}

    turbomath::Vector set_derivative(turbomath::Vector val) {derivative = val;}
    turbomath::Vector set_value(turbomath::Vector val) {state = val;}
    turbomath::Vector set_process_noise(turbomath::Vector val) {process_noise = val;}
    turbomath::Vector set_measurement_noise(turbomath::Vector val) {measurement_noise = val;}
};


//x - yaw
//y - pitch
//z - Roll
class Estimator {
public:
    Kalman1 altitude{ALTITUDE_MEASUREMENT_NOISE, ALTITUDE_PROCESS_NOISE};
    float ground_level = 0;

    turbomath::Vector heading {0, 0, 0};
    turbomath::Vector gyro {0, 0, 0};
    turbomath::Vector vel {0, 0, 0};
    turbomath::Vector local_acc {0, 0, 0};
    turbomath::Vector linear_acc {0, 0, 0};
    turbomath::Vector velocity {0, 0, 0};
    turbomath::Vector position {0, 0, 0};
    float dt = 0;
    float high_gain_remaining = HIGH_GAIN_FOR;
    bool stationary = false;
    turbomath::Vector known_heading {0, 0, 0};
    turbomath::Vector gravity_vector {0, 0, 1};

    turbomath::Quaternion imu_offset {1, 0, 0, 0};
    

    KalmanVector kalman_acc {{0.5,0.5,0.5}, {1, 1, 1}};
    float acc_scale = 1;

    turbomath::Vector gyro_bias {0, 0, 0};
    KalmanVector kalman_gyro_bias {
        {
            GYRO_MEASUREMENT_NOISE, 
            GYRO_MEASUREMENT_NOISE, 
            GYRO_MEASUREMENT_NOISE      
        },  
        {
            GYRO_BIAS_PROCESS_NOISE,
            GYRO_BIAS_PROCESS_NOISE,
            GYRO_BIAS_PROCESS_NOISE
        }
    };

    KalmanVector kalman_gyro {
        {
            GYRO_MEASUREMENT_NOISE, 
            GYRO_MEASUREMENT_NOISE, 
            GYRO_MEASUREMENT_NOISE
        },
        {
            GYRO_PROCESS_NOISE,
            GYRO_PROCESS_NOISE,
            GYRO_PROCESS_NOISE
        }
    };
    turbomath::Vector old_gyro[2] = {{0, 0, 0},{0, 0, 0}};

    void set_stationary(float yaw, float pitch, float roll) {
        turbomath::Vector g {0, 0, GRAVITY};
        turbomath::Quaternion rot_q = turbomath::Quaternion{yaw, pitch, roll};
        known_heading = {yaw, pitch, roll};
        gravity_vector = rot_q.rotate(g);
        stationary = true;
    }

    void set_moving() {
        stationary = false;
    }

    void update_imu(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float _dt) {
        dt = _dt;
        turbomath::Vector gyro_measurement = {
            gyro_x,
            gyro_y,
            gyro_z
        };
        gyro_measurement *= M_PI / 180; 

        turbomath::Vector acc_measurement = {
            acc_x,
            acc_y,
            acc_z
        };
        acc_measurement *= GRAVITY;

        if (stationary) {
            float modifier = (high_gain_remaining-- > 0) ? HIGH_GAIN_MULIPLIER * high_gain_remaining : 1;
            kalman_gyro_bias.insert(gyro_measurement);
            gyro_bias = kalman_gyro_bias.get_value();

            float absolute = acc_measurement.norm() * gravity_vector.norm();
            turbomath::Vector normal = acc_measurement.cross(gravity_vector);
            turbomath::Vector unit_normal = normal.normalized();
            //can probably do some epic vector math to get this directly
            float angle = -asinf(normal.norm() / absolute);
            float temp_sin = sinf(angle/2);
            float temp_cos = cosf(angle/2);
            turbomath::Quaternion q;
            q.x = temp_sin * unit_normal.x;
            q.y = temp_sin * unit_normal.y;
            q.z = temp_sin * unit_normal.z;
            q.w = temp_cos;
            
            imu_offset = turbomath::Quaternion {
                imu_offset.w - ACC_OFFSET_WEIGHT * modifier * (imu_offset.w - q.w),
                imu_offset.x - ACC_OFFSET_WEIGHT * modifier * (imu_offset.x - q.x),
                imu_offset.y - ACC_OFFSET_WEIGHT * modifier * (imu_offset.y - q.y),
                imu_offset.z - ACC_OFFSET_WEIGHT * modifier * (imu_offset.z - q.z)
            };
        }
        //update kalman filters
        kalman_gyro.insert(gyro_measurement - gyro_bias, dt);
        kalman_acc.insert(acc_measurement, dt);
        
        //reset high gain mode
        high_gain_remaining = HIGH_GAIN_FOR;

        //rotate to calibrated position
        local_acc = imu_offset.rotate(kalman_acc.get_value());
        gyro = imu_offset.rotate(kalman_gyro.get_value());

        //smooth gyro
        turbomath::Vector smoothed_gyro =  (-1.0 * old_gyro[1] + 8.0 * old_gyro[0] + 5.0 * gyro) / 12.0;
        old_gyro[1] = old_gyro[0];
        old_gyro[0] = gyro;
        gyro = smoothed_gyro;

        //calculate heading
        heading += gyro * dt;

        turbomath::Vector g {0, 0, -1};
        turbomath::Quaternion q_real {heading.x, heading.y, heading.z};
        q_real.invert();
        linear_acc = q_real.rotate(local_acc) + g;
        velocity += linear_acc * dt;
        position += velocity * dt;
    }

    void update_altitude(float value, float dt) {
        altitude.insert(value, dt);
        if (stationary) {

            ground_level = altitude.get_value();
        }
    }

    float get_altitude() {
        return altitude.get_value() - ground_level;
    }

    float get_ax() {return local_acc.x;}
    float get_ay() {return local_acc.y;}
    float get_az() {return local_acc.z;}
    float get_gx() {return gyro.x;}
    float get_gy() {return gyro.y;}
    float get_gz() {return gyro.z;}
    float get_hx() {return heading.x;}
    float get_hy() {return heading.y;}
    float get_hz() {return heading.z;}

    void print_everything() {
        Serial.println();
        Serial.println("-------------------");
        Serial.print("stationary: "); Serial.println(stationary);
        Serial.print("gravity x: "); Serial.println(gravity_vector.x);
        Serial.print("gravity y: "); Serial.println(gravity_vector.y);
        Serial.print("gravity z: "); Serial.println(gravity_vector.z);
        Serial.print("bias x: "); Serial.println(gyro_bias.x * 180 / M_PI);
        Serial.print("bias y: "); Serial.println(gyro_bias.y * 180 / M_PI);
        Serial.print("bias z: "); Serial.println(gyro_bias.z * 180 / M_PI);
        Serial.print("gyro x: "); Serial.println(gyro.x * 180 / M_PI);
        Serial.print("gyro y: "); Serial.println(gyro.y * 180 / M_PI);
        Serial.print("gyro z: "); Serial.println(gyro.z * 180 / M_PI);
        Serial.print("acc scale: "); Serial.println(acc_scale);
        Serial.print("heading x: "); Serial.println(heading.x * 180 / M_PI);
        Serial.print("heading y: "); Serial.println(heading.y * 180 / M_PI);
        Serial.print("heading z: "); Serial.println(heading.z * 180 / M_PI);
        Serial.print("local acc x: "); Serial.println(local_acc.x);
        Serial.print("local acc y: "); Serial.println(local_acc.y);
        Serial.print("local acc z: "); Serial.println(local_acc.z);
        Serial.print("linear acc x "); Serial.println(linear_acc.x);
        Serial.print("linear acc y "); Serial.println(linear_acc.y);
        Serial.print("linear acc z "); Serial.println(linear_acc.z);
        Serial.print("velocity x: "); Serial.println(velocity.x);
        Serial.print("velocity y: "); Serial.println(velocity.y);
        Serial.print("velocity z: "); Serial.println(velocity.z);
        Serial.print("dt: "); Serial.println(dt * 1000);
        Serial.println("-------------------");
        Serial.println();
    }
};