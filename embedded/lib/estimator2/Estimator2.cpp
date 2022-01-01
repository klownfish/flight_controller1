#include "Estimator2.h"
#include "Arduino.h"

#define RAD_TO_DEG (180/PI)
#define DEG_TO_RAD (PI/180)

void Estimator::insert_gyro(float gx, float gy, float gz, uint32_t time) {
    turbomath::Vector gyro_raw {gx, gy, gz};
    if (stationary) {
        gyro_bias.insert(gyro_raw);
    }
    local_gyro.insert(gyro_raw - gyro_bias.get_value());
}


void Estimator::insert_acceleration(float ax, float ay, float az, uint32_t time) {
    if (last_acceleration_update == 0) {
        last_acceleration_update = time;
    }
    float dt = (time - last_acceleration_update) / 1000000.0;
    last_acceleration_update = time;

    turbomath::Vector accel_raw {ax, ay, az};
    
    if (stationary) {
        measured_gravity_vector.insert(accel_raw);

        //find quaternion to rotate accel with
        turbomath::Vector cross = measured_gravity_vector.get_value().cross(gravity_vector);
        float dot = measured_gravity_vector.get_value().dot(gravity_vector);
        turbomath::Vector normal = cross.normalized();
        //get cosine from cross product
        float angle = PI - atan2(cross.norm(), dot); 
        float temp_sin = sinf(angle/2);
        float temp_cos = cosf(angle/2);
        turbomath::Quaternion q;
        q.x = temp_sin * normal.x;
        q.y = temp_sin * normal.y;
        q.z = temp_sin * normal.z;
        q.w = temp_cos;        
        imu_offset = q;

        //zero known values
        heading = stationary_heading;
    }

    //local gyro and accel
    local_accel.insert(imu_offset.rotate(accel_raw));
}

void Estimator::update(uint32_t time) {
    if (last_main_update == 0) {
        last_main_update = time;
    }
    float dt = (time - last_main_update) / 1000000.0;
    last_main_update = time;

    //heading
    turbomath::Quaternion gyro_quat {0, local_gyro.get_value().x, local_gyro.get_value().y, local_gyro.get_value().z};
    turbomath::Quaternion heading_deriv = (heading * (1/2.0)) * gyro_quat;
    heading = heading + heading_deriv * dt;
    heading = heading.normalize();
}

void Estimator::insert_pressure(float pressure, uint32_t time) {
    if (last_pressure_update == 0) {
        last_pressure_update = time;
    }
    float dt = (time - last_pressure_update) / 1000000;
    last_pressure_update = time;
    float raw_alt =  44330 * (1.0 - pow(pressure / 1003, 0.1903));
    altitude.insert(raw_alt);
}

float Estimator::get_altitude() {
    return altitude.get_value();
}

void Estimator::set_moving() {
    stationary = false;
}

void Estimator::set_stationary(float roll, float pitch, float yaw) {
    stationary = true;
    stationary_heading = {roll, pitch, yaw};
    gravity_vector = stationary_heading.rotate({0, 0, GRAVITY});
}

void Estimator::ignition_started() {
    ignited = true;
}

turbomath::Quaternion Estimator::get_heading() {
    return heading;
}

turbomath::Vector Estimator::get_local_acceleration() {
    return local_accel.get_value();
}