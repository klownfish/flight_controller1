#include "Estimator2.h"
#include "Arduino.h"

#define RAD_TO_DEG (180/PI)
#define DEG_TO_RAD (PI/180)

void Estimator::insert_imu(float ax, float ay, float az, float gx, float gy, float gz, uint32_t time) {
    if (last_imu_update == 0) {
        last_imu_update = time;
    }
    float dt = (time - last_imu_update) / 1000000.0;
    last_imu_update = time;

    turbomath::Vector accel_raw {ax, ay, az};
    turbomath::Vector gyro_raw {gx, gy, gz};

    if (stationary) {
        gyro_bias.insert(gyro_raw);
        measured_gravity_vector.insert(accel_raw);

        //find quaternion to rotate accel with
        turbomath::Vector cross = measured_gravity_vector.get_value().cross(gravity_vector);
        float dot = measured_gravity_vector.get_value().dot(gravity_vector);
        turbomath::Vector normal = cross.normalized();
        //get cosine from cross product
        float angle = -atan2(cross.norm(), dot); 
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
    local_gyro.insert(gyro_raw - gyro_bias.get_value());

    //#define MEGA_PRINT
    #ifdef MEGA_PRINT   
    float roll, pitch, yaw;
    heading.get_RPY(&roll, &pitch, &yaw);
    Serial.println("lol\n");
    Serial.println(dt);

    Serial.print("stationary: "); Serial.println(stationary);

    Serial.print("local accel x:"); Serial.println(local_accel.get_value().x);
    Serial.print("local accel y:"); Serial.println(local_accel.get_value().y);
    Serial.print("local accel z:"); Serial.println(local_accel.get_value().z);

    Serial.print("roll: "); Serial.println(roll * RAD_TO_DEG);
    Serial.print("pitch: "); Serial.println(pitch * RAD_TO_DEG);
    Serial.print("yaw: "); Serial.println(yaw * RAD_TO_DEG);

    Serial.println("YOYO");
    #endif
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
    //float raw_alt =  
    float old_alt = altitude.get_value();
    //altitude.insert()
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

turbomath::Vector Estimator::get_local_acc() {
    return local_accel.get_value();
}