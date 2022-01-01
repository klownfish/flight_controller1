class Pid {
    float p;
    float i;
    float d;

    float setpoint;

    float integral;
    float value;
    float last_error;

public:
    Pid(float p_, float i_, float d_) {
        p = p_;
        i = i_;
        d = d_;
    }

    float update(float value, float dt) {
        float error = setpoint - value;
        float rate_error = (error - last_error) / dt;
        integral += error * dt;
        last_error = error;
        return p * error + i * integral + d * rate_error;
    }

    void set_setpoint(float value) {
        setpoint = value;
    }

    float get_setpoint() {
        return setpoint;
    }
};