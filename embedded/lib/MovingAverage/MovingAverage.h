#include <stdint.h>

template <typename T>
class MovingAverage {
public:
    MovingAverage(int16_t length);
    void insert(T value);
    T get_value();
private:
    int16_t length_ = 0;
    T current_ = 0;
};

template <typename T>
MovingAverage<T>::MovingAverage(int16_t len) {
    length_ = len;
    current_ = 0;
}

template <typename T>
void MovingAverage<T>::insert(T value) {
    current_ += (value - current_) / length_;
}

template <typename T>
T MovingAverage<T>::get_value() {
    return current_;
}