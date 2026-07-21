/**
 * @file filters.cpp
 * @brief Filters implementation.
 * @see filters.h
 */

#include "filters.h"

LowPassFilter::LowPassFilter(float alpha) : _alpha(alpha), _value(0.0f) {}

float LowPassFilter::update(float new_val) {
    _value = _alpha * new_val + (1.0f - _alpha) * _value;
    return _value;
}

float LowPassFilter::get() const {
    return _value;
}

void LowPassFilter::reset(float initial_val) {
    _value = initial_val;
}
