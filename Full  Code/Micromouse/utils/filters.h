/**
 * @file filters.h
 * @brief Software filters (Low-pass, moving average).
 */

#ifndef FILTERS_H
#define FILTERS_H

/**
 * @brief Simple exponential moving average (EMA) low-pass filter.
 */
class LowPassFilter {
public:
    LowPassFilter(float alpha);
    float update(float new_val);
    float get() const;
    void reset(float initial_val);

private:
    float _alpha;
    float _value;
};

#endif /* FILTERS_H */
