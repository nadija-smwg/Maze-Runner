/**
 * @file ring_buffer.h
 * @brief Generic ring buffer implementation.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Generic Ring Buffer structure (e.g., for sensor data).
 */
template <typename T, size_t Size>
class RingBuffer {
public:
    RingBuffer() : _head(0), _tail(0), _count(0) {}

    bool push(const T& item) {
        if (is_full()) return false;
        _buffer[_head] = item;
        _head = (_head + 1) % Size;
        _count++;
        return true;
    }

    bool pop(T& item) {
        if (is_empty()) return false;
        item = _buffer[_tail];
        _tail = (_tail + 1) % Size;
        _count--;
        return true;
    }

    bool is_empty() const { return _count == 0; }
    bool is_full() const { return _count == Size; }
    size_t count() const { return _count; }

private:
    T _buffer[Size];
    size_t _head;
    size_t _tail;
    size_t _count;
};

#endif /* RING_BUFFER_H */
