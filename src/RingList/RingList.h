#ifndef RINGLIST_H
#define RINGLIST_H

#include <stdint.h>

template <typename T>
class RingList {
public:
    RingList(uint8_t size);
    ~RingList();
    
    uint8_t push(T data);
    T pop();

    uint8_t getSize();

    bool isFull();
    bool isEmpty();

private:
    const uint8_t LIST_MAX_SIZE;
    T* list;

    uint8_t head;
    uint8_t tail;

    uint8_t size;
};

template <typename T>
RingList<T>::RingList(uint8_t size) 
    : LIST_MAX_SIZE(size), list(new T[size]), head(0), tail(0), size(0) {
}

template <typename T>
RingList<T>::~RingList() {
    delete[] this->list;
}

template <typename T>
uint8_t RingList<T>::push(T data) {
    if (this->isFull()) return 1;

    this->list[this->tail] = data;
    this->tail = (this->tail + 1) % this->LIST_MAX_SIZE;
    this->size++;

    return 0;
}

template <typename T>
T RingList<T>::pop() {
    if (this->isEmpty()) {
        // Handle the case when the list is empty
        // Depending on your use case, you might throw an exception,
        // return a default-constructed T, or handle it differently
        return T(); // Return default-constructed T
    }

    T data = this->list[this->head];
    this->head = (this->head + 1) % this->LIST_MAX_SIZE;
    this->size--;

    return data;
}

template <typename T>
uint8_t RingList<T>::getSize() {
    return this->size;
}

template <typename T>
bool RingList<T>::isFull() {
    return this->size == this->LIST_MAX_SIZE;
}

template <typename T>
bool RingList<T>::isEmpty() {
    return this->size == 0;
}

#endif  // RINGLIST_H