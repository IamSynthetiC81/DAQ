#pragma once

#include <stdint.h>

template <typename T>
class CircularList {
public:
    CircularList(uint8_t size);
    ~CircularList();
    
    uint8_t push(T data);
    T pop();

    uint8_t getSize();

    bool isFull();
    bool isEmpty();
private:
    uint8_t LIST_MAX_SIZE = 10;
    T* list;

    uint8_t head = 0;
    uint8_t tail = 0;

    uint8_t size = 0;
};

template <typename T>
CircularList<T>::CircularList(uint8_t size) {
    this->LIST_MAX_SIZE = size;
    this->list = new T[this->LIST_MAX_SIZE];
    this->head = 0;
    this->tail = 0;
    this->size = 0;
}

template <typename T>
CircularList<T>::~CircularList() {
    delete[] this->list;
}

template <typename T>
uint8_t CircularList<T>::push(T data) {
    if (this->isFull()) return 1;

    this->list[this->tail] = data;
    this->tail = (this->tail + 1) % this->LIST_MAX_SIZE;
    this->size++;

    return 0;
}

template <typename T>
T CircularList<T>::pop() {
    if (this->isEmpty()) return 0;  // Assuming T can be zero-initialized

    T data = this->list[this->head];
    this->head = (this->head + 1) % this->LIST_MAX_SIZE;
    this->size--;

    return data;
}

template <typename T>
uint8_t CircularList<T>::getSize() {
    return this->size;
}

template <typename T>
bool CircularList<T>::isFull() {
    return this->size == this->LIST_MAX_SIZE;
}

template <typename T>
bool CircularList<T>::isEmpty() {
    return this->size == 0;
}
