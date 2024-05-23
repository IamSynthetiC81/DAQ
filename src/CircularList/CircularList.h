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
    private:
        uint8_t LIST_MAX_SIZE = 10;
        T* list = new T[LIST_MAX_SIZE];

        uint8_t head = 0;
        uint8_t tail = 0;

        uint8_t size = 0;

        bool isFull();
        bool isEmpty();
};