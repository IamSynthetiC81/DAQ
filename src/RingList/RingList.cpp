// #include "CircularList.h"

// template <typename T>
// CircularList<T>::CircularList(uint8_t size) {
//     this->LIST_MAX_SIZE = size;
//     this->list = new T[this->LIST_MAX_SIZE];
//     this->head = 0;
//     this->tail = 0;
//     this->size = 0;
// }

// template <typename T>
// CircularList<T>::~CircularList() {
//     delete[] this->list;
// }

// template <typename T>
// uint8_t CircularList<T>::push(T data) {
//     if (this->isFull()) return 1;

//     this->list[this->tail] = data;
//     this->tail = (this->tail + 1) % this->LIST_MAX_SIZE;
//     this->size++;

//     return 0;
// }

// template <typename T>
// T CircularList<T>::pop() {
//     if (this->isEmpty()) return nullptr;

//     T data = this->list[this->head];
//     this->head = (this->head + 1) % this->LIST_MAX_SIZE;
//     this->size--;

//     return data;
// }

// template <typename T>
// uint8_t CircularList<T>::getSize() {
//     return this->size;
// }

