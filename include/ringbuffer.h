
#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

template <class T,int SIZE>
class RingBuffer {
  T buffer[SIZE];
  int numElements = 0;

public:

  // Adds new item to buffer, overwriting the oldes one if buffer is full
  void add(T& newItem){
    for(int i = SIZE - 1; i > 0; i--){
      buffer[i] = buffer[i-1];
    }
    buffer[0] = newItem;
    if(numElements < SIZE){
      numElements++;
    }
  }

  //Number of Elements in Buffer. 0 after clear(), always =< SIZE
  int getNumElements(){
    return numElements;
  }

  // Get element from buffer. 0 =< index < getNumElements()
  // index 0: newest Element
  // index getNumElements(): oldest Element
  T& getElement(int index){
    if(index <= getNumElements() && index >= 0){
      return buffer[index];
    }else{
    printf("Index is out of bounds");
    return buffer[0];
    }
  }

  // Clears Buffer
  void clear(){
    numElements = 0;
  }
};


#endif /* RINGBUFFER_H_ */
