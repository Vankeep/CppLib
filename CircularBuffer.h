#ifndef _CircularBuffer_h
#define _CircularBuffer_h
template < typename TYPE, int SIZE >
class CircularBuffer {
public:
     void write(T t){
        buffer[tail] = t;
        tail = (tail + 1) % SIZE;
        head = tail == 0 ? SIZE - 1 : tail - 1;
     }

     T readTail(){
        return buffer[tail];
     }

     T readHead(){
        return buffer[head];
     }

     T read(int index){
        if (index > size) return 0;
        return buffer[(tail + index) % SIZE];
     }

private:
    TYPE buffer[SIZE + 1];
    int head = 0, tail = 0;
};
#endif