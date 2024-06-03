#ifndef _CircularBuffer_h
#define _CircularBuffer_h

template < typename TYPE, int SIZE >
class CircularBuffer {
public:
    CircularBuffer() {
        for (int i = 0; i < SIZE; i++) {
            buffer[i] = 1500;
        }
    }

    void write(TYPE t) {
        buffer[tail] = t;
        tail = (tail + 1) % SIZE;
        head = tail == 0 ? SIZE - 1 : tail - 1;
    }

    TYPE readTail() {
        return buffer[tail];
    }

    TYPE readHead() {
        return buffer[head];
    }

    TYPE read(int index) {
        if (index >= SIZE) return 0;
        return buffer[(tail + index) % SIZE];
    }

private:
    TYPE buffer[SIZE];
    int head = 0, tail = 0;
};

#endif
