#ifndef PARALLEL_Q_H
#define PARALLEL_Q_H

template <typename T>
class PARALLEL_Q
{
private:
    int counter = 0;
    int Q_LEN;
    bool is_empty, initialized = false;
public:
    T * q;
    int head = 0, tail = 0;
    PARALLEL_Q();
    PARALLEL_Q(int len);
    ~PARALLEL_Q();
    void init(int len);
    void pop();
    T front();
    T back();
    void clear();
    void push(T op);
    void push_parallel(T &op, int index);
    void push_pos(T &op, int index);
    bool push_parallel_prepare(int length);
    bool empty();
    int size();
};

template <typename T>
PARALLEL_Q<T>::PARALLEL_Q()
{
    initialized = false;
}

template <typename T>
PARALLEL_Q<T>::PARALLEL_Q(int len)
{
    Q_LEN = len;
    q = new T[Q_LEN];
    initialized = true;
}

template <typename T>
PARALLEL_Q<T>::~PARALLEL_Q()
{
    if (initialized) delete [] q;
}

template <typename T>
void PARALLEL_Q<T>::init(int len)
{
    if (initialized) delete[] q;
    Q_LEN = len;
    q = new T[Q_LEN];
    counter = 0;
    head = 0;
    tail = 0;
    is_empty = true;
    initialized = true;
}

template <typename T>
void PARALLEL_Q<T>::pop()
{
    assert(initialized && "Queue is not initialized!");
    if (counter == 0)
        return;
    head++;
    head %= Q_LEN;
    counter--;
    if (counter == 0)
        is_empty = true;
    return;
}

template <typename T>
T PARALLEL_Q<T>::front()
{
    assert(initialized && "Queue is not initialized!");
    return q[head];
}

template <typename T>
T PARALLEL_Q<T>::back()
{
    assert(initialized && "Queue is not initialized!");
    return q[(tail + Q_LEN -1) % Q_LEN];
}

template <typename T>
void PARALLEL_Q<T>::clear()
{
    assert(initialized && "Queue is not initialized!");
    head = 0;
    tail = 0;
    counter = 0;
    is_empty = true;
    return;
}

template <typename T>
void PARALLEL_Q<T>::push(T op)
{
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    q[tail] = op;
    counter++;
    if (is_empty)
        is_empty = false;
    tail++;
    tail %= Q_LEN;
}

template <typename T>
void PARALLEL_Q<T>::push_pos(T &op, int index)
{
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    counter++;
    index %= Q_LEN;
    q[index] = op;
    if (is_empty)
        is_empty = false;
    tail++;
    tail %= Q_LEN;
}

template <typename T>
void PARALLEL_Q<T>::push_parallel(T &op, int index)
{   
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {   
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    index %= Q_LEN;
    q[index] = op;
}

template <typename T>
bool PARALLEL_Q<T>::push_parallel_prepare(int length)
{
    assert(initialized && "Queue is not initialized!");
    while (counter + length >= Q_LEN)
    {
        if (counter == 0)
        {
            printf("[PARALLEL_Q] Frame too large for buffer (length=%d >= Q_LEN=%d). Frame dropped.\n",
                   length, Q_LEN);
            return false;
        }
        pop();
    }
    counter += length;
    if (is_empty)
        is_empty = false;
    tail += length;
    tail %= Q_LEN;
    return true;
}

template <typename T>
bool PARALLEL_Q<T>::empty()
{
    assert(initialized && "Queue is not initialized!");
    return is_empty;
}

template <typename T>
int PARALLEL_Q<T>::size()
{
    assert(initialized && "Queue is not initialized!");
    return counter;
} 

#endif