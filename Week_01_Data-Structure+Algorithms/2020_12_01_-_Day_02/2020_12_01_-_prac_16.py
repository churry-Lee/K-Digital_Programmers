# 리스트를 이용한 환형 큐의 ADT 구현

class CircularQueue:
    def __init__(self, n):
        self.maxCount = n
        self.data = [None] * n
        self.count = 0
        self.front = -1
        self.rear = -1

    def size(self):
        return self.count
    
    def isEmpty(self):
        return self.count == 0

    def isFull(self):
        return self.count == self.maxCount
    # rear 포인터 이동
    def enqueue(self, x):
        if self.isFull():
            raise IndexError('Qeueue is Full')
        # 나머지 연산을 통해 위치 지정
        self.rear = (self.rear + 1) % self.maxCount
        self.data[self.rear] = x
        self.count += 1
    # front 포인터 이동
    def dequeue(self):
        if self.isEmpty():
            raise IndexError('Queue is Empty')
        self.front = (self.front + 1) % self.maxCount
        x = self.data[self.front]
        self.count -= 1
        return x

    def peek(self):
        if self.isEmpty():
            raise IndexError('Queue is Empty')
        return self.data[(self.front + 1) % self.maxCount]