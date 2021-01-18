class MyStack(object):
    def __init__(self):
        self.lst = list()

    def push(self, x):
        self.lst.append(x)

    def pop(self):
        return self.lst.pop()

    def size(self):
        return len(self.lst)

class MyQueue(object):
    def __init__(self, max_size):
        self.stack1 = MyStack()
        self.stack2 = MyStack()   # pop을 할 때 stack1에 있는 원소를 역순으로 저장해주기 위한 변수
        self.max_size = max_size

    def qsize(self):
        # pop을 하고 push를 하면 pop 이전 까지 push된 원소들이 stack2에 쌓이게 되므로,
        # 전체 길이는 stack1과 2의 합으로 계산해야됨
        return self.stack1.size() + self.stack2.size()

    def push(self, item):
        if self.qsize() < self.max_size:
            self.stack1.push(item)
            return True
        else:
            return False
            
    def pop(self):
        if self.stack2.size() == 0:
            while self.stack1.size() > 0:
                self.stack2.push(self.stack1.pop())
        return self.stack2.pop()

# n = int(input('반복 횟수를 입력하세요> ' ))
# max_size = int(input('최대 용량을 입력하세요> ' ))
# print('반복 횟수: {}, 최대 용량: {}'.format(n, max_size))

n, max_size = map(int, input().strip().split(' '))

q = MyQueue(max_size)
for _ in range(n):
    # item은 가변 인자로
    command, *item = input().strip().split(' ')
    # 입력은 대소문자 구별 없이, 읽을 때는 대문자로 변환하여 확인
    if command.upper() == 'PUSH':
        # 가변 인자를 사용하면 패킹이 되기 때문에 언패킹 해주기 위해 item[0]
        print(q.push(item[0]))
    elif command.upper() == 'POP':
        print(q.pop())
    elif command.upper() == 'SIZE':
        print(q.qsize())