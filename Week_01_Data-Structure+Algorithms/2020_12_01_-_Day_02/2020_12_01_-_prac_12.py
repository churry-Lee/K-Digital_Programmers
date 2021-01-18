# 중위표현 수식 -> 후위표현 수식

class ArrayStack:

    def __init__(self):
        self.data = []

    def size(self):
        return len(self.data)

    def isEmpty(self):
        return self.size() == 0

    def push(self, item):
        self.data.append(item)

    def pop(self):
        return self.data.pop()

    def peek(self):
        return self.data[-1]

prec = {
    '*': 3, '/': 3,
    '+': 2, '-': 2,
    '(': 1
    }

def solution(S):
    opStack = ArrayStack()
    answer = ''
    for char in S:
        print('char', char)
        # char가 피연산자인 경우, 그리고 괄호가 아닌 경우
        if char not in prec and char != ')':
            answer += char
        # char가 닫힌 괄호일 경우
        elif char == ')':
            # 열린 괄호가 나올 때까지 stack 되어 있는 연산자를 빼옴
            while opStack.peek() != '(':
                answer += opStack.pop()
            # stack 되어 있는 마지막 원소인 열린 괄호 제거
            opStack.pop()

        # char가 연산자인 경우
        else:
            # stack이 비어있지 않고, 열린 괄호가 아닐 경우 반복문 실행
            while not opStack.isEmpty() and prec[char] != 1:
                # stack 되어 있는 마지막 원소가 불러온 연산자보다 우선순위가 크거나 같을 경우
                if prec[opStack.peek()] >= prec[char]:
                    # stack 되어 있는 마지막 원소를 불러옴
                    answer += opStack.pop()
                # 종료 조건
                else:
                    break
            opStack.push(char)

        print(opStack.data, '\n')
    # 수식을 전부 스캔하고 나서도 스택되어 있는 연산자가 있을 경우 모드 불러옴
    while not opStack.isEmpty():
        answer += opStack.pop()

    return answer

print(solution('A-B*C+D'))


























'''def solution(S):
    opStack = ArrayStack()
    answer = ''
    
    for char in S:
        
        if char == '(':
            opStack.push(char)
            
        elif char == ')':
            while opStack.peek() != '(':
                answer += opStack.pop()
            opStack.pop()
            
        elif char in '+-*/':
            if opStack.isEmpty():
                opStack.push(char)
            else:
                while opStack.size() > 0 :
                    if prec[opStack.peek()] >= prec[char]:
                        answer += opStack.pop()
                    else:
                        break
                opStack.push(char)
                
        else:
            answer += char

    while not opStack.isEmpty():
        answer += opStack.pop()
        
    return answer'''