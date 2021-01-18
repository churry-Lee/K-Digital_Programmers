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


def infixToPostfix(tokenList):
    opStack = ArrayStack()
    postfixList = []

    for token in tokenList:
        if type(token) is int:
            postfixList.append(token)
        elif token == '(':
            opStack.push(token)
        elif token == ')':
            while opStack.peek() != '(':
                postfixList.append(opStack.pop())
            opStack.pop()
        else:
            while not opStack.isEmpty():
                if prec[opStack.peek()] >= prec[token]:
                    postfixList.append(opStack.pop())
                else:
                    break
            opStack.push(token)
        print(opStack.data)

    while not opStack.isEmpty():
        postfixList.append(opStack.pop())

    return postfixList

print(infixToPostfix(['(', 1, '+', 2, ')', '*', '(', 3, '+', 4, ')']))