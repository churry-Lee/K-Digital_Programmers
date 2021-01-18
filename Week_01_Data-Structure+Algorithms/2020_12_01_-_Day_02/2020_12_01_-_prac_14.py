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

# 문자열 형태의 수식을 tokens 리스트로 반환하는 함수
def splitTokens(exprStr):
    tokens = []
    val = 0
    valProcessing = False
    for c in exprStr:
        if c == ' ':
            continue
        if c in '0123456789':
            val = val * 10 + int(c)
            valProcessing = True
        else:
            if valProcessing:
                tokens.append(val)
                val = 0
            valProcessing = False
            tokens.append(c)
    if valProcessing:
        tokens.append(val)

    return tokens

# 중위 표현식으로 되어 있는 tokens 리스트를 후휘 표현식 리스트로 변환
def infixToPostfix(tokenList):
    prec = {
        '*': 3,
        '/': 3,
        '+': 2,
        '-': 2,
        '(': 1,
    }

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

    while not opStack.isEmpty():
        postfixList.append(opStack.pop())

    return postfixList


def postfixEval(tokenList):
    valStack = ArrayStack()

    for token in tokenList:
        if type(token) is int:
            valStack.push(token)
        elif token == '*':
            num_1 = valStack.pop()
            num_2 = valStack.pop()
            valStack.push(num_2 * num_1)
        elif token == '/':
            num_1 = valStack.pop()
            num_2 = valStack.pop()
            valStack.push(num_2 / num_1)
        elif token == '+':
            num_1 = valStack.pop()
            num_2 = valStack.pop()
            valStack.push(num_2 + num_1)
        elif token == '-':
            num_1 = valStack.pop()
            num_2 = valStack.pop()
            valStack.push(num_2 - num_1)

        print(valStack.data)

    return valStack.pop()


def solution(expr):
    # 문자열 형태의 수식을 tokens 리스트로 반환하는 함수
    tokens = splitTokens(expr)
    print('중위표현식', tokens)
    # 중위 표현식으로 되어 있는 tokens 리스트를 후휘 표현식 리스트로 변환하는 함수
    postfix = infixToPostfix(tokens)
    print('후휘표현식', postfix)
    # 후회 표현식을 계산하는 함수
    val = postfixEval(postfix)
    return val

expr = "(1+2)*(3+4)"   # return -1
print(solution(expr))