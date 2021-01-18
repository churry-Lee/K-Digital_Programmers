# 괄호 짝 맞추기
# 괄호의 쌍이 올바른지 유효성 검사
# 차근 차근 잘 생각하며 문제 분석을 해보자.

s = '{{}}'
# s = '({})[]'
# s = '[)'
# s = '({)}'

def solution(s):
    stack = []
    match = {
        ')': '(',
        '}': '{',
        ']': '['
    }

    for char in s:
        if char in '({[':
            stack.append(char)
        elif char in match:
            if len(stack) == 0:
                return False
            else:
                t = stack.pop()
                if t != match[char]:
                    return False
    return len(stack) == 0

print(solution(s))