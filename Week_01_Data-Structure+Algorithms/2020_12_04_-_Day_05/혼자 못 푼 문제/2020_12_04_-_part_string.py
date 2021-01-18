# 사전 순 부분문자열

'''
문제 설명

주어진 문자열의 부분집합을 구한 후 길이와 무관하게 사전순으로 가장 마지막에 오는 문자를 찾아라
단, 문자의 순서는 바뀌지 않는다.
    + s: 문자열
    + *return: 사전 순 가장 마지막 문자*

제한 사항
* s: 길이 1 ~ 1,000,000 문자열
* s는 소문자 알파벳으로만 이루어져 있음
'''

s = 'xyb'    # result 'yb'
# s = 'yxtc'   # result 'yyc'

# 깊이 우선 탐색을 이용하여 부분집합 구하기
# 부분집합을 구하고 값을 찾을 경우 시간 초과
# 부분집합을 구하는 것도 어려워서 이해가 잘 안감

# def solution(s):
#     stack = []

#     # 부분집합을 구하기 위한 함수
#     def dfs(index, path):
#         stack.append(path)

#         for i in range(index, len(s)):
#             dfs(i + 1, path + [s[i]])
    
#     dfs(0, [])

#     stack.sort()

#     return ''.join(stack[-1])

# 알파벳 순서가 유지 된다고 했으므로,
# 스택을 하며 알파벳의 대소 비교 시행
# 스택된 문자가 스택될 문자보다 작을 경우 제거
# 그렇지 않으면 스택
# 따라서 이를 반복문으로 수행하면 사전순으로 가장 마지막에 올 문자 찾을 수 있음

def solution(s):
    stack = []

    for i in s:
        while len(stack) > 0 and stack[-1] < i:
            stack.pop()
        stack.append(i)

    return ''.join(stack)

print(solution(s))