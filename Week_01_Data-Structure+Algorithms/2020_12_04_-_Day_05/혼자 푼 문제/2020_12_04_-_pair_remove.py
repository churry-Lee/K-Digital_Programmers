# 짝 지어 제거하기

'''
문제 설명
주어진 문자열에서 같은 알파벳이 2개 붙어 있는 짝을 찾는다.
그 다음 그 둘을 제거 하고, 문자열을 이어 붙인다.
이 과정을 반복하여 모두 제거한다면 종료
    + S: 문자열
    + *return: 제거 가능하면 1, 아니면 0*

제한 사항
* 문자열의 길이: 1,000,000
* 문자열은 모두 소문자로 이루어져 있음
'''

S = 'baabaa'    # return 1
# S = 'cdcd'      # return 0

def solution(S):
    # 문자열을 리스트로 변환
    S = list(S)
    # stack 리스트 생성, 편리한 코딩 작성을 위해 dummy 원소 추가
    stack = ['0']

    # 반복문을 실행하며 짝이 있는 지 검사
    for i in range(len(S)):
        # 스택된 마지막 문자와 스택될 문자가 같지 않으면 스택
        if stack[-1] != S[i]:
            stack.append(S[i])
        # 같으면 스택 안하고, 스택된 마지막 문자 제거
        else:
            stack.pop()

    # dummy 원소가 1개 있기 때문에, 원소가 1개만 남으면 True(1) 리턴
    if len(stack) == 1:
        return 1
    # 그렇지 않으면 False(0) 리턴
    else:
        return 0

print(solution(S))