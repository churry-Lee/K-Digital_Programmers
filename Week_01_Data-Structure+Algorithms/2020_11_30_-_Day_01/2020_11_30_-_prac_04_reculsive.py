# 피보나치 수열 재귀 함수 풀이
def solution(x):

    if x >= 2:
        return solution(x-1) + solution(x-2)
    else:
        return x