# 세 소수의 합

'''
주어진 n을 3개의 소수의 합으로 구할 수 있는 경우의 수를 구하여라

ex) n = 33
3 + 7 + 23
3 + 11 + 19
3 + 13 + 17
5 + 11 + 17

에라토스테스트의 체의 알고리즘을 이용하라
'''

n = 33
'''
def solution(n):
    a = [False, False] + [True]*(n-1)
    primes=[]

    for i in range(2,n+1):
        if a[i]:
            primes.append(i)
        for j in range(2*i, n+1, i):
            a[j] = False
    print(primes)

    comb = []

    for i in primes:
        for j in primes:
            for k in primes:
                if i == j or i == k or j == k:
                    continue
                else:
                    if i + j + k == n:
                        comb.append(sorted([i, j, k]))
                    else:
                        continue
    
    comb = set(list(map(str, comb)))

    return len(comb)

*반복문을 이용한 풀이 - 정확성 테스트는 통과, 효율성 테스트 시간 초과*
'''
from itertools import combinations

def solution(n):
    answer = 0
    
    # 에라토스테스트의 체
    sieve = [True] * (n + 1)

    root = int(n ** 0.5) + 1
    for i in range(2, root):
        if sieve[i] == True:
            for j in range(2 * i, n + 1, i):
                sieve[j] = False
    primes = [i for i in range(2, n + 1) if sieve[i] == True]

    # 부분집합의 합
    comb = combinations(primes, 3)
    for c in comb:
        if sum(c) == n:
            answer += 1

    return answer

print(solution(n))