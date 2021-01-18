'''
사탕 담기

m 그램(gram)을 담을 수 있는 가방에 사탕을 가득 채우는 경우의 수를 구하려 합니다. 
단, 같은 사탕은 또 넣을 수 없습니다.
가방이 감당할 수 있는 무게 m, 사탕별 무게가 담긴 배열 weights가 매개변수로 주어질 때, 
가방을 정확히 m 그램으로 채우는 경우의 수를 return 하는 solution 함수를 작성해주세요.

combination을 안 쓰고 풀 수 있는 방법 생각해보기
'''

m = 3000
weigths = [500, 1500, 2500, 1000, 2000]   # return 3

from itertools import combinations

def solution(m, weights):
    answer = 0

    for i in range(len(weights)):
        combi = combinations(weights, i)
        answer += [sum(candies) for candies in combi].count(m)

    return answer

print(solution(m, weigths))