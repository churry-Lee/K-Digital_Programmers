# 정렬(sort) 대표 문제

'''
문제 설명
- 가장 큰 수(level 2) -

0 또는 양의 정수가 주어졌을 때, 
정수를 이어 붙여 만들 수 있는 가장 큰 수를 구하라

numbers: 0 또는 양의 정수가 담긴 배열
return: numbers의 순서를 재배치하여 만들 수 있는 가장 큰 수

제한 사항
numbers: 길이 1 ~ 100,000
numbers의 원소: 0 ~ 1,000
return: 문자열 형태
'''

numbers = [6, 10, 2]   # return "6210"
# numbers = [3, 30, 34, 5, 9]   # return "9534330"

def solution(numbers):
    numbers = list(map(str, numbers))
    numbers.sort(key = lambda x : x * 3, reverse=True)

    return str(int(''.join(numbers)))

print(solution(numbers))

# 19행의 정렬이 시간 복잡도 지배
# 정렬 함수는 O(nlon)의 시간 복잡도를 가짐