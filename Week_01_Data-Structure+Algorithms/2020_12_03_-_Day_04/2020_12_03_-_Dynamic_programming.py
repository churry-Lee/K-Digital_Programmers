# 동적계획법(Dynamic Programming) 대표 문제

'''
문제 설명
- N으로 표현(level 3) -

숫자 N과 number가 주어질 때, 
N과 사칙연산만 사용해서 표현 할 수 있는 방법 중 
N 사용횟수의 최솟값을 return 하도록 solution 함수를 작성하세요.
N: 사용할 숫자
number: 표현 할 수
return: N 사용 횟 수

제한 사항
N: 1 ~ 9
number: 1 ~ 32,000
수식에는 괄호와 사칙연산만 가능하며 나누기 연산에서 나머지는 무시합니다.
최솟값이 8보다 크면 -1을 return 합니다.
'''

N = 5
number = 12    # return 4

# N = 2
# number = 11    # return 3

'''def solution(N, number):
    if N == number:
        return 1
    
    num_set = []

    for i in range(1, 9):
        num_set_part = {int(str(N)*(i))}
        for j in range(i // 2):
            set1 = num_set[j]
            set2 = num_set[i-j-2]

            for num1 in set1:
                for num2 in set2:
                    num_set_part.add(num1 + num2)
                    num_set_part.add(num1 - num2)
                    num_set_part.add(num2 - num1)
                    num_set_part.add(num1 * num2)
                    if num1 != 0:
                        num_set_part.add(num2 // num1)
                    if num2 != 0:
                        num_set_part.add(num1 // num2)

            if number in num_set_part:
                return i
        num_set.append(num_set_part)

    return -1'''

def solution(N, number):
    if N == number:
        return 1
    # 만들어지는 숫자를 저장할 리스트 변수
    # 제한사항이 8까지이기 때문에 8개의 set 만듦
    # set으로 중복된 원소 제거
    num_set = [set() for x in range(8)]
    # 5, 55, 555 등의 단순히 이어 붙여서 만들 수 있는 수 생성하여 추가
    for i, x in enumerate(num_set, start=1):
        x.add(int(str(N) * i))
    
    for i in range(1, len(num_set)):
        # i 번째 계산에서 필요한 하위 set 원소를 불러오기 위한 반복문
        for j in range(i):
            # set에서 각각의 원소를 불러오기 위한 반복문
            for num1 in num_set[j]:
                for num2 in num_set[i-j-1]:
                    num_set[i].add(num1 + num2)
                    num_set[i].add(num1 - num2)
                    num_set[i].add(num1 * num2)
                    if num2 != 0:
                        num_set[i].add(num1 // num2)
        # 숫자를 찾았을 때
        if number in num_set[i]:
            answer = i + 1
            break
    # 숫자가 없을 때(회수가 8 이상)
    else:
        answer = -1
        
    return answer

print(solution(N, number))