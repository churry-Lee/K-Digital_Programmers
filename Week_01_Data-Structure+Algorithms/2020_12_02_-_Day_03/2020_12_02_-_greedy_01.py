# 탐욕법(Greedy) 대표 문제 01

'''
문제 설명
- 체육복(level 1) -

여벌 체육복이 있는 학생이 도둑맞은 학생들에게 빌려주려 함
바로 앞번호의 학생이나 바로 뒷번호의 학생에게만 체육복을 빌려줄 수 있음

n: 전체 학생 수 
lost: 도둑맞은 학생들의 번호가 담긴 배열
reserve: 여벌 체육복을 가져온 학생들의 번호가 담긴 배열
return: 체육수업을 들을 수 있는 학생의 최댓값

제한 사항
전체 학생 수 2 ~ 30명
도난당한 학생 수 1 ~ n 이하(중복 번호 없음)
여벌 체육복이 있어야지만 빌려줄 수 있음
여벌 체육복을 가져온 학생이 도둑 맞았을 경우 빌려줄 수 없음
'''
'''
탐욕법(Greedy)

알고리즘의 각 단계에서, 그 순간에 최적이라고 생각되는 것을 선택
완전한 최적은 아니지만, 그에 준하는 solution을 내놓음

탐욕법은 진짜 이해 안감....
'''

n = 5
lost = [2, 4]
reserve = [1, 3, 5]   # return 5

# n = 5
# lost = [2, 4]
# reserve = [3]   # return 4

# n = 3
# lost = [3]
# reserve = [1]   # return 2

def solution(n, lost, reserve):
    answer = 0
    # 학생 수의 크기 + 2(편의성을 위해, 맨 앞과 맨 뒤에 dummy 생성)의 크기 만큼
    # 배열 생성, 모든 학생이 1개의 체육복을 가졌다고 가정
    stu = [1] * (n+2)
    # 학생들이 도둑맞고 여벌로 가져온 체육복 수를 고려하여
    # 현재 학생들의 체육복 현황을 저장하기 위한 코드
    for i in range(len(lost)):
        stu[lost[i]] -= 1
    for i in range(len(reserve)):
        stu[reserve[i]] += 1
    # 자기 앞 또는 뒤의 학생이 체육복이 없고, 자기가 체육복이 2개 일 때,
    # 체육복 빌려줌
    # 위에서 초기값을 학생 수보다 2 더 크게 한 이유가,
    # 아래 반복문의 index error를 피하기 위해 dummy를 생성한 것임
    for i in range(1, n+1):
        if stu[i-1] == 0 and stu[i] == 2:
            stu[i-1], stu[i] = 1, 1
        elif stu[i] == 2 and stu[i+1] == 0:
            stu[i], stu[i+1] = 1, 1
    # 맨 앞과 뒤의 dummy를 제외하고 체육복이 0 이상인 학생 수를 계산
    for i in stu[1:len(stu)-1]:
        if i > 0:
            answer += 1

    return answer

print(solution(n, lost, reserve))

# 학생의 수에 비례하는 O(n)의 시간 복잡도를 가짐

'''
def solution(n, lost, reserve):
    answer = 0
    
    s = set(lost) & set(reserve)
    l = set(lost) - s
    r = set(reserve) - s
    
    for i in sorted(r):
        if i - 1 in l:
            l.remove(i - 1)
        elif i + 1 in l:
            l.remove(i + 1)
            
    answer = n - len(l)
    
    return answer
'''