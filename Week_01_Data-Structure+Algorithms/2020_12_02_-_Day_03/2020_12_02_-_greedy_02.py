# 탐욕법(Greedy) 대표 문제 02

'''
문제 설명
- 큰 수 만들기(level 2) -

입력된 수에서 k개의 수를 제거하여 가장 큰 수 만들기

number: 문자열 형식의 숫자
k: 제거할 수의 개수
return: 만들 수 있는 가장 큰 수

제한 사항
number: 1 ~ 1,000,000 이하의 수
k: 1 ~ number의 자리 수 미만의 자연수
'''

# number = '1924'
# k = 2            # return '94'

number = '1231234'
k = 3            # return '3234'

# number = '4177252841'
# k = 4            # return '775841'

# number에 작은 수가 동일하게 있을 때, 앞의 수를 빼는 것이 큰 수를 만들기 유리
# number안에 있는 숫자들의 순서는 유지하되,
# stack된 수가 stack 하려는 수보다 작을 경우 뺌
# 수를 뺄 때 k 갱신
# 만약 k를 다 충족시키지 못 했을 경우 가장 뒤의 숫자를 남은 k만큼 빼줌

def solution(number, k):
    # 숫자를 stack하기 위한 빈 리스트 생성
    tmp = []
    # number 의 수를 불러옴
    for num in number:
        # k의 개수가 남아있고, tmp에 수가 stack되어 있고,
        # stack된 수가 넣으려는 수보다 작을 때 실행
        while k > 0 and len(tmp) > 0 and tmp[-1] < num:
            # 수를 빼주므로 k 값 갱신
            k -= 1
            # stack 된 수 중 가장 마지막 원소 제거
            # 반복문을 순회하며 앞의 수들도 비교 하여 뺄 수는 빼줌
            tmp.pop()
        # num을 stack
        tmp.append(num)
    # 반복문 후에도 k의 개수가 남아 있다면
    # 남은 k 수만 큼 뒤에서부터 자름
    if k != 0:
        tmp = tmp[:-k]
    # stack된 숫자의 리스트 형식을 문자열로 변환
    answer = ''.join(tmp)

    return answer

print(solution(number, k))

# for문이 전체 시간 복잡도를 지배
# number의 길이 만큼에 비례하는 선형 시간 비례 복잡도 O(n)을 가짐

'''
def solution(number, k):
    answer = [number[0]]
    for num in number[1:]:
        while len(answer) > 0 and answer[-1] < num and k > 0:
            k -= 1
            answer.pop()
        answer.append(num)
    if k != 0:
        answer = answer[:-k]
    
    return ''.join(answer)
'''