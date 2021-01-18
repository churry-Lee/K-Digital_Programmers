# 해쉬(Hash) 대표 문제

'''
문제 설명
- 완주하지 못한 선수(level 1) -
(https://programmers.co.kr/learn/courses/30/lessons/42576)

마라톤에 참여한 선수 중 단 한 명의 선수만 완주를 못 함
participant: 마라톤에 참여한 선수들의 이름이 담긴 배열
completion: 완주한 선수들의 이름이 담긴 배열
return: 완주하지 못한 선수의 이름

제한사항
선수의 수: 1 ~ 100,000
len(completion) = len(participant) - 1
선수의 이름: 길이 1 ~ 20 알파벳 소문자
*동명이인이 있을 수 있음
'''
'''
해시(hash) or 해시 테이블(hash table)

주어진 입력값을 key로 하여, hash function을 사용해
빈 공간(저장소, bucket or slot)에 매칭되는 값을 저장하는 자료구조

=> key를 value에 mapping할 수 있는 자료구조 

Dictionary 구조를 이용하기 때문에 각각의 값에 상수시간에 접근 가능
'''

participant = ['leo', 'kiki', 'eden']
completion = ['eden', 'kiki']

def solution(participant, completion):
    dic_name = {}

    for name in participant:
        if name not in dic_name:
            dic_name[name] = 1
        else:
            dic_name[name] += 1

    # get 함수를 사용해 위 for문을 간단히 구현할 수 있음
    # for name in participant:
    #     dic_name[name] = dic_name.get(name, 0) + 1


    for name in completion:
        dic_name[name] -= 1

    for key in dic_name:
        if dic_name[key] == 1:
            return key

    # 리스트 컴프리헨션으로 바로 위 for문을 재작성    
    # answer = [key for key, val in dic_name.items() if val == 1]
    # return answer[0]

print(solution(participant, completion))

# participant의 길이에 비례하는 O(n)의 시간 복잡도를 가짐
# hash를 사용하면 값에 접근하는 시간은 O(1)


'''
def solution(participant, completion):
    participant.sort()
    completion.sort()
    for i in range(len(completion)):
        if completion[i] != participant[i]:
            return participant[i]
    return participant[i+1]
'''
