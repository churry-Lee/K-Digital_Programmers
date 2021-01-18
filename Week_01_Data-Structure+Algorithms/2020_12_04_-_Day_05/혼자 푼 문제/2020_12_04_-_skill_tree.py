# 스킬트리(level 2)

'''
문제 설명
선행 스킬을 충족하는 스킬 트리 찾기
    + skill: 선행스킬 문자열
    + skill_trees: 유저들이 만든 스킬 트리
    + *return: skill_trees에서 skill의 순서를 만족하는 개수*

제한 사항
* 스킬은 알파벳 대문자로만 이루어져 있음
* skill: 길이 1 ~ 26, 중복 없음
* skill_trees: 길이 1 ~ 20, 원소는 스킬을 나타내는 문자열
'''

skill = 'A'
skill_trees = ['BACDE', 'CBADF', 'AECB', 'BDA', 'CDB']
# 'BACDE': B가 C보다 먼저 올 수 없음, X
# 'CBADF': 충족
# 'AECB': 충족
# 'BDA': C를 배우지 않은 상태에서 B를 배울 수 없음, X

# return 2

def solution(skill, skill_trees):
    answer = 0
    valid = []
    # 유효성 검사를 위해 skill_trees의 각 원소에서 skill에 포함된 문자만 남기고 나머지는 제거
    for skill_tree in skill_trees:
        skill_tree = list(skill_tree)
        temp = []
        for char in skill_tree:
            if char in skill:
                temp.append(char)
        valid.append(temp)
    # 스킬의 유효성 검사
    for i in valid:
        # 스킬의 길이가 0이 아니고, 처음 스킬이 skill의 처음과 같지 않은 경우 건너뜀
        if len(i) != 0 and i[0] != skill[0]:
            continue
        else:
            # 문자열 비교를 위해 스킬 리스트를 문자열로 변환
            i = ''.join(i)
            # 변환된 문자열 형태의 스킬이 skill내에 있으면 +1
            if i in skill:
                answer += 1
            # 없는 경우 건너뜀
            else:
                continue

    return answer

print(solution(skill, skill_trees))