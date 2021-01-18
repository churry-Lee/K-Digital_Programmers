'''
나머지 한 점

직사각형을 만드는 데 필요한 4개의 점 중 3개의 좌표가 주어질 때, 나머지 한 점의 좌표를 구하려고 합니다. 
점 3개의 좌표가 들어있는 배열 v가 매개변수로 주어질 때, 
직사각형을 만드는 데 필요한 나머지 한 점의 좌표를 return 하도록 solution 함수를 완성해주세요.
'''

v = [[1, 4], [3, 4], [3, 10]]   # return [1, 10]
# v = [[1, 1], [2, 2], [1, 2]]    # return [2, 1]


def solution(v):
    answer = []

    vx = []   # 찾고자 하는 점의 x축 좌표를 저장할 변수
    vy = []   # 찾고자 하는 점의 y축 좌표를 저장할 변수
    
    # 반복문을 통해 좌표 정보 읽어옴
    for i in v:
        # 읽어온 좌표의 x축 좌표가 변수 v1에 없을 경우
        if i[0] not in vx:
            # v1에 추가
            vx.append(i[0])
        # 동일한 좌표가 존재할 경우 제거
        else:
            vx.remove(i[0])

        # 읽어온 좌표의 y축 좌표가 변수 v2에 없을 경우
        if i[1] not in vy:
            # v2에 추가
            vy.append(i[1])
        # 동일한 좌표가 존재할 경우 제거
        else:
            vy.remove(i[1])
    # v1과 v2 리스트를 합하여 answer에 저장
    answer = vx + vy

    return answer

print(solution(v))