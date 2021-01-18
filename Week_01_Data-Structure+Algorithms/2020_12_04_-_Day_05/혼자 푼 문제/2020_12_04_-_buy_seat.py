# 좌석 구매

'''
문제 설명
공연 관람을 위한 100,000 크기의 격자 모양 좌석이 있습니다.
K명의 사람이 좌석이 비어 있으면, 공연을 관람하고, 그렇지 않으면 공연을 안 봅니다.
    + seat: 구매하려는 좌석의 좌표가 순서대로 담겨있는 배열
    + *return: 표를 구매하는데 성공한 사람의 수*

제한 사항
* 관람객 수: 1 ~ 100,000
* seat의 가장 앞에서 부터 순서대로 구매
* seat의 원소는 좌표 이며, [가로 좌표, 세로 좌표]
* 좌표 범위: 각각 1 ~ 100,000
'''

seat = [[1, 1], [2, 2], [3, 3]]     # return 3
seat = [[1, 1], [2, 1], [1, 2], [3, 4], [2, 1], [2, 1]]     # return 4

'''
def solution(seat):
    purchase = []

    for i in seat:
        if i not in purchase:
            purchase.append(i)


    return len(purchase)

*반복문을 이용한 풀이 - 시간 초과 *
'''



def solution(seat):
    # map 함수로 원소를 문자열로 변환
    # set 함수를 이용해 중복 원소 제거
    seat = set(list(map(str, seat)))

    return len(seat)

print(solution(seat))