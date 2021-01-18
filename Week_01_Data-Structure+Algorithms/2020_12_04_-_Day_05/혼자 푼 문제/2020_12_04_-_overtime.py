# 야근 지수

# 배상 비용 최소화(min_cost.py) 문제와 동일
# but, 야근 지수는 시간이 더 빡빡함
# 배상 비용 최소화 코드로 실행시 효율성에서 시간 초과 발생
# heapq 모듈로 max_heap 구성이 안됨,,, 이상함, 답이 안나옴
# heapq 모듈을 사용할 수 있게 하기 위해 각 원소에 -1 을 곱함
# 어차피 최종 결과는 제곱의 합이라 답에 영향 없음

works = [2, 1, 2]
n = 10

import heapq

def solution(n, works):
    answer = 0

    # 각 원소의 총합이 작업량 보다 적으면 모든 일을 처리 할 수 있기 때문에
    # return 0
    if n > sum(works):
        return answer

    # heap 구성을 위해 원소 * -1
    works = [-i for i in works]
    # 힙 구성
    heapq.heapify(works)

    while n > 0:
        # 가장 작은 값 pop(), (-1 곱하기 전 가장 큰 값)
        min_works = heapq.heappop(works)
        # 작업량 빼준 후 다시 푸시
        heapq.heappush(works, min_works + 1)
        n -= 1

    # 남은 작업량(야근 피로도) 계산
    for i in works:
        answer += (i**2)

    return answer


print(solution(n, works))