# 힙(heap) 대표 문제

'''
문제 설명
- 더 맵게(level 2) -

모든 음식의 스코빌 지수를 K 이상으로 만들기
섞은 지수 = 가장 맵지 않은 지수 + (두 번째로 맵지 않은 지수 * 2)
모든 음식의 지수가 K 이상이 될 때까지 반복
scoville: 음식의 스코빌 지수를 담은 배열
K: 원하는 스코빌 지수
return: 모든 음식의 스코빌 지수가 K 이상으로 만들기 위한 최소 횟수

제한 사항
scoville: 2 ~ 1,000,000
K: 0 ~ 1,000,000,000
scoville 원소: 0 ~ 1,000,000
모든 음식의 스코빌 지수를 K 이상으로 만들 수 없는 경우에는 -1 return
'''
'''
힙(heaps)
    - max heap
    - min heap

최소/최대 원소를 빠르게 꺼낼 수 있는 자료구조
- 힙 구성(heapify)   O(NlogN)
- 삽입(insert)       O(logN)
- 삭제(remove)       O(logN)

힙의 구현
    - 완전 이진 트리로 구성
      => 배열을 이용해서 구현 가능

힙의응용
     - 정렬(heapsort)
     - 우선 순위 큐(priority queue)

힙 모듈 사용
import heapq

heapq.heapify(L)      리스트 L로부터 min heap 구성
m = heapq.heqppop(L)  min heap L 에서 최소값 삭제(반환)
heapq.heappush(L, x)  min heap L 에 원소 x 삽입

heapq._heapify_max(L) 리스트 L로부터 max heap 구성
'''

import heapq

scoville = [1, 2, 3, 9, 10, 12]
K = 7     # return 2

def solution(scoville, K):
    answer = 0
    # 스코빌 지수 리스트로 힙 구성
    heapq.heapify(scoville)
    # 무한 루프 형태로 반복문 작성
    while True:
        # 가장 맵지 않은 음식 
        min_scoville_1 = heapq.heappop(scoville)
        # 무한 루프 종료 조건
        # 최소 지수가 K 이상일 경우
        if min_scoville_1 >= K:
            break
        # 리스트의 길이가 0 일 경우
        elif len(scoville) == 0:
            answer = -1
            break
        # 두번째로 덜 매운 음식
        min_scoville_2 = heapq.heappop(scoville)
        # 음식 섞기
        new_scoville = min_scoville_1 + (min_scoville_2 * 2)
        # 섞은 음식을 heap에 삽입
        heapq.heappush(scoville, new_scoville)
        # 섞은 횟수 갱신
        answer += 1

    return answer

print(solution(scoville, K))

# 시간 복잡도 O(NlogN)
# 리스트와 정렬을 이용할 경우 O(n**2)이 되어 효율성 테스트에서 떨어짐

'''
# 리스트와 정렬로 이전에 작성한 코드
# 정확성 테스트는 모두 통과 했으나,
# 효율성에서 모두 시간초과 발생

def solution(scoville, K):
    answer = 0
    new_scoville = []

    while scoville:
        scoville.sort()
        if scoville[0] < K:
            if len(scoville) > 1:
                scoville.append(scoville.pop(0) + (scoville.pop(0) * 2))
                answer += 1
            else:
                return -1
        else:
            new_scoville.append(scoville.pop(0))

    return answer
'''