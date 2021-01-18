# 깊이/너비 우선 탐색(DFS/BFS) 대표 문제

'''
문제 설명
- 여행 경로(level 3) - 

주어진 항공권을 모두 이용하여 여행경로를 짜려고 합니다. 항상 ICN 공항에서 출발합니다.
tickets: 항공권 정보가 담긴 2차원 배열
return: 방문하는 항공 경로

제한 사항
모든 공항은 알파벳 대문자 3글자로 이루어집니다.
주어진 공항 수는 3개 이상 10,000개 이하입니다.
tickets의 각 행 [a, b]는 a 공항에서 b 공항으로 가는 항공권이 있다는 의미입니다.
주어진 항공권은 모두 사용해야 합니다.
만일 가능한 경로가 2개 이상일 경우 알파벳 순서가 앞서는 경로를 return 합니다.
모든 도시를 방문할 수 없는 경우는 주어지지 않습니다.
'''

# tickets = [['ICN', 'JFK'], ['HND', 'IAD'], ['JFK', 'HND']]       # return ['ICN', 'JFK', 'HND', 'IAD']
tickets = [['ICN', 'SFO'], ['ICN', 'ATL'], ['SFO', 'ATL'], ['ATL', 'ICN'], ['ATL', 'SFO']]    # return ['ICN', 'ATL', 'ICN', 'SFO', 'ATL', 'SFO']

# 한 붓 그리기, DFS 응용
# dictionary의 key에 출발 지점 저장
# 알파벳 순으로 정렬하기 위해 리스트 이용

def solution(tickets):
    # key: 출발지, val: 도착지 로 dictionary 작성
    routes = {}
    for i in tickets:
        routes[i[0]] = routes.get(i[0], []) + [i[1]]
    # 도착지를 알파벳 역순으로 정렬, 뒤에서 부터 pop 하는게 더 효율이 좋으므로, 역순으로 정렬
    for i in routes:
        routes[i].sort(reverse=True)

    stack = ['ICN']
    path = []
    while stack:
        top = stack[-1]
        if top not in routes or len(routes[top]) == 0:
            path.append(stack.pop())
        else:
            stack.append(routes[top][-1])
            routes[top].pop()

    return path[::-1]

print(solution(tickets))

# 알파벳 순으로 정렬하는 for 문이 전체 복잡도를 지배
# 따라서, O(NlogN)의 시간 복잡도를 가짐