'''
기능 개발

프로그래머스 팀에서는 기능 개선 작업을 수행 중입니다. 
각 기능은 진도가 100%일 때 서비스에 반영할 수 있습니다.
또, 각 기능의 개발속도는 모두 다르기 때문에 뒤에 있는 기능이 앞에 있는 기능보다 먼저 개발될 수 있고, 
이때 뒤에 있는 기능은 앞에 있는 기능이 배포될 때 함께 배포됩니다.
먼저 배포되어야 하는 순서대로 작업의 진도가 적힌 정수 배열 progresses와 각 작업의 개발 속도가 적힌 정수 배열 speeds가 주어질 때 
각 배포마다 몇 개의 기능이 배포되는지를 return 하도록 solution 함수를 완성하세요.
'''

# 큐 문제(선입선출)
# 작업이 완료되는 앞에서부터 순서대로 배포되는 코드 작성
# 앞의 작업이 완료되어야지 작업이 배포 될 수 있음
# 작업은 작업속도에 따라서 진행이 됨

progresses = [93, 30, 55]	
speeds = [1, 30, 5]	       # return [2, 1]

# progresses = [95, 90, 99, 99, 80, 99]	
# speeds = [1, 1, 1, 1, 1, 1]            # return [1, 3, 2]

def solution(progresses, speeds):
    answer = []
    
    # progresses 내의 원소가 존재하는 동안 반복문 실행
    while progresses:
        # 완료되는 작업의 수를 저장할 변수 및 반복문이 한 사이클 돌 때 마다 0으로 초기화
        count = 0
        # 첫번째 작업이 100 보다 작을 경우 동안 반복문 실행
        while progresses[0] < 100:
            # 작업 리스트로 반복문 실행
            for i in range(len(progresses)):
                # i 번째 작업이 100% 이상이 되면, 일단 건너뜀
                if progresses[i] >= 100:
                    continue
                # 100%가 안되었을 경우, 작업 속도를 더해주어 작업 진행 현황 갱신
                else:
                    progress = progresses[i] + speeds[i]
                    progresses[i] = progress

        # progresses에 원소가 존재하고, 첫번째 작업이 100% 이상일 경우 반복문 실행
        while progresses and progresses[0] >= 100:
            # 완료된 작업의 수에 1을 더해주고,
            count += 1
            # 100% 이상이 된 원소를 앞에서 부터 제거
            progresses.pop(0)
            speeds.pop(0)
        # 한 사이클 완료된 작업의 수 answer 리스트에 추가
        answer.append(count)
    
    return answer

print(solution(progresses, speeds))