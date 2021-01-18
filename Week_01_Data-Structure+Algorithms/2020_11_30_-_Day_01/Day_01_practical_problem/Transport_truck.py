'''
운송 트럭

트럭은 최대 무게가 한정되어있습니다. 
직원은 트럭에 상품을 순서대로 실으며, 상품을 실을 수 없는 트럭은 바로 목적지로 출발합니다. 
이때 우리는 모든 상품을 운반하는데 필요한 트럭은 최소 몇 대인지 구하려 합니다.

트럭의 허용 무게 max_weight와 상품의 스펙을 담은 배열 specs, 운반할 상품의 이름이 순서대로 들은 배열 names가 주어집니다. 
이때, 상품을 순서대로 운반하기 위해 필요한 트럭 수를 리턴하는 함수, soution을 완성하세요.
'''

max_weight = 300
specs = [['toy','70'], ['snack', '200']]	
names = ['toy', 'snack', 'snack']          # return 2

# max_weight = 200
# specs = [['toy','70'], ['snack', '200']]	
# names = ['toy', 'snack', 'toy']            # return 3

def solution(max_weight, specs, names):
    # 트럭은 최소 한 대 이상이 움직이기 때문에 초기값 1
    answer = 1

    # 상품의 specs 를 dictionary 로 변환 key = '상품 이름', value = '상품 무게'
    specs = dict(specs)
    # 상품의 무게 합을 저장해 줄 변수
    weight = 0

    # 상품의 이름들로 반복문 실행
    for name in names:
        # 상품의 무게가 문자열로 되어있기 때문에, 정수형으로 변환,
        # 상품에 해당하는 무게를 weight에 더해줌
        weight += int(specs[name])

        # 더해준 무게가 최대 무게보다 클 경우
        if weight > max_weight:
            # 기존 트럭에 더 이상 실을 수 없으므로, 한 대 추가
            answer += 1
            # 추가된 트럭에 실린 상품의 무게로 weight 초기화
            weight = int(specs[name])

    return answer

print(solution(max_weight, specs, names))