'''
카펫

Leo가 본 카펫에서 갈색 격자의 수 brown, 빨간색 격자의 수 red가 매개변수로 주어질 때 
카펫의 가로, 세로 크기를 순서대로 배열에 담아 return 하도록 solution 함수를 작성해주세요.

강의 노트의 해설을 봐도 이해가 잘 안감
'''

# brown = 10
# red = 2      # return [4, 3]

# brown = 8
# red = 1      # return [3, 3]

brown = 24
red = 24     # return [8, 6]

def solution(brown, red):
    # 중앙에 빨강색이 존재해야 하므로 높이는 최소 3(양쪽 brown 2 + 가운데 red 1)이므로, 초기값 3
    height = 3 
    # 너비는 = 전체 카펫 개수 // 높이
    width = (brown + red) // height
    
    while (width - 2) * (height - 2) != red:
        width -= 1
        height = (brown + red) // width

    return [width, height]

print(solution(brown, red))