# 피보나치 수열 반복문 풀이
def solution(x):
    
    f0 = 0
    f1 = 1 

    for _ in range(x): 
        f0, f1 = f1, f0 + f1 
        
    return f0

