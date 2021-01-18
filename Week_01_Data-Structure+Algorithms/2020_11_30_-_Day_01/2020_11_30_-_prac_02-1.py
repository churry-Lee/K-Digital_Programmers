L = [20, 37, 58, 72, 91]
x = 65

def solution(L, x):

    for i, v in enumerate(L):
        if v > x:
            L.insert(i, x)
            break
        elif L[-1] < x:
            L.append(x)

    return L

print(solution(L, x))