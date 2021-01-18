# 완료하지 못 한 선수

participant = ['leo', 'kiki', 'eden', 'leo']
completion = ['eden', 'kiki']


def solution(participant, completion):
    player = {}
    for name in participant:
        if name not in player:
            player[name] = player.get(name, 1)
        elif name in player:
            player[name] += 1

    for name in completion:
        if name in player:
            player[name] -= 1

    for answer in player:
        if player[answer] > 0:
            return answer


print(solution(participant, completion))
