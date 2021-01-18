# 이진 트리의 넓이 우선 순회 구현

class ArrayQueue:

    def __init__(self):
        self.data = []

    def size(self):
        return len(self.data)

    def isEmpty(self):
        return self.size() == 0

    def enqueue(self, item):
        self.data.append(item)

    def dequeue(self):
        return self.data.pop(0)

    def peek(self):
        return self.data[0]


class Node:

    def __init__(self, item):
        self.data = item
        self.left = None
        self.right = None


class BinaryTree:

    def __init__(self, r):
        self.root = r


    def bft(self):
        # 빈 리스트 생성
        traversal = []
        # 빈 큐 생성
        visitQueue = ArrayQueue()
        # 루트 노드가 있다면(빈 트리가 아니라면)
        if self.root:
            # 루트 노드를 큐에 추가
            visitQueue.enqueue(self.root)
        # 큐가 비어 있지 않은 동안에
        while not visitQueue.isEmpty():
            # 큐에 있는 원소를 node에 저장
            node = visitQueue.dequeue()
            # 저장된 노드를 traversal 리스트에 저장(node 방문)
            traversal.append(node.data)
            # 왼쪽 자식이 있으면
            if node.left:
                # 왼쪽 자식 노드를 큐에 저장
                visitQueue.enqueue(node.left)
            # 오른쪽 자식이 있으면
            if node.right:
                # 오른쪽 자식 노드를 큐에 저장
                visitQueue.enqueue(node.right)

        return traversal

def solution(x):
    return 0