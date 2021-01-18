# 일반적인 단일 연결 리스트 코드

# Node 생성자
class Node:
    def __init__(self, item):
        self.data = item
        self.next = None


class LinkedList:
    def __init__(self):
        self.nodeCount = 0
        self.head = None
        self.tail = None

    def __repr__(self):
        if self.nodeCount == 0:
            return 'LinkedList: empty'

        s = ''
        curr = self.head
        while curr is not None:
            s += repr(curr.data)
            if curr.next is not None:
                s += ' -> '
            curr = curr.next
        return s

    # 원하는 위치의 Node.data를 반환
    def getAt(self, pos):
        if pos < 1 or pos > self.nodeCount:
            return None

        i = 1
        curr = self.head
        while i < pos:
            curr = curr.next
            i += 1

        return curr

    # 원하는 위치에 Node 삽입
    def insertAt(self, pos, newNode):
        if pos < 1 or pos > self.nodeCount + 1:
            return False

        if pos == 1:
            newNode.next = self.head
            self.head = newNode

        else:
            if pos == self.nodeCount + 1:
                prev = self.tail
            else:
                prev = self.getAt(pos - 1)
            newNode.next = prev.next
            prev.next = newNode

        if pos == self.nodeCount + 1:
            self.tail = newNode

        self.nodeCount += 1
        return True

    # 연결 리스트의 길이 반환
    def getLength(self):
        return self.nodeCount

    # 연결 리스트 순회(or 연결 리스트 to 리스트)
    def traverse(self):
        result = []
        curr = self.head
        while curr is not None:
            result.append(curr.data)
            curr = curr.next
        return result

    # 두 개의 연결 리스트 병합
    def concat(self, L):
        self.tail.next = L.head
        if L.tail:
            self.tail = L.tail
        self.nodeCount += L.nodeCount

    # 원하는 위치의 Node 삭제
    def popAt(self, pos):   
        deleteNode = self.getAt(pos)
        if pos < 1 or pos > self.nodeCount:
            raise IndexError
        if pos == 1:
            if self.nodeCount == 1:
                self.head = None
                self.tail = None
            else:
                self.head = deleteNode.next
        else:
            prev = self.getAt(pos - 1)
            prev.next = deleteNode.next
            if pos == self.nodeCount:
                self.tail = prev
        self.nodeCount -= 1
        return deleteNode.data


head = Node(2)
node2 = Node(4)
node3 = Node(6)

L = LinkedList()

L.insertAt(1, head)
L.insertAt(2, node2)
L.insertAt(3, node3)

print(L.traverse())
print(L.getLength())
print('head', L.head.data)

L.popAt(1)
print(L.traverse())
print(L.getLength())
print('head', L.head.data)

node4 = Node(9)

L2 = LinkedList()

L2.insertAt(1, node4)

L.concat(L2)

print(L)
