from heapq import heappush, heappop


class PriorityQueue:

    def __init__(self):
        self.heap = []

    def add(self, value, priority=0):
        heappush(self.heap, (priority, value))

    def pop(self):
        priority, value = heappop(self.heap)
        return value

    def __len__(self):
        return len(self.heap)
