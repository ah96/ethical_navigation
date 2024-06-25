import heapq
import numpy as np
import random
import math

class PathPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.width = len(grid[0])
        self.height = len(grid)

    def a_star(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(open_list, (priority, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def dijkstra(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    heapq.heappush(open_list, (new_cost, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def rrt(self, start, goal, max_iterations=1000, step_size=1):
        nodes = {start: None}
        for _ in range(max_iterations):
            rand_point = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
            nearest_node = min(nodes.keys(), key=lambda p: self.euclidean_distance(p, rand_point))
            new_node = self.step_from_to(nearest_node, rand_point, step_size)
            if self.is_valid(new_node):
                nodes[new_node] = nearest_node
                if self.euclidean_distance(new_node, goal) < step_size:
                    nodes[goal] = new_node
                    break

        return self.reconstruct_path(nodes, start, goal)

    def utilitarian_path(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(open_list, (priority, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def deontological_path(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                if self.is_restricted_area(next):
                    continue
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(open_list, (priority, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def virtue_ethics_path(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    if self.is_kind_action(current, next):
                        priority -= 1  # Reward for kindness
                    heapq.heappush(open_list, (priority, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def get_neighbors(self, current):
        x, y = current
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [neighbor for neighbor in neighbors if self.is_valid(neighbor)]

    def is_valid(self, point):
        x, y = point
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def euclidean_distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def step_from_to(self, a, b, step_size):
        if self.euclidean_distance(a, b) < step_size:
            return b
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
        return (int(a[0] + step_size * math.cos(theta)), int(a[1] + step_size * math.sin(theta)))

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def is_restricted_area(self, point):
        x, y = point
        restricted_areas = [(6, 6), (7, 6), (8, 6)]  # Example of restricted areas
        return (x, y) in restricted_areas

    def is_kind_action(self, current, next):
        # For example, avoid getting too close to restricted areas
        if self.is_restricted_area(next):
            return False
        return True
