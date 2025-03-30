from collections import deque
import heapq
import time

MOVES = {
    'L': (0, -1),  # lewo
    'R': (0, 1),   # prawo
    'U': (-1, 0),  # góra
    'D': (1, 0),   # dół
}

rows, cols = 4, 4

start_board = [
    [1, 2, 7, 0],
    [8, 9, 12, 10],
    [13, 3, 6, 4],
    [14, 4, 15, 5],
]

goal_board = [
    [1, 2, 3, 4],
    [5, 6, 7, 8],
    [9, 10, 11, 12],
    [13, 14, 15, 0],
]

def board_to_tuple(board):
    return tuple(num for row in board for num in row)

def find_zero(board):
    flat = board_to_tuple(board)
    idx = flat.index(0)
    return divmod(idx, cols)

def is_valid(r, c):
    return 0 <= r < rows and 0 <= c < cols

def swap(flat, i, j):
    lst = list(flat)
    lst[i], lst[j] = lst[j], lst[i]
    return tuple(lst)

def hamming_distance(state, goal):
    return sum(1 for i in range(len(state)) if state[i] != goal[i] and state[i] != 0)

def manhattan_distance(state, goal):
    total = 0
    for i, val in enumerate(state):
        if val == 0:
            continue
        goal_index = goal.index(val)
        x1, y1 = divmod(i, cols)
        x2, y2 = divmod(goal_index, cols)
        total += abs(x1 - x2) + abs(y1 - y2)
    return total

def bfs(start_board, goal_board):
    start = board_to_tuple(start_board)
    goal = board_to_tuple(goal_board)
    zero_pos = find_zero(start_board)

    queue = deque([(start, zero_pos, [])])
    visited = set()
    visited.add(start)

    while queue:
        state, (zr, zc), path = queue.popleft()

        if state == goal:
            return path, len(visited)

        for move, (dr, dc) in MOVES.items():
            nr, nc = zr + dr, zc + dc
            if not is_valid(nr, nc):
                continue

            new_zero_idx = nr * cols + nc
            zero_idx = zr * cols + zc
            new_state = swap(state, zero_idx, new_zero_idx)

            if new_state not in visited:
                visited.add(new_state)
                queue.append((new_state, (nr, nc), path + [move]))

    return None, len(visited)

def dfs(start_board, goal_board, max_depth=50):
    start = board_to_tuple(start_board)
    goal = board_to_tuple(goal_board)
    zero_pos = find_zero(start_board)

    stack = [(start, zero_pos, [])]
    visited = set()
    visited.add(start)

    while stack:
        state, (zr, zc), path = stack.pop()

        if state == goal:
            return path, len(visited)

        if len(path) >= max_depth:
            continue

        for move, (dr, dc) in MOVES.items():
            nr, nc = zr + dr, zc + dc
            if not is_valid(nr, nc):
                continue

            zero_idx = zr * cols + zc
            new_zero_idx = nr * cols + nc
            new_state = swap(state, zero_idx, new_zero_idx)

            if new_state not in visited:
                visited.add(new_state)
                stack.append((new_state, (nr, nc), path + [move]))

    return None, len(visited)

def astar(start_board, goal_board, heuristic_fn):
    start = board_to_tuple(start_board)
    goal = board_to_tuple(goal_board)
    zero_pos = find_zero(start_board)

    heap = [(heuristic_fn(start, goal), 0, start, zero_pos, [])]
    visited = set()
    visited.add(start)

    while heap:
        est_total_cost, cost_so_far, state, (zr, zc), path = heapq.heappop(heap)

        if state == goal:
            return path, len(visited)

        for move, (dr, dc) in MOVES.items():
            nr, nc = zr + dr, zc + dc
            if not is_valid(nr, nc):
                continue

            zero_idx = zr * cols + zc
            new_zero_idx = nr * cols + nc
            new_state = swap(state, zero_idx, new_zero_idx)

            if new_state not in visited:
                visited.add(new_state)
                new_cost = cost_so_far + 1
                est_total = new_cost + heuristic_fn(new_state, goal)
                heapq.heappush(heap, (est_total, new_cost, new_state, (nr, nc), path + [move]))

    return None, len(visited)

def solve(board, goal, algorithm='bfs', heuristic='manhattan'):
    if algorithm == 'bfs':
        return bfs(board, goal)
    elif algorithm == 'dfs':
        return dfs(board, goal, max_depth=80)
    elif algorithm == 'astar':
        if heuristic == 'manhattan':
            return astar(board, goal, manhattan_distance)
        elif heuristic == 'hamming':
            return astar(board, goal, hamming_distance)
        else:
            raise ValueError("Nieznana heurystyka: wybierz 'manhattan' lub 'hamming'")
    else:
        raise ValueError("Nieznany algorytm: wybierz 'bfs', 'dfs' lub 'astar'")

start_time = time.time()
solution, visited_count = solve(start_board, goal_board, algorithm='astar', heuristic='manhattan')
end_time = time.time()

print("Rozwiązanie:", solution)
print("Długość ścieżki:", len(solution) if solution else 0)
print("Liczba odwiedzonych stanów:", visited_count)
print("Czas wykonania: %.4f sekundy" % (end_time - start_time))
