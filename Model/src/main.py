from collections import deque
import heapq
import time

MOVES = {
    'L': (0, -1),
    'R': (0, 1),
    'U': (-1, 0),
    'D': (1, 0),
}

rows, cols = 4, 4
MAX_DEPTH = 20

start_board = [
    [1, 2, 3, 4],
    [5, 7, 8, 12],
    [9, 6, 10, 0],
    [13, 14, 11, 15],
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

def dfs(state, zero_pos, goal, depth, path, visited):
    if depth > MAX_DEPTH:
        return None

    if state == goal:
        return path

    visited.add(state)
    zr, zc = zero_pos

    for move, (dr, dc) in MOVES.items():
        nr, nc = zr + dr, zc + dc
        if not is_valid(nr, nc):
            continue

        new_zero_idx = nr * cols + nc
        zero_idx = zr * cols + zc
        new_state = swap(state, zero_idx, new_zero_idx)

        if new_state not in visited:
            result = dfs(new_state, (nr, nc), goal, depth + 1, path + [move], visited)
            if result:
                return result

    visited.remove(state)
    return None

def manhattan_distance(board, goal):
    distance = 0
    goal_positions = {num: (r, c) for r, row in enumerate(goal) for c, num in enumerate(row)}

    for r in range(rows):
        for c in range(cols):
            val = board[r][c]
            if val != 0:
                gr, gc = goal_positions[val]
                distance += abs(r - gr) + abs(c - gc)
    return distance

def hamming_distance(board, goal):
    count = 0
    for r in range(rows):
        for c in range(cols):
            if board[r][c] != 0 and board[r][c] != goal[r][c]:
                count += 1
    return count

def astar(start_board, goal_board, heuristic='manhattan'):
    start = board_to_tuple(start_board)
    goal = board_to_tuple(goal_board)
    zero_pos = find_zero(start_board)

    heap = []
    g_score = {start: 0}

    if heuristic == 'manhattan':
        h = manhattan_distance(start_board, goal_board)
    elif heuristic == 'hamming':
        h = hamming_distance(start_board, goal_board)
    else:
        raise ValueError("Unsupported heuristic")

    heapq.heappush(heap, (h, 0, start, zero_pos, []))
    visited = set()

    while heap:
        f, g, state, (zr, zc), path = heapq.heappop(heap)

        if state == goal:
            return path, len(visited)

        if state in visited:
            continue
        visited.add(state)

        for move, (dr, dc) in MOVES.items():
            nr, nc = zr + dr, zc + dc
            if not is_valid(nr, nc):
                continue

            new_zero_idx = nr * cols + nc
            zero_idx = zr * cols + zc
            new_state = swap(state, zero_idx, new_zero_idx)

            if new_state in visited:
                continue

            new_g = g + 1
            if new_state not in g_score or new_g < g_score[new_state]:
                g_score[new_state] = new_g
                board_2d = [list(new_state[i * cols:(i + 1) * cols]) for i in range(rows)]

                if heuristic == 'manhattan':
                    h = manhattan_distance(board_2d, goal_board)
                elif heuristic == 'hamming':
                    h = hamming_distance(board_2d, goal_board)

                heapq.heappush(heap, (new_g + h, new_g, new_state, (nr, nc), path + [move]))

    return None, len(visited)

def solve(board, goal, method='bfs'):
    if method == 'dfs':
        start = board_to_tuple(board)
        goal_tuple = board_to_tuple(goal)
        zero_pos = find_zero(board)
        visited = set()
        path = dfs(start, zero_pos, goal_tuple, 0, [], visited)
        return path, len(visited)
    elif method == 'astar':
        return astar(board, goal, heuristic='manhattan')
    elif method == 'astar_hamming':
        return astar(board, goal, heuristic='hamming')
    else:
        return bfs(board, goal)

# Run all solvers
start_time = time.time()
solution_dfs, visited_dfs = solve(start_board, goal_board, method='dfs')
end_time = time.time()

start_time2 = time.time()
solution_bfs = solve(start_board, goal_board, method='bfs')
end_time2 = time.time()

start3 = time.time()
solution_astar = solve(start_board, goal_board, method='astar')
end_time3 = time.time()
start4 = time.time()
solution_astar_hamming = solve(start_board, goal_board, method='astar_hamming')
end_time4 = time.time()

print("DFS:", solution_dfs)
print("DFS path length:", len(solution_dfs) if solution_dfs else 0)
print("DFS visited states:", visited_dfs)
print("DFS time: %.4f seconds" % (end_time - start_time))

print("\nBFS:", solution_bfs[0])
print("BFS path length:", len(solution_bfs[0]) if solution_bfs[0] else 0)
print("BFS visited states:", solution_bfs[1])
print("BFS time: %.4f seconds" % (end_time2 - start_time2))

print("\nA* (Manhattan):", solution_astar[0])
print("A* Manhattan path length:", len(solution_astar[0]) if solution_astar[0] else 0)
print("A* Manhattan visited states:", solution_astar[1])
print("A* Manhatan time: ", end_time3 - start3 )

print("\nA* (Hamming):", solution_astar_hamming[0])
print("A* Hamming path length:", len(solution_astar_hamming[0]) if solution_astar_hamming[0] else 0)
print("A* Hamming visited states:", solution_astar_hamming[1])
print("A* Hamming time: ", end_time4 - start4 )
