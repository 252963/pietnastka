from collections import deque

MOVES = {
    'L': (0, -1),
    'R': (0, 1),
    'U': (-1, 0),
    'D': (1, 0),
}

rows, cols = 4, 4

start_board = [
    [1, 2, 4, 8],
    [5, 7, 3, 12],
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
            return path

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

    return None  # No solution


solution = bfs(start_board, goal_board)
print("Moves to solve:", solution)
