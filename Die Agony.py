import numpy as np

BOARD = np.array([[57, 33, 132, 268, 492, 732],
                  [81, 123, 240, 443, 353, 508],
                  [186, 42, 195, 704, 452, 228],
                  [-7, 2, 357, 452, 317, 395],
                  [5, 23, -4, 592, 445, 620],
                  [0, 77, 32, 403, 337, 452]], dtype=int)

CELLS_TO_BE_SEEN = np.ones(BOARD.shape, dtype=int)

CELLS_TO_BE_SEEN[5, 0] = 0
CELLS_TO_BE_SEEN[0, 5] = 0

CELL = np.array([5, 0])
DIE = np.array([np.nan] * 6)  # TOP 0, BOTTOM 1, FRONT 2, BACK 3, RIGHT 4, LEFT 5

CELLS_SEEN = [CELL]

SCORE = 0
N = 0

MOVES = ([1, 0], [-1, 0], [0, 1], [0, -1])

FINAL_DIE = np.nan


def move_die(die, move):
    die = np.array(die)

    if move[1] == 0:
        if move[0] == -1:
            tb = die[[3, 2]]
            fb = die[[0, 1]]
            die[[0, 1]] = tb
            die[[2, 3]] = fb
        elif move[0] == 1:
            tb = die[[2, 3]]
            fb = die[[1, 0]]
            die[[0, 1]] = tb
            die[[2, 3]] = fb
    elif move[0] == 0:
        if move[1] == -1:
            tb = die[[4, 5]]
            rl = die[[1, 0]]
            die[[0, 1]] = tb
            die[[4, 5]] = rl
        elif move[1] == 1:
            tb = die[[5, 4]]
            rl = die[[0, 1]]
            die[[0, 1]] = tb
            die[[4, 5]] = rl

    return die


def check_validity(n, score, die, cell_value):
    if cell_value == n * die + score:
        return True
    else:
        return False


def make_move(die, cell, n, score):
    global BOARD
    global MOVES
    global CELLS_TO_BE_SEEN
    global FINAL_DIE
    global N
    global CELL
    global CELLS_SEEN

    start_die = np.array(die)
    start_cell = np.array(cell)
    start_score = score
    n += 1
    for move in MOVES:
        cell = start_cell + move
        if min(cell) < 0 or max(cell) > 5:
            continue
        die = move_die(start_die, move)

        if np.isnan(die[0]):
            new_face = (BOARD[cell[0], cell[1]] - start_score) / n
            if new_face == int(new_face):
                die[0] = new_face
            else:
                continue

        if check_validity(n, start_score, die[0], BOARD[cell[0], cell[1]]):
            score = start_score + n * die[0]
            if score == 732 and cell[0] == 0 and cell[1] == 5:
                N = n
                FINAL_DIE = die
                CELL = cell
                return True
            CELLS_TO_BE_SEEN[cell[0], cell[1]] = 0
            CELLS_SEEN.append(cell)
            if make_move(die, cell, n, score):
                return True
            CELLS_TO_BE_SEEN[cell[0], cell[1]] = 1
            CELLS_SEEN.pop(-1)


make_move(DIE, CELL, N, SCORE)
print('THE SOLUTION IS:', (CELLS_TO_BE_SEEN*BOARD).sum())
