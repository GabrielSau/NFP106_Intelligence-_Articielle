import pygame
import sys
import random
from copy import deepcopy

# =========================
# CONFIGURATION
# =========================
WIDTH = 540
HEIGHT = 600
GRID_SIZE = 9
CELL_SIZE = WIDTH // GRID_SIZE

WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
GRAY = (180, 180, 180)
BLUE = (80, 120, 255)
GREEN = (60, 180, 75)
RED = (220, 50, 50)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Sudoku - Heuristique LCV")
font = pygame.font.SysFont("arial", 32)
small_font = pygame.font.SysFont("arial", 22)

# =========================
# SUDOKU INITIAL
# =========================
initial_board = [
    [5, 3, 0, 0, 7, 0, 0, 0, 0],
    [6, 0, 0, 1, 9, 5, 0, 0, 0],
    [0, 9, 8, 0, 0, 0, 0, 6, 0],

    [8, 0, 0, 0, 6, 0, 0, 0, 3],
    [4, 0, 0, 8, 0, 3, 0, 0, 1],
    [7, 0, 0, 0, 2, 0, 0, 0, 6],

    [0, 6, 0, 0, 0, 0, 2, 8, 0],
    [0, 0, 0, 4, 1, 9, 0, 0, 5],
    [0, 0, 0, 0, 8, 0, 0, 7, 9]
]

board = deepcopy(initial_board)

# =========================
# FONCTIONS SUDOKU
# =========================

def draw_board():
    screen.fill(WHITE)

    # Cases
    for row in range(9):
        for col in range(9):
            x = col * CELL_SIZE
            y = row * CELL_SIZE

            pygame.draw.rect(screen, WHITE, (x, y, CELL_SIZE, CELL_SIZE))

            value = board[row][col]
            if value != 0:
                color = BLACK if initial_board[row][col] != 0 else BLUE
                text = font.render(str(value), True, color)
                screen.blit(
                    text,
                    (
                        x + CELL_SIZE // 2 - text.get_width() // 2,
                        y + CELL_SIZE // 2 - text.get_height() // 2,
                    ),
                )

    # Lignes
    for i in range(10):
        thickness = 4 if i % 3 == 0 else 1
        pygame.draw.line(screen, BLACK, (0, i * CELL_SIZE),
                         (WIDTH, i * CELL_SIZE), thickness)
        pygame.draw.line(screen, BLACK, (i * CELL_SIZE, 0),
                         (i * CELL_SIZE, WIDTH), thickness)

    # Texte du bas
    info = small_font.render(
        "ESPACE = Résoudre avec LCV | R = Reset",
        True,
        GREEN
    )
    screen.blit(info, (20, 555))


def is_valid(board, row, col, num):
    # Ligne
    if num in board[row]:
        return False

    # Colonne
    for r in range(9):
        if board[r][col] == num:
            return False

    # Bloc 3x3
    start_row = (row // 3) * 3
    start_col = (col // 3) * 3

    for r in range(start_row, start_row + 3):
        for c in range(start_col, start_col + 3):
            if board[r][c] == num:
                return False

    return True


def get_domain(board, row, col):
    domain = []

    for num in range(1, 10):
        if is_valid(board, row, col, num):
            domain.append(num)

    return domain


def find_empty(board):
    for row in range(9):
        for col in range(9):
            if board[row][col] == 0:
                return row, col
    return None


# =========================
# HEURISTIQUE LCV
# =========================
# Least Constraining Value :
# Choisit la valeur qui réduit
# le moins les domaines voisins.
# =========================

def count_constraints(board, row, col, value):
    """
    Compte combien de domaines voisins
    seraient réduits si on place 'value'
    """

    impact = 0

    # Vérifie les voisins
    for r in range(9):
        for c in range(9):

            if board[r][c] == 0:

                # Même ligne, colonne ou bloc
                same_row = r == row
                same_col = c == col
                same_box = (
                    r // 3 == row // 3 and
                    c // 3 == col // 3
                )

                if same_row or same_col or same_box:
                    domain = get_domain(board, r, c)

                    if value in domain:
                        impact += 1

    return impact


def order_lcv(board, row, col):
    """
    Trie les valeurs selon LCV
    La meilleure valeur est celle
    qui impacte le moins les autres.
    """

    domain = get_domain(board, row, col)

    scored = []

    for value in domain:
        impact = count_constraints(board, row, col, value)
        scored.append((impact, value))

    scored.sort()

    return [value for impact, value in scored]


# =========================
# SOLVEUR BACKTRACKING + LCV
# =========================

def solve(board):
    pygame.event.pump()

    empty = find_empty(board)

    if not empty:
        return True

    row, col = empty

    # Valeurs triées par LCV
    for value in order_lcv(board, row, col):

        if is_valid(board, row, col, value):

            board[row][col] = value

            draw_board()

            # Animation
            pygame.draw.rect(
                screen,
                GREEN,
                (col * CELL_SIZE, row * CELL_SIZE,
                 CELL_SIZE, CELL_SIZE),
                4
            )

            pygame.display.update()
            pygame.time.delay(40)

            if solve(board):
                return True

            # Backtrack
            board[row][col] = 0

            draw_board()

            pygame.draw.rect(
                screen,
                RED,
                (col * CELL_SIZE, row * CELL_SIZE,
                 CELL_SIZE, CELL_SIZE),
                4
            )

            pygame.display.update()
            pygame.time.delay(40)

    return False


# =========================
# BOUCLE PRINCIPALE
# =========================

running = True

while running:

    draw_board()
    pygame.display.update()

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            # Résolution
            if event.key == pygame.K_SPACE:
                solve(board)

            # Reset
            if event.key == pygame.K_r:
                board = deepcopy(initial_board)

pygame.quit()
sys.exit()