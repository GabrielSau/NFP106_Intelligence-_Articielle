import pygame
import random

# --- Configuration ---
N = 8
CELL_SIZE = 80
WINDOW_SIZE = N * CELL_SIZE
FPS = 10

# Couleurs
WHITE = (240, 240, 240)
GREY = (200, 200, 200)
BLACK = (50, 50, 50)
RED = (200, 50, 50)
GOLD = (255, 215, 0)
GREEN = (0, 180, 0)
BLUE = (50, 120, 255)

class MinConflictsQueens:
    def __init__(self, n):
        self.n = n
        self.state = [random.randint(0, n - 1) for _ in range(n)]

    def get_conflicts(self, col, row):
        count = 0
        for i in range(self.n):
            if i == col:
                continue

            if (
                self.state[i] == row
                or abs(self.state[i] - row) == abs(i - col)
            ):
                count += 1

        return count

    def get_all_conflicts(self):
        conflicted = []

        for col in range(self.n):
            if self.get_conflicts(col, self.state[col]) > 0:
                conflicted.append(col)

        return conflicted

    def step(self):

        conflicted_cols = self.get_all_conflicts()

        if not conflicted_cols:
            return True

        # --- Trouver la reine avec le plus de conflits ---
        max_conflicts = -1
        worst_cols = []

        for col in conflicted_cols:

            conflicts = self.get_conflicts(col, self.state[col])

            if conflicts > max_conflicts:
                max_conflicts = conflicts
                worst_cols = [col]

            elif conflicts == max_conflicts:
                worst_cols.append(col)

        # Choisir une des pires reines aléatoirement
        col = random.choice(worst_cols)

        # --- Chercher la meilleure ligne ---
        min_conflicts = self.n + 1
        best_rows = []

        for row in range(self.n):

            c = self.get_conflicts(col, row)

            if c < min_conflicts:
                min_conflicts = c
                best_rows = [row]

            elif c == min_conflicts:
                best_rows.append(row)

        # Déplacer la reine
        self.state[col] = random.choice(best_rows)

        return False


def draw_board(screen, game, conflicted_cols):
    for row in range(N):
        for col in range(N):

            color = WHITE if (row + col) % 2 == 0 else GREY

            pygame.draw.rect(
                screen,
                color,
                (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            )
                # --- Dessiner les conflits ---
            for col1 in range(N):
                row1 = game.state[col1]

                for col2 in range(col1 + 1, N):
                    row2 = game.state[col2]

                    conflit = (
                        row1 == row2
                        or abs(row1 - row2) == abs(col1 - col2)
                    )

                    if conflit:

                        x1 = col1 * CELL_SIZE + CELL_SIZE // 2
                        y1 = row1 * CELL_SIZE + CELL_SIZE // 2

                        x2 = col2 * CELL_SIZE + CELL_SIZE // 2
                        y2 = row2 * CELL_SIZE + CELL_SIZE // 2

                        pygame.draw.line(
                            screen,
                            RED,
                            (x1, y1),
                            (x2, y2),
                            4
                        )

            # Dessiner les reines
            if game.state[col] == row:

                queen_color = (
                    RED if col in conflicted_cols else GOLD
                )

                pygame.draw.circle(
                    screen,
                    queen_color,
                    (
                        col * CELL_SIZE + CELL_SIZE // 2,
                        row * CELL_SIZE + CELL_SIZE // 2
                    ),
                    CELL_SIZE // 3
                )


def draw_ui(screen, font, auto_mode, solved):
    mode_text = "MODE AUTO" if auto_mode else "MODE PAS A PAS"

    color = GREEN if auto_mode else BLUE

    text = font.render(mode_text, True, color)

    screen.blit(text, (10, 10))

    controls = font.render(
        "SPACE = Etape | A = Auto | P = Pause | R = Reset",
        True,
        BLACK
    )

    screen.blit(controls, (10, WINDOW_SIZE - 30))

    if solved:
        solved_text = font.render("RESOLU !", True, GREEN)
        screen.blit(
            solved_text,
            (WINDOW_SIZE // 2 - 70, WINDOW_SIZE // 2 - 20)
        )


def main():
    pygame.init()

    screen = pygame.display.set_mode(
        (WINDOW_SIZE, WINDOW_SIZE)
    )

    pygame.display.set_caption(
        "Min-Conflicts : N-Reines"
    )

    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 24, bold=True)

    game = MinConflictsQueens(N)

    solved = False
    running = True

    # False = pas à pas
    # True = automatique
    auto_mode = False

    while running:

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:

                # Une seule étape
                if event.key == pygame.K_SPACE:
                    if not solved:
                        solved = game.step()

                # Mode automatique
                elif event.key == pygame.K_a:
                    auto_mode = True

                # Pause
                elif event.key == pygame.K_p:
                    auto_mode = False

                # Reset
                elif event.key == pygame.K_r:
                    game = MinConflictsQueens(N)
                    solved = False
                    auto_mode = False

        # Exécution automatique
        if auto_mode and not solved:
            solved = game.step()

        conflicted = game.get_all_conflicts()

        screen.fill(BLACK)

        draw_board(screen, game, conflicted)

        draw_ui(screen, font, auto_mode, solved)

        pygame.display.flip()

        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()