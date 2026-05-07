import pygame
import sys
import time
import random
import copy

# ─── Palette ────────────────────────────────────────────────────────────────
BG          = (15,  15,  25)
GRID_BG     = (22,  22,  38)
CELL_HOVER  = (35,  35,  58)
LINE_THIN   = (50,  50,  80)
LINE_THICK  = (100, 100, 160)
TEXT_GIVEN  = (230, 230, 255)
TEXT_USER   = (100, 200, 255)
TEXT_SOLVE  = (80,  220, 140)
TEXT_BACK   = (255, 160,  60)   # backtrack colour (orange)
TEXT_ERROR  = (255,  80,  80)
SELECTED_BG = (40,  60, 120)
ACCENT      = (100, 150, 255)
BTN_NORMAL  = (30,  30,  55)
BTN_HOVER   = (50,  60, 100)
BTN_ACTIVE  = (60,  40, 100)
BTN_TEXT    = (200, 210, 255)
SHADOW      = (8,    8,  18)
STEP_PANEL  = (18,  18,  35)
STEP_BORDER = (70,  70, 120)

# ─── Sudoku logic ────────────────────────────────────────────────────────────

def is_valid(board, row, col, num):
    if num in board[row]:
        return False
    if num in [board[r][col] for r in range(9)]:
        return False
    br, bc = (row // 3) * 3, (col // 3) * 3
    for r in range(br, br + 3):
        for c in range(bc, bc + 3):
            if board[r][c] == num:
                return False
    return True


def solve(board, steps=None):
    for row in range(9):
        for col in range(9):
            if board[row][col] == 0:

                # On teste TOUS les nombres de 1 à 9
                for num in range(1, 10):

                    # ← Nouvelle étape : montrer le test
                    if steps is not None:
                        steps.append((row, col, num, "try"))

                    if is_valid(board, row, col, num):

                        # Placement valide
                        board[row][col] = num

                        if steps is not None:
                            steps.append((row, col, num, "place"))

                        if solve(board, steps):
                            return True

                        # Backtracking
                        board[row][col] = 0

                        if steps is not None:
                            steps.append((row, col, 0, "backtrack"))

                return False
    return True


def generate_full_board():
    board = [[0]*9 for _ in range(9)]
    nums = list(range(1, 10))
    for box in range(9):
        r, c = (box // 3) * 3, (box % 3) * 3
        random.shuffle(nums)
        k = 0
        for i in range(r, r + 3):
            for j in range(c, c + 3):
                board[i][j] = nums[k]; k += 1
    solve(board)
    return board


def make_puzzle(full, clues=30):
    board = copy.deepcopy(full)
    cells = [(r, c) for r in range(9) for c in range(9)]
    random.shuffle(cells)
    removed = 0
    for r, c in cells:
        if removed >= 81 - clues:
            break
        val = board[r][c]
        board[r][c] = 0
        tmp = copy.deepcopy(board)
        if solve(tmp):
            removed += 1
        else:
            board[r][c] = val
    return board


# ─── App ─────────────────────────────────────────────────────────────────────

class SudokuApp:
    CELL      = 62
    MARGIN    = 30
    BTN_H     = 46
    BTN_W     = 130
    ANIM_DELAY = 12   # ms per backtracking step when animating
    PANEL_H   = 110   # height of step-by-step info panel

    def __init__(self):
        pygame.init()
        self.grid_px = self.CELL * 9
        self.w = self.MARGIN * 2 + self.grid_px
        # extra height for step panel
        self.h = self.MARGIN * 3 + self.grid_px + self.BTN_H + 50 + self.PANEL_H
        self.screen = pygame.display.set_mode((self.w, self.h))
        pygame.display.set_caption("Sudoku  ·  Backtracking")
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_num   = pygame.font.SysFont("consolas", 30, bold=True)
        self.font_small = pygame.font.SysFont("consolas", 16)
        self.font_btn   = pygame.font.SysFont("consolas", 14, bold=True)
        self.font_title = pygame.font.SysFont("consolas", 22, bold=True)
        self.font_step  = pygame.font.SysFont("consolas", 15)
        self.font_step_b= pygame.font.SysFont("consolas", 15, bold=True)

        self.new_game()

    # ── Game state ──────────────────────────────────────────────────────────

    def new_game(self):
        full          = generate_full_board()
        self.solution = copy.deepcopy(full)
        self.puzzle   = make_puzzle(full, clues=random.randint(28, 36))
        self.board    = copy.deepcopy(self.puzzle)
        self.given    = [[self.puzzle[r][c] != 0 for c in range(9)] for r in range(9)]
        self.errors   = [[False]*9 for _ in range(9)]
        self.selected = None

        # Animation mode
        self.animating  = False
        self.anim_steps = []
        self.anim_idx   = 0
        self.anim_board = None
        self.last_anim  = 0

        # Step-by-step mode
        self.step_mode  = False
        self.step_steps = []   # list of (row, col, val, is_backtrack)
        self.step_idx   = 0    # current position (0 = puzzle state)
        self.step_board = None

        self.status_msg = ""
        self.solved     = False

    # ── Coordinates ─────────────────────────────────────────────────────────

    def cell_rect(self, r, c):
        x = self.MARGIN + c * self.CELL
        y = self.MARGIN + 30 + r * self.CELL
        return pygame.Rect(x, y, self.CELL, self.CELL)

    def pixel_to_cell(self, px, py):
        for r in range(9):
            for c in range(9):
                if self.cell_rect(r, c).collidepoint(px, py):
                    return r, c
        return None

    # ── Buttons ─────────────────────────────────────────────────────────────

    def btn_rects(self):
        """Top row: New Game | Résoudre (BT) | Pas à pas | Vérifier"""
        y = self.MARGIN * 2 + 30 + self.grid_px + 10
        gap = 10
        labels = 4
        total = labels * self.BTN_W + (labels - 1) * gap
        x0 = (self.w - total) // 2
        rects = []
        for i in range(labels):
            rects.append(pygame.Rect(x0 + i * (self.BTN_W + gap), y, self.BTN_W, self.BTN_H))
        return rects  # [New Game, Solve anim, Step mode, Check]

    def step_btn_rects(self):
        """Step panel: ◀ Précédent | ▶ Suivant | ▶▶ Fin | Quitter pas-à-pas"""
        panel_y = self.MARGIN * 2 + 30 + self.grid_px + 10 + self.BTN_H + 14 + 34
        bw, bh, gap = 140, 38, 10
        total = 3 * bw + 2 * gap
        x0 = (self.w - total) // 2
        rects = [
            pygame.Rect(x0,                  panel_y, bw, bh),
            pygame.Rect(x0 + bw + gap,       panel_y, bw, bh),
            pygame.Rect(x0 + 2*(bw + gap),   panel_y, bw, bh),
        ]
        return rects  # [Précédent, Suivant, Fin]

    # ── Validation ──────────────────────────────────────────────────────────

    def check_board(self):
        any_error = False
        for r in range(9):
            for c in range(9):
                v = self.board[r][c]
                if v != 0 and v != self.solution[r][c]:
                    self.errors[r][c] = True
                    any_error = True
                else:
                    self.errors[r][c] = False
        if not any_error:
            all_filled = all(self.board[r][c] != 0 for r in range(9) for c in range(9))
            self.status_msg = "✓ Parfait !" if all_filled else "Aucune erreur pour l'instant"
        else:
            self.status_msg = "Des erreurs détectées"

    # ── Solve animation ─────────────────────────────────────────────────────

    def start_solve_anim(self):
        self.step_mode  = False
        self.anim_board = copy.deepcopy(self.puzzle)
        steps = []
        tmp = copy.deepcopy(self.puzzle)
        solve(tmp, steps)
        self.anim_steps = steps
        self.anim_idx   = 0
        self.anim_board = copy.deepcopy(self.puzzle)
        self.animating  = True
        self.last_anim  = pygame.time.get_ticks()
        self.errors     = [[False]*9 for _ in range(9)]
        self.status_msg = "Résolution en cours…"
        self.selected   = None

    def tick_anim(self):
        now = pygame.time.get_ticks()
        if now - self.last_anim < self.ANIM_DELAY:
            return
        steps_per_tick = max(1, len(self.anim_steps) // 400)
        for _ in range(steps_per_tick):
            if self.anim_idx >= len(self.anim_steps):
                self.animating  = False
                self.board      = copy.deepcopy(self.anim_board)
                self.status_msg = "✓ Résolu par backtracking !"
                self.solved     = True
                return
            r, c, v, bt = self.anim_steps[self.anim_idx]
            self.anim_board[r][c] = v
            self.anim_idx += 1
        self.last_anim = now

    # ── Step-by-step mode ───────────────────────────────────────────────────

    def start_step_mode(self):
        self.animating  = False
        self.step_mode  = True
        self.errors     = [[False]*9 for _ in range(9)]
        self.selected   = None
        steps = []
        tmp   = copy.deepcopy(self.puzzle)
        solve(tmp, steps)
        self.step_steps = steps
        self.step_idx   = 0
        # Build board snapshots lazily: replay from puzzle each time (fast enough for step mode)
        self.step_board = copy.deepcopy(self.puzzle)
        self.status_msg = "Pas à pas — utilisez ◀ ▶ pour naviguer"

    def _rebuild_step_board(self, target_idx):
        """Rebuild board by replaying steps 0..target_idx-1 from the puzzle."""
        board = copy.deepcopy(self.puzzle)
        for i in range(target_idx):
            r, c, v, _ = self.step_steps[i]
            board[r][c] = v
        self.step_board = board

    def step_forward(self):
        if self.step_idx < len(self.step_steps):
            self.step_idx += 1
            self._rebuild_step_board(self.step_idx)
        if self.step_idx == len(self.step_steps):
            self.status_msg = "✓ Résolution terminée !"
            self.board = copy.deepcopy(self.step_board)

    def step_backward(self):
        if self.step_idx > 0:
            self.step_idx -= 1
            self._rebuild_step_board(self.step_idx)
            self.status_msg = "Pas à pas — utilisez ◀ ▶ pour naviguer"

    def step_to_end(self):
        self.step_idx = len(self.step_steps)
        self._rebuild_step_board(self.step_idx)
        self.status_msg = "✓ Résolution terminée !"
        self.board = copy.deepcopy(self.step_board)

    def current_step_info(self):
        """Return (row, col, val, is_backtrack, step_number, total) or None."""
        if not self.step_mode or self.step_idx == 0:
            return None
        r, c, v, bt = self.step_steps[self.step_idx - 1]
        return r, c, v, bt, self.step_idx, len(self.step_steps)

    # ── Draw helpers ────────────────────────────────────────────────────────

    def _draw_step_panel(self):
        """Draw the step-by-step information panel below the buttons."""
        panel_x = self.MARGIN
        panel_y = self.MARGIN * 2 + 30 + self.grid_px + 10 + self.BTN_H + 10
        panel_w = self.grid_px
        panel_h = self.PANEL_H

        pygame.draw.rect(self.screen, STEP_PANEL,
                         (panel_x, panel_y, panel_w, panel_h), border_radius=8)
        pygame.draw.rect(self.screen, STEP_BORDER,
                         (panel_x, panel_y, panel_w, panel_h), 1, border_radius=8)

        info = self.current_step_info()
        ty = panel_y + 10

        if info is None and self.step_idx == 0:
            hint = self.font_step.render("Appuyez sur ▶ Suivant pour commencer le pas à pas", True, ACCENT)
            self.screen.blit(hint, hint.get_rect(centerx=self.w // 2, y=ty + 6))
        elif info:
            r, c, v, bt, idx, total = info

            # Progress bar
            bar_x = panel_x + 10
            bar_y = ty + 2
            bar_w = panel_w - 20
            bar_h = 6
            pygame.draw.rect(self.screen, STEP_BORDER, (bar_x, bar_y, bar_w, bar_h), border_radius=3)
            fill = int(bar_w * idx / total)
            bar_col = TEXT_BACK if bt else TEXT_SOLVE
            if fill > 0:
                pygame.draw.rect(self.screen, bar_col, (bar_x, bar_y, fill, bar_h), border_radius=3)
            prog = self.font_step.render(f"Étape {idx} / {total}", True, (150, 150, 200))
            self.screen.blit(prog, (bar_x + bar_w + 8, bar_y - 2))

            ty += 18

            # Action description
            if bt:
                action_col  = TEXT_BACK
                action_icon = "↩"
                action_txt  = f"RETOUR ARRIÈRE  ligne {r+1}, col {c+1}"
                detail_txt  = f"Aucune valeur valide → effacement, on remonte"
            else:
                action_col  = TEXT_SOLVE
                action_icon = "→"
                action_txt  = f"PLACEMENT  ligne {r+1}, col {c+1}  ←  {v}"
                detail_txt  = f"La valeur {v} est valide ici, on continue"

            icon_s  = self.font_step_b.render(action_icon, True, action_col)
            label_s = self.font_step_b.render(action_txt,  True, action_col)
            self.screen.blit(icon_s,  (panel_x + 12, ty))
            self.screen.blit(label_s, (panel_x + 12 + icon_s.get_width() + 6, ty))
            ty += 22

            detail_s = self.font_step.render(detail_txt, True, (170, 170, 210))
            self.screen.blit(detail_s, (panel_x + 12, ty))
            ty += 20

            # Box info
            box_r, box_c = (r // 3) + 1, (c // 3) + 1
            box_txt = f"Boîte {box_r},{box_c}   |   Contraintes : ligne · colonne · boîte"
            box_s = self.font_step.render(box_txt, True, (120, 120, 170))
            self.screen.blit(box_s, (panel_x + 12, ty))

        elif self.step_idx == len(self.step_steps):
            done = self.font_step_b.render("✓ Toutes les étapes jouées — grille résolue !", True, TEXT_SOLVE)
            self.screen.blit(done, done.get_rect(centerx=self.w // 2, y=ty + 10))

        # Navigation buttons
        s_rects = self.step_btn_rects()
        mx, my  = pygame.mouse.get_pos()
        labels  = ["◀  Précédent", "▶  Suivant", "▶▶  Fin"]
        for i, (rect, lbl) in enumerate(zip(s_rects, labels)):
            hov = rect.collidepoint(mx, my)
            disabled = (i == 0 and self.step_idx == 0) or \
                       (i in (1, 2) and self.step_idx == len(self.step_steps))
            col = BTN_NORMAL if disabled else (BTN_HOVER if hov else BTN_NORMAL)
            alpha_col = (40, 40, 60) if disabled else col
            pygame.draw.rect(self.screen, alpha_col, rect, border_radius=8)
            border_col = (50, 50, 70) if disabled else ACCENT
            pygame.draw.rect(self.screen, border_col, rect, 1, border_radius=8)
            txt_col = (80, 80, 100) if disabled else BTN_TEXT
            surf = self.font_btn.render(lbl, True, txt_col)
            self.screen.blit(surf, surf.get_rect(center=rect.center))

    # ── Draw ────────────────────────────────────────────────────────────────

    def draw(self):
        self.screen.fill(BG)

        # Title
        title = self.font_title.render("SUDOKU", True, ACCENT)
        self.screen.blit(title, (self.MARGIN, 6))
        sub = self.font_small.render("backtracking solver", True, LINE_THICK)
        self.screen.blit(sub, (self.MARGIN + title.get_width() + 12, 10))

        # Grid shadow
        shadow_rect = pygame.Rect(self.MARGIN + 4, self.MARGIN + 34, self.grid_px, self.grid_px)
        pygame.draw.rect(self.screen, SHADOW, shadow_rect, border_radius=4)

        # Grid background
        grid_rect = pygame.Rect(self.MARGIN, self.MARGIN + 30, self.grid_px, self.grid_px)
        pygame.draw.rect(self.screen, GRID_BG, grid_rect, border_radius=4)

        # Current display board
        if self.step_mode:
            display = self.step_board
        elif self.animating:
            display = self.anim_board
        else:
            display = self.board

        # Highlighted cell from current step
        step_cell = None
        step_bt   = False
        if self.step_mode and self.step_idx > 0 and self.step_idx <= len(self.step_steps):
            sr, sc, sv, step_bt = self.step_steps[self.step_idx - 1]
            step_cell = (sr, sc)

        mx, my = pygame.mouse.get_pos()
        hover  = self.pixel_to_cell(mx, my)

        for r in range(9):
            for c in range(9):
                rect = self.cell_rect(r, c)
                v    = display[r][c]

                # Background
                if step_cell == (r, c):
                    bg_col = (80, 30, 30) if step_bt else (30, 70, 40)
                    pygame.draw.rect(self.screen, bg_col, rect)
                elif self.selected == (r, c):
                    pygame.draw.rect(self.screen, SELECTED_BG, rect)
                elif hover == (r, c) and not self.animating and not self.step_mode:
                    pygame.draw.rect(self.screen, CELL_HOVER, rect)

                # Highlight same row/col/box as selected (manual mode)
                if self.selected and not self.animating and not self.step_mode:
                    selr, selc = self.selected
                    same_box = (r // 3 == selr // 3) and (c // 3 == selc // 3)
                    if (r == selr or c == selc or same_box) and (r, c) != self.selected:
                        s = pygame.Surface((self.CELL, self.CELL), pygame.SRCALPHA)
                        s.fill((60, 80, 140, 60))
                        self.screen.blit(s, rect.topleft)

                # Highlight same row/col/box as current step cell
                if step_cell and (r, c) != step_cell:
                    sr2, sc2 = step_cell
                    same_box2 = (r // 3 == sr2 // 3) and (c // 3 == sc2 // 3)
                    if r == sr2 or c == sc2 or same_box2:
                        s = pygame.Surface((self.CELL, self.CELL), pygame.SRCALPHA)
                        tint = (80, 30, 30, 35) if step_bt else (30, 70, 30, 35)
                        s.fill(tint)
                        self.screen.blit(s, rect.topleft)

                # Number
                if v != 0:
                    if self.errors[r][c]:
                        color = TEXT_ERROR
                    elif step_cell == (r, c):
                        color = TEXT_BACK if step_bt else TEXT_SOLVE
                    elif self.animating and not self.given[r][c]:
                        color = TEXT_SOLVE
                    elif self.given[r][c]:
                        color = TEXT_GIVEN
                    elif self.step_mode and not self.given[r][c]:
                        color = TEXT_SOLVE
                    else:
                        color = TEXT_USER
                    surf = self.font_num.render(str(v), True, color)
                    self.screen.blit(surf, surf.get_rect(center=rect.center))

        # Grid lines
        ox = self.MARGIN
        oy = self.MARGIN + 30
        for i in range(10):
            thick = 3 if i % 3 == 0 else 1
            color = LINE_THICK if i % 3 == 0 else LINE_THIN
            pygame.draw.line(self.screen, color,
                             (ox + i * self.CELL, oy),
                             (ox + i * self.CELL, oy + self.grid_px), thick)
            pygame.draw.line(self.screen, color,
                             (ox, oy + i * self.CELL),
                             (ox + self.grid_px, oy + i * self.CELL), thick)

        # ── Top button row ──────────────────────────────────────────────────
        labels    = ["  Nouvelle partie", "  Résoudre (BT)", "  Pas à pas", "  Vérifier"]
        btn_rects = self.btn_rects()
        for i, (rect, label) in enumerate(zip(btn_rects, labels)):
            hov = rect.collidepoint(mx, my)
            active = (i == 2 and self.step_mode)
            col = BTN_ACTIVE if active else (BTN_HOVER if hov else BTN_NORMAL)
            pygame.draw.rect(self.screen, col, rect, border_radius=8)
            border = (200, 150, 255) if active else ACCENT
            pygame.draw.rect(self.screen, border, rect, 1, border_radius=8)
            surf = self.font_btn.render(label, True, BTN_TEXT)
            self.screen.blit(surf, surf.get_rect(center=rect.center))

        # ── Step panel ──────────────────────────────────────────────────────
        if self.step_mode:
            self._draw_step_panel()
        else:
            # Status message when not in step mode
            if self.status_msg:
                col = TEXT_SOLVE if "✓" in self.status_msg else \
                      (TEXT_ERROR if "erreur" in self.status_msg.lower() else ACCENT)
                smsg = self.font_small.render(self.status_msg, True, col)
                self.screen.blit(smsg, smsg.get_rect(
                    centerx=self.w // 2,
                    y=self.MARGIN * 2 + 30 + self.grid_px + 10 + self.BTN_H + 18))

        pygame.display.flip()

    # ── Event handling ───────────────────────────────────────────────────────

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            if self.animating:
                continue   # block input while auto-solving

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos

                # Top buttons
                btn_rects = self.btn_rects()
                if btn_rects[0].collidepoint(pos):
                    self.new_game()
                    return
                elif btn_rects[1].collidepoint(pos):
                    self.start_solve_anim()
                    return
                elif btn_rects[2].collidepoint(pos):
                    if self.step_mode:
                        self.step_mode  = False
                        self.status_msg = ""
                    else:
                        self.start_step_mode()
                    return
                elif btn_rects[3].collidepoint(pos):
                    if not self.step_mode:
                        self.check_board()
                    return

                # Step navigation buttons (only in step mode)
                if self.step_mode:
                    s_rects = self.step_btn_rects()
                    if s_rects[0].collidepoint(pos):
                        self.step_backward()
                    elif s_rects[1].collidepoint(pos):
                        self.step_forward()
                    elif s_rects[2].collidepoint(pos):
                        self.step_to_end()
                    return

                # Cell selection (manual play)
                if not self.step_mode:
                    cell = self.pixel_to_cell(*pos)
                    if cell:
                        self.selected = cell

            if event.type == pygame.KEYDOWN:
                # Step mode keyboard shortcuts
                if self.step_mode:
                    if event.key in (pygame.K_RIGHT, pygame.K_SPACE):
                        self.step_forward()
                    elif event.key == pygame.K_LEFT:
                        self.step_backward()
                    elif event.key == pygame.K_END:
                        self.step_to_end()
                    elif event.key == pygame.K_ESCAPE:
                        self.step_mode  = False
                        self.status_msg = ""
                    return

                # Manual digit entry
                if self.selected:
                    r, c = self.selected
                    if not self.given[r][c]:
                        if pygame.K_1 <= event.key <= pygame.K_9:
                            self.board[r][c] = event.key - pygame.K_0
                            self.errors[r][c] = False
                            self.status_msg = ""
                        elif event.key in (pygame.K_BACKSPACE, pygame.K_DELETE, pygame.K_0):
                            self.board[r][c] = 0
                            self.errors[r][c] = False
                    # Arrow navigation
                    dr = {pygame.K_UP: -1, pygame.K_DOWN: 1}.get(event.key, 0)
                    dc = {pygame.K_LEFT: -1, pygame.K_RIGHT: 1}.get(event.key, 0)
                    if dr or dc:
                        self.selected = (max(0, min(8, r + dr)), max(0, min(8, c + dc)))

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self):
        while True:
            self.handle_events()
            if self.animating:
                self.tick_anim()
            self.draw()
            self.clock.tick(60)


if __name__ == "__main__":
    SudokuApp().run()