import heapq
import math
import pygame
import sys

import Glouton_a_completer as base

# ─────────────────────────────────────────────
# Plus besoin de W (poids) car nous utilisons
# une recherche GLOUTONNE (Greedy Best-First).
# La priorité n'est dictée que par h(n).
# ─────────────────────────────────────────────

def heuristique_manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ── helpers internes ──────────────────────────

def _pop_valide(pq, visite, g):
    while pq:
        fcur, gcur, node = heapq.heappop(pq)
        if node in visite:
            continue
        if gcur != g.get(node):
            continue
        return (fcur, gcur, node)
    return None

def _maj_vues_ui(etat):
    etat["visite"] = set(etat["visite_s"]) | set(etat["visite_g"])
    etat["frontiere"] = set(etat["frontiere_s"]) | set(etat["frontiere_g"])

    g_all = dict(etat["g_g"])
    for node, val in etat["g_s"].items():
        prev = g_all.get(node)
        if prev is None or val < prev:
            g_all[node] = val
    etat["g"] = g_all

    if not etat.get("trouve", False):
        etat["parent"] = dict(etat["parent_s"])

def _finaliser_rencontre(etat, meet, couts):
    path_start = []
    cur = meet
    while cur is not None:
        path_start.append(cur)
        cur = etat["parent_s"].get(cur)
    path_start.reverse()

    path_goal = []
    cur = etat["parent_g"].get(meet)
    while cur is not None:
        path_goal.append(cur)
        cur = etat["parent_g"].get(cur)

    full = path_start + path_goal

    parent_full = {}
    for i, node in enumerate(full):
        parent_full[node] = None if i == 0 else full[i - 1]

    g_full = {}
    if full:
        g_full[full[0]] = 0
        for i in range(1, len(full)):
            g_full[full[i]] = g_full[full[i - 1]] + base.cout_case(couts, full[i])

    etat["parent"] = parent_full
    etat["g"] = g_full
    etat["courant"] = meet
    etat["trouve"] = True
    etat["termine"] = True

    _maj_vues_ui(etat)

# ── API publique ──────────────────────────────

def astar_initialiser(depart, arrivee):
    g0 = 0
    h_s = heuristique_manhattan(depart, arrivee)
    h_g = heuristique_manhattan(arrivee, depart)

    # GLOUTON: la priorité est uniquement h(n)
    pq_s = [(h_s, g0, depart)]
    pq_g = [(h_g, g0, arrivee)]
    heapq.heapify(pq_s)
    heapq.heapify(pq_g)

    etat = {
        "pq_s": pq_s,
        "pq_g": pq_g,
        "g_s": {depart: 0},
        "g_g": {arrivee: 0},
        "parent_s": {depart: None},
        "parent_g": {arrivee: None},
        "visite_s": set(),
        "visite_g": set(),
        "frontiere_s": {depart},
        "frontiere_g": {arrivee},
        "tour": "g",
        "depart": depart,
        "arrivee": arrivee,
        "courant": None,
        "courant_s": None,
        "courant_g": None,
        "termine": False,
        "trouve": False,
        "visite": set(),
        "frontiere": {depart, arrivee},
        "parent": {depart: None},
        "g": {depart: 0},
    }
    return etat

def astar_faire_une_etape(grille, etat, arrivee, couts):
    if etat["termine"]:
        return

    if not etat["pq_s"] or not etat["pq_g"]:
        etat["termine"] = True
        etat["trouve"] = False
        etat["courant"] = etat["courant_s"] = etat["courant_g"] = None
        _maj_vues_ui(etat)
        return

    if etat["tour"] == "s":
        popped = _pop_valide(etat["pq_s"], etat["visite_s"], etat["g_s"])
        etat["tour"] = "g"
        if popped is None:
            if not etat["pq_s"]:
                etat["termine"] = True
                etat["trouve"] = False
                etat["courant"] = etat["courant_s"] = None
            _maj_vues_ui(etat)
            return

        _fcur, gcur, courant = popped
        etat["courant"] = courant
        etat["courant_s"] = courant
        etat["visite_s"].add(courant)
        etat["frontiere_s"].discard(courant)

        if courant in etat["g_g"]:
            _finaliser_rencontre(etat, courant, couts)
            return

        cr, cc = courant
        for vr, vc, _dir_name in base.voisins_4(grille, cr, cc):
            nxt = (vr, vc)
            if nxt in etat["visite_s"]:
                continue
            new_g = gcur + base.cout_case(couts, nxt)
            if new_g < etat["g_s"].get(nxt, float("inf")):
                etat["g_s"][nxt] = new_g
                etat["parent_s"][nxt] = courant
                h = heuristique_manhattan(nxt, arrivee)
                # GLOUTON: on ne pousse que h(n) en priorité
                heapq.heappush(etat["pq_s"], (h, new_g, nxt))
                etat["frontiere_s"].add(nxt)
                if nxt in etat["g_g"]:
                    _finaliser_rencontre(etat, nxt, couts)
                    return

    else:  # tour == "g"
        popped = _pop_valide(etat["pq_g"], etat["visite_g"], etat["g_g"])
        etat["tour"] = "s"
        if popped is None:
            if not etat["pq_g"]:
                etat["termine"] = True
                etat["trouve"] = False
                etat["courant"] = etat["courant_g"] = None
            _maj_vues_ui(etat)
            return

        _fcur, gcur, courant = popped
        etat["courant"] = courant
        etat["courant_g"] = courant
        etat["visite_g"].add(courant)
        etat["frontiere_g"].discard(courant)

        if courant in etat["g_s"]:
            _finaliser_rencontre(etat, courant, couts)
            return

        cr, cc = courant
        for vr, vc, _dir_name in base.voisins_4(grille, cr, cc):
            nxt = (vr, vc)
            if nxt in etat["visite_g"]:
                continue
            new_g = gcur + base.cout_case(couts, nxt)
            if new_g < etat["g_g"].get(nxt, float("inf")):
                etat["g_g"][nxt] = new_g
                etat["parent_g"][nxt] = courant
                h = heuristique_manhattan(nxt, etat["depart"])
                # GLOUTON: on ne pousse que h(n) en priorité
                heapq.heappush(etat["pq_g"], (h, new_g, nxt))
                etat["frontiere_g"].add(nxt)
                if nxt in etat["g_s"]:
                    _finaliser_rencontre(etat, nxt, couts)
                    return

    _maj_vues_ui(etat)

def reconstruire_chemin(parent, depart, arrivee):
    if depart is None or arrivee is None:
        return None
    if depart == arrivee:
        return [depart]
    if arrivee not in parent:
        return None

    path = []
    cur = arrivee
    vus = set()
    while cur is not None:
        if cur in vus:
            return None
        vus.add(cur)
        path.append(cur)
        cur = parent.get(cur)

    path.reverse()
    if not path or path[0] != depart:
        return None
    return path


# ── Injection dans l'app de base ──────────────
base.heuristique_manhattan = heuristique_manhattan
base.astar_initialiser = astar_initialiser
base.astar_faire_une_etape = astar_faire_une_etape
base.reconstruire_chemin = reconstruire_chemin


# ── Application ───────────────────────────────

class AppliBidirectionnelle(base.AppliAStar):
    def __init__(self, grille):
        self.route_g = []
        self.index_route_g = 0
        self.dernier_pas_route_g = 0
        
        super().__init__(grille)
        pygame.display.set_caption("Labyrinthe - Bidirectionnel Glouton")

        self.pos_pingouin_g = self.sortie
        self.dir_pingouin_g = 0
        self.nb_pas_g = 0
        self.cout_total_g = 0

    def reinitialiser_tout(self):
        self.route_g = []
        self.index_route_g = 0
        self.dernier_pas_route_g = 0
        
        super().reinitialiser_tout()
        
        self.pos_pingouin_g = self.sortie
        self.dir_pingouin_g = 0
        self.nb_pas_g = 0
        self.cout_total_g = 0

    def _sync_depuis_etat_algo(self):
        super()._sync_depuis_etat_algo()

        if self.etat_algo is None:
            return

        courant_g = self.etat_algo.get("courant_g")
        if courant_g is None:
            self.route_g = []
            self.index_route_g = 0
            return

        # Construction de la route animée pour le pingouin E (arrivée)
        parent_g = self.etat_algo.get("parent_g", {})
        full, up_len = base.route_dans_arbre_parent_detail(parent_g, self.pos_pingouin_g, courant_g)
        
        self.route_g = full[1:]
        self.index_route_g = 0

    def _avancer_sur_routes(self, now):
        # 1. Avancer le pingouin de départ (S)
        if self.route:
            super()._avancer_sur_route(now)

        # 2. Avancer le pingouin d'arrivée (G)
        if not self.route_g:
            return

        if self.index_route_g >= len(self.route_g):
            self.route_g = []
            return

        if now - self.dernier_pas_route_g < base.PAS_ROUTE_MS:
            return

        old = self.pos_pingouin_g
        nxt = self.route_g[self.index_route_g]
        self.index_route_g += 1

        if nxt != old:
            self.nb_pas_g += 1
            if nxt != self.sortie:
                self.cout_total_g += base.cout_case(self.couts, nxt)

        # Mettre à jour la direction visuelle
        d = base.nom_direction(old, nxt)
        if d == "Haut":
            self.dir_pingouin_g = 0
        elif d == "Droite":
            self.dir_pingouin_g = 1
        elif d == "Bas":
            self.dir_pingouin_g = 2
        elif d == "Gauche":
            self.dir_pingouin_g = 3

        self.pos_pingouin_g = nxt
        self.vu.add(nxt)
        self.dernier_pas_route_g = now

    def _alpha_fog_spotlight(self, r, c):
        """Surcharge du brouillard pour illuminer autour des DEUX pingouins."""
        base_alpha = super()._alpha_fog_spotlight(r, c)

        if self.pos_pingouin_g is None:
            return base_alpha

        # Calcul du brouillard autour du pingouin G (arrivée)
        pr, pc = self.pos_pingouin_g
        d = math.sqrt((r - pr) ** 2 + (c - pc) ** 2)
        base_fog = base.ALPHA_FOG_CONNU if (r, c) in self.vu else base.ALPHA_FOG_INCONNU

        if d <= base.RAYON_LUMIERE_CASES:
            alpha_g = min(base_fog, base.ALPHA_MIN_SPOT)
        elif d >= base.RAYON_FONDU_CASES:
            alpha_g = base_fog
        else:
            t = (d - base.RAYON_LUMIERE_CASES) / (base.RAYON_FONDU_CASES - base.RAYON_LUMIERE_CASES)
            t = t * t
            alpha_g = int(base.ALPHA_MIN_SPOT * (1 - t) + base_fog * t)
            alpha_g = max(0, min(255, alpha_g))

        # On prend la plus grande visibilité (le plus petit alpha des deux pingouins)
        return min(base_alpha, alpha_g)

    def dessiner_barre_bas(self):
        y = base.HAUT_BAR_H + self.hauteur_monde
        pygame.draw.rect(self.ecran, base.COL_PANEL, pygame.Rect(0, y, self.largeur_fenetre, base.BAS_BAR_H))
        pygame.draw.line(self.ecran, base.COL_PANEL_BORD, (0, y), (self.largeur_fenetre, y), 2)

        opt = "—" if self.cout_opt is None else str(int(self.cout_opt))
        self._dessiner_texte(12, y + 8, f"Coût trouvé (Bidirectionnel Glouton) : {opt}", self.font_petit)
        self._dessiner_texte(
            12,
            y + 32,
            f"Pingouin S (départ)  → pas: {self.nb_pas} | coût: {self.cout_total}",
            self.font_petit,
        )
        self._dessiner_texte(
            self.largeur_monde - 310,
            y + 32,
            f"Pingouin E (arrivée) → pas: {self.nb_pas_g} | coût: {self.cout_total_g}",
            self.font_petit,
            base.COL_TEXTE_MUET,
        )

    def dessiner_monde(self):
        super().dessiner_monde()
        if self.pos_pingouin_g is None:
            return

        pr, pc = self.pos_pingouin_g
        rect = self._rect_case(pr, pc)
        frame = self.frames_pingouin[self.dir_pingouin_g][self.frame_pingouin]
        fw, fh = frame.get_size()

        # Petit décalage si les pingouins se chevauchent sur la même case
        dx, dy = (6, 4) if self.pos_pingouin_g == self.pos_pingouin else (0, 0)

        self.ecran.blit(
            frame,
            (rect.x + (base.TAILLE_CASE - fw) // 2 + dx,
             rect.y + (base.TAILLE_CASE - fh) // 2 + dy),
        )
        tag = self.font_tiny.render("E", True, base.COL_SORTIE)
        self.ecran.blit(tag, (rect.x + base.TAILLE_CASE - 12, rect.y + 4))

    def run(self):
        """Remplacement de la boucle d'événements pour animer les deux trajets"""
        while True:
            now = pygame.time.get_ticks()

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit(0)

                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_q:
                        pygame.quit()
                        sys.exit(0)
                    if ev.key == pygame.K_r:
                        self.reinitialiser_tout()
                    if ev.key == pygame.K_f:
                        self.brouillard_actif = not self.brouillard_actif
                    if ev.key == pygame.K_e:
                        self.reinitialiser_tout()
                        self.mode = "auto"
                        self.dernier_event_auto = 0
                        self.etat_algo = astar_initialiser(self.depart, self.sortie)
                        self._sync_depuis_etat_algo()
                    if ev.key == pygame.K_SPACE:
                        if self.mode == "play" or self.overlay_chemin_opt:
                            self.reinitialiser_tout()
                        if self.etat_algo is None:
                            self.etat_algo = astar_initialiser(self.depart, self.sortie)
                        self.mode = "step"
                        astar_faire_une_etape(self.grille, self.etat_algo, self.sortie, self.couts)
                        self._sync_depuis_etat_algo()
                    if ev.key == pygame.K_p:
                        self.reinitialiser_tout()
                        if self.chemin_opt:
                            self.reinitialiser_pour_chemin_optimal()
                        else:
                            self._histo_push("Pas de chemin trouvé.")

            # Gestion coordonnée du mouvement
            en_mouvement = bool(self.route) or bool(self.route_g)

            if self.mode in ("auto", "step") and en_mouvement:
                self._avancer_sur_routes(now)

            en_mouvement_apres = bool(self.route) or bool(self.route_g)

            if self.mode == "auto" and not en_mouvement_apres and self.etat_algo is not None:
                if now - self.dernier_event_auto >= base.ASTAR_EVENT_MS:
                    if self.etat_algo.get("termine"):
                        self.mode = "idle"
                        self.reveler_complet = True
                    else:
                        astar_faire_une_etape(self.grille, self.etat_algo, self.sortie, self.couts)
                        self._sync_depuis_etat_algo()
                        self.dernier_event_auto = now

            if self.mode == "play":
                self._maj_chemin_optimal(now)

            self._animer_pingouin(now)

            self.ecran.fill(base.COL_FOND)
            self.dessiner_barre_haut()
            self.dessiner_monde()
            self.dessiner_panneau_droit()
            self.dessiner_barre_bas()

            pygame.display.flip()
            self.clock.tick(base.FPS)

if __name__ == "__main__":
    AppliBidirectionnelle(base.LABYRINTHE).run()