## Quel algorithme d'exploration locale est meilleur ?

Recuit Simulé semble etre le meilleur algorythme.

## Quels sont les avantages et inconvénients de chacun ?

### Hill Climbing :
Avantages :
- Simple à implémenter.
- Efficace pour les problèmes de petite taille.
- Utile pour les problèmes où l'espace de recherche est trop grand pour être exploré de manière exhaustive.

Inconvenients:
- Peut se bloquer dans des optima locaux.
- Ne garantit pas de trouver la solution optimale globale.
- Sensible au choix de la solution initiale.

### Recuit Simulé :
Avantages :
- Peut sortir des optima locaux grâce à l'acceptation de solutions moins bonnes.
- Flexible et peut être appliqué à une large variété de problèmes.
- Ne nécessite pas de dérivées ou de gradients, ce qui le rend utile pour les fonctions non différentiables.

Inconvenients :
- Peut être lent pour les grands problèmes.
- Le choix des paramètres (température initiale, taux de refroidissement) peut être délicat.
- Ne garantit pas de trouver la solution optimale globale.

## Proposition d'un autre algorithme

La Recherche à Voisinage Variable