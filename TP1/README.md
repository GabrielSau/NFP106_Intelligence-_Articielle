## Quel algorithme de problèmes à satisfaction de contraintes est meilleur ?

Backtracking

## Quels sont les avantages et inconvénients de chacun ?

Backtracking 
Avantages : 
- Simple à implémenter.
- Garantit de trouver une solution si elle existe (complet).

Inconvénients : 
- Très lent sur de grands problèmes (complexité exponentielle).
- Souffre du "phénomène de répétition" : il peut refaire les mêmes erreurs dans différentes branches.


Min-Conflicts:
Avantages : 
- Incroyablement rapide, même pour des millions de variables (ex: le problème des $N$-reines pour $N=1,000,000$).
- Consomme très peu de mémoire.

Inconvénients :
- Incomplet : il peut rester bloqué dans un minimum local et ne jamais trouver de solution, même s'il en existe une.
- Ne peut pas prouver qu'un problème n'a pas de solution.

## Proposez en un autre
LCV