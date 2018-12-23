# IA - TP4 : Labyrinthe avec algorithme génétique
## Sergiy Goloviatinski
### Utilisation du programme
- `./tp4_Labyrinth_Sergiy_Goloviatinski.py grids/grid40.npy 75`
    - 1er argument: chemin vers fichier de grille
    - 2ème argument: condition d'arrêt en secondes
    - Si pas d'arguments, une grille aléatoire 5x5 sera utilisée avec comme condition d'arrêt 0.5 secondes
- `/tp4_Labyrinth_Sergiy_Goloviatinski.py random 7 4`
    - si 1er argument == "random"
    - le 2ème argument est le côté de la grille
    - le 3ème argument est la condition d'arrêt en secondes
### Choix des hyperparamètres
#### Fonction de fitness
- Elle se base sur la distance de manhattan entre la dernière case du chemin + la longueur du chemin
    - afin d'accorder plus d'importance à la distance de manhattan, un facteur la multiplie
#### Sélection
- Comme fonction de sélection j'ai choisi `selRoulette` car il y a quand même une chance qu'un individu avec un faible fitness soit sélectionné
#### Mutation
- Comme fonction de mutation j'ai choisi `mutShuffleIndexes` car c'était la plus approprié pour faire muter un genome basé sur des direction relatives
- J'ai choisi une probabilité de 0.75 pour favoriser la mutation et donc une éviter que les chemins convergent vers le même endroit à cause de mon mécanisme d'influence de choix décris plus bas dans ce readme (Hyperparamètres personnels)
#### Crossover
- Comme fonction de crossover j'ai choisi `cxOnePoint` car le chemin serait coupé en un seul endroit pour les 2 individus et c'est ainsi qu'il perdrait le moins de sens à mon avis
- J'ai choisi une probabilité de 0.25 car le crossover n'a que peu de sens dans ce problème, mais peut quand même débloquer des chemins qui vont dans une direction faussée ou dans un cul de sac
#### Taille des populations
- La taille population initiale devait pas être trop grande sinon le traitement prendrait trop de temps mais pas trop petite non plus car sinon les solutions obtenues ne seraient pas assez variés
    - Donc j'ai choisi une population initiale proportionnelle à la taille de grille, avec un facteur l'ajustant
- La taille de la population choisie pour la selection était choisi comme 1/3 de la population totale, ceci pour que une bonne partie des individus faibles soient remplacés mais pas tous (j'avais hésité de choisir 1/2 de la population initiale mais je me suis dis qu'une partie de solution potentiellement intéréssantes mais avec un fitness trop bas risquerait de se faire remplacer)
#### Hyperparamètres personnels
- J'ai des probabilités qui influencent le choix de la prochaine case du chemin (dans la méthode `generate_gene`):
    - Probabilité de choisir complètement au hasard une direction valide
    - Probabilité de choisir une direction valide moins celle qui est la plus loin de la cible (manhattan)
    - Probabilité de choisir la direction valide la plus proche de la cible (manhattan)
- Les meilleurs individus sont sauvegardés et réinjectés dans la population tous les 5 episodes

### Conclusion
- Ma solution fonctionne très bien avec les grilles fournies avec un temps raisonnable (~ ordre de grandeur comme spécifié dans le powerpoint est largement suffisant, souvent trouve 1ère solution avant 1 seconde) jusqu'à 30x30 y compris, par contre pour la grille 40x40 ça ne trouve pas de solutions, peut être est-ce lié à ma façon de valider un chemin qui ferait trop de parcours sur les différents individus et qui serait trop chronophage à cause de sa complexité algorithmique
- Les conditions particulières issus d'une génération aléatoire de la grille sont gérés
- Le programme est résistant aux crash