# IA - TP4 : Labyrinthe avec algorithme génétique
## Sergiy Goloviatinski
### Choix des hyperparamètres
#### Fonction de fitness
- Elle se base sur la distance de manhattan entre la dernière case du chemin + la longueur du chemin
    - afin d'accorder plus d'importance à la distance de manhattan, un facteur la multiplie
#### Sélection
- Comme fonction de sélection j'ai choisi `selRoulette` car
#### Mutation
- Comme fonction de mutation j'ai choisi `mutShuffleIndexes` car c'était la plus approprié pour faire muter un genome basé sur des direction relatives
#### Crossover
- Comme fonction de crossover j'ai choisi `cxOnePoint` car le chemin serait coupé en un seul endroit pour les 2 individus et c'est ainsi qu'il perdrait le moins de sens à mon avis
#### Taille des populations
- La taille population initiale devait pas être trop grande sinon le traitement prendrait trop de temps mais pas trop petite non plus car sinon les solutions obtenues ne seraient pas assez variés
    - Donc j'ai choisi une population initiale proportionnelle à la taille de grille, avec un facteur l'ajustant
- La taille de la population choisie pour la selection était choisi comme 1/3 de la population totale, ceci pour que une bonne partie des individus faibles soient remplacés mais pas tous (j'avais hésité de choisir 1/2 de la population initiale mais je me suis dis qu')
#### Hyperparamètres personnels
- J'ai des probabilités qui influencent le choix de la prochaine case du chemin (dans la méthode `generate_gene`):
    - Probabilité de choisir complètement au hasard une direction valide
    - Probabilité de choisir une direction valide moins celle qui est la plus loin de la cible (manhattan)
    - Probabilité de choisir la direction valide la plus proche de la cible (manhattan)
- Les meilleurs individus sont sauvegardés et reinjectés dans la population tous les 5 episodes

### Conclusion