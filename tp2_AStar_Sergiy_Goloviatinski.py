#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Sergiy Goloviatinski, inf3dlm-b


from city import *
from collections import deque
from math import sqrt
import heapq
import inspect
import sys

def initCitiesFromFile(fileName):
    cityDict = {}
    with open(fileName) as f:
        for c in [l.split() for l in f]:
            cityName = c[0]
            x = c[1]
            y = c[2]
            cityDict[cityName] = City(cityName, x, y)
    return cityDict


def initCityConnections(cityDict, connectionFileName):
    with open(connectionFileName) as f:
        for c in [l.split() for l in f]:
            distance = c[2]
            cityA = c[0]
            cityB = c[1]
            cityDict[cityA].addNeighbour(cityDict[cityB], int(distance))
            cityDict[cityB].addNeighbour(cityDict[cityA], int(distance))


# g(x) s'accumulle avec chaque noeud visité (somme des distances parcourues) et h(x) est donné dans le pdf (se réduit à chaque noeud visité vu qu'on se rapproche de la ville d'arrivée)
# Fortement inspiré de https://fr.wikipedia.org/wiki/Algorithme_A*
def findShortestPathFromAtoB(cityA, cityB, h, printPathInfo=True):
    cityState = CityState(cityA,0,h(cityA,cityB))
    closedList=deque()
    openList=[cityState]
    heapq.heapify(openList)

    searchCount=0

    while len(openList)>0:

        cityState=heapq.heappop(openList)

        if cityState.city == cityB:
            parent=cityState.parent
            path=[cityState]
            while parent is not None:
                path.append(parent)
                parent=parent.parent
            if printPathInfo:
                print(f"\tFound path (length = {len(path)}) from {cityA.name} to {cityB.name} after exploring {searchCount} states")
            return list(reversed(path))

        searchCount+=1
        neighbours = cityState.city.neighbours

        for neighbourCity, neighbourDistance in neighbours.items():
            neighbourCityState=CityState(neighbourCity,cityState.g+neighbourDistance,h(neighbourCity,cityB) ,cityState)
            if neighbourCityState in closedList and neighbourCityState.f > closedList[closedList.index(neighbourCityState)].f or neighbourCityState in openList and neighbourCityState.f > openList[openList.index(neighbourCityState)].f:
                pass
            else:
                heapq.heappush(openList,neighbourCityState)

        closedList.append(cityState)

cityDict = initCitiesFromFile("data/positions.txt")
initCityConnections(cityDict, "data/connections.txt")

heuristicList=[]

h0 = lambda a,b: 0
h1 = lambda a,b: abs(a.x-b.x)
h2 = lambda a,b: abs(a.y-b.y)
h3 = lambda a,b: sqrt(abs(a.x-b.x)**2 + abs(a.y-b.y)**2)
h4 = lambda a,b: abs(a.x-b.x) + abs(a.y-b.y)

heuristicList.extend([h0,h1,h2,h3,h4])

if __name__ == '__main__':

    cityTupleList=[]

    if len(sys.argv)>1 and (len(sys.argv)-1)%2==0:
        for i in range(1,len(sys.argv)-1,2):
            try:
                cityTupleList.append((cityDict[sys.argv[i].capitalize()],cityDict[sys.argv[i+1].capitalize()]))
            except KeyError as e:
                raise KeyError(f"The city you specified ({e.args[0]}) don't exist in our dataset")
    elif len(sys.argv)>1 and (len(sys.argv)-1)%2!=0:
        raise Exception("The cities must be given by pairs")
    else:
        # Default test values
        cityTupleList.append((cityDict["Bern"],cityDict["Amsterdam"]))
        cityTupleList.append((cityDict["Lisbon"],cityDict["Warsaw"]))


    for cityA, cityB in cityTupleList:
        print(f"Find shorthest path from {cityA.name} to {cityB.name} with {len(heuristicList)} different heuristics:\n")
        for h in heuristicList:
            print(f"\tHeuristic {inspect.getsource(h)}")
            spaces=[""]
            for state in findShortestPathFromAtoB(cityA, cityB, h):
                print(f"\t\t{''.join(spaces)}{'⮡ ' if len(spaces)>1 else ''}{state}")
                spaces.append(" ")
            print()
        print()
