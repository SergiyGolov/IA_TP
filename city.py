#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Sergiy Goloviatinski, inf3dlm-b


class City(object):
    def __init__(self, name, x, y):
        self.name = name
        self.x = int(x)
        self.y = int(y)
        self.neighbours = {}

    def addNeighbour(self, neighbourCity, distance):
        self.neighbours[neighbourCity] = distance

    def __str__(self):
        lines = []
        lines.append(f"Name: {self.name} x: {self.x} y: {self.y}")
        for neighbour, distance in self.neighbours.items():
            lines.append(f"\t-> {neighbour.name} : {distance}")
        return '\n'.join(lines)

class CityState(object):
    def __init__(self,city,g=0,h=0,parent=None):
        self.city=city
        self.parent=parent
        self.g=g
        self.h=h
        self.f=g+h

    def __str__(self):
        return str(f"CityState: {self.city.name}, g: {self.g} h:{self.h} f:{self.f}")

    def __lt__(self, other):
        return self.f < other.f
