# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 00:48:48 2018

@author: allen
"""
class MOOUtility():
    @classmethod
    def simple_cull(cls, inputPoints, dominates): #类方法
        paretoPoints = set()
        candidateRowNr = 0
        dominatedPoints = set()
        while True:
            candidateRow = inputPoints[candidateRowNr]
            inputPoints.remove(candidateRow)
            rowNr = 0
            nonDominated = True
            while len(inputPoints) != 0 and rowNr < len(inputPoints):
                row = inputPoints[rowNr]
                if dominates(candidateRow, row):
                    # If it is worse on all features remove the row from the array
                    inputPoints.remove(row)
                    dominatedPoints.add(tuple(row))
                elif dominates(row, candidateRow):
                    nonDominated = False
                    dominatedPoints.add(tuple(candidateRow))
                    rowNr += 1
                else:
                    rowNr += 1
    
            if nonDominated:
                # add the non-dominated point to the Pareto frontier
                paretoPoints.add(tuple(candidateRow))
    
            if len(inputPoints) == 0:
                break
        return paretoPoints, dominatedPoints
    
    def dominates(row, candidateRow):
        return sum([row[x] >= candidateRow[x] for x in range(len(row))]) == len(row)   