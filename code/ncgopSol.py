# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 10:28:33 2018

@author: Yinxing Xue
"""
import math
import numpy as np
from moipProb import MOIPProblem 
from moipSol import BaseSol
from naiveSol import NaiveSol
from cwmoipSol import CwmoipSol
from normalConstraint import UtopiaPlane
from moipSol import CplexSolResult
from mooUtility import MOOUtility 
from decimal import Decimal


class NcgopSol(CwmoipSol):  
    'define the NC+H&R solution of a MOBIP'
    def __init__(self, moipProblem):  
        #override parent initializer  
        CwmoipSol.__init__(self,moipProblem)  
        self.utopiaPlane = None
        
    #override the parent method 
    def execute(self):
        self.utopiaPlane = UtopiaPlane(self.moipProblem,self.solver);
        self.utopiaPlane.calculate();
        print("utopiaPlane done.");



if __name__ == "__main__":
    prob = MOIPProblem(4,43,3)  
    prob.displayObjectiveCount()
    prob.displayFeatureCount()
    prob.exetractFromFile("../test/parameter_wp1.txt")
    prob.displayObjectives()
    prob.displayVariableNames()
    prob.displayObjectiveSparseMapList()
    prob.displaySparseInequationsMapList()
    prob.displaySparseEquationsMapList()
    prob.displayAttributeMatrix()
    
    sol= NcgopSol(prob)
    sol.prepare()
    sol.execute()
    sol.outputCplexParetoMap("../result/Pareto_wp1.txt")
    sol.displaySolvingAttempts()
    sol.displayObjsBoundsDictionary()
    sol.displayCplexSolutionSetSize()
    sol.displayCplexResultMap()
    sol.displayCplexParetoSet()
    sol.displayVariableLowerBound()
    sol.displayVariableUpperBound()
    sol.displayVariableTypes()
    sol.displayVariableNames()
else:
    print("ncgopSol.py is being imported into another module")
        
        
        