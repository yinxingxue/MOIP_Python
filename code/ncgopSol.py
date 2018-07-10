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
from normalConstraint import SolRep
from moipSol import CplexSolResult
from mooUtility import MOOUtility 
from decimal import Decimal


class NcgopSol(CwmoipSol):  
    'define the NC+H&R solution of a MOBIP'
    def __init__(self, moipProblem):  
        #override parent initializer  
        CwmoipSol.__init__(self,moipProblem)
        #represent the coefficient of the original inequation constraints
        self.origi_A = [] 
        #represent the right side value of the original inequation constraints
        self.origi_B = []
        #represent the coefficient of the original equation constraints
        self.origi_Aeq = [] 
        #represent the right side value of the original equation constraints
        self.origi_Beq = []
        #represent the coefficient of the extra inequation constraints
        self.extra_A = [] 
        #represent the right side value of the extra inequation constraints
        self.extra_B = []
        #represent the coefficient of the extra equation constraints
        self.extra_Aeq = [] 
         #represent the right side value of the extra equation constraints
        self.extra_Beq = []
        self.utopiaPlane = None
        self.pGenerator = None
        self.objNo = moipProblem.objectiveCount  
        self.varNv= moipProblem.featureCount
        self.builtEqualAndInequalMaps(moipProblem) 
    
    def builtEqualAndInequalMaps(self,moipProblem):
        for i in range(0, len(moipProblem.sparseInequationsMapList)):
            inequal = moipProblem.sparseInequationsMapList[i].copy()
            right = inequal[self.varNv]
            del inequal[self.varNv]
            #for testing purpose
            #print (inequal,moipProblem.sparseInequationsMapList[i])
            self.origi_A.append(inequal)
            self.origi_B.append(right)
        for i in range(0, len(moipProblem.sparseEquationsMapList)):
            equal = moipProblem.sparseEquationsMapList[i].copy()
            right = equal[self.varNv]
            del equal[self.varNv]
            #for testing purpose
            #print (equal,moipProblem.sparseEquationsMapList[i])
            self.origi_Aeq.append(equal)
            self.origi_Beq.append(right)
        assert len(self.origi_A)== len(self.origi_B)     
        assert len(self.origi_Aeq)== len(self.origi_Beq)     
        
    #override the parent method 
    def execute(self):
        temp = self.moipProblem.attributeMatrix[2]
        self.moipProblem.attributeMatrix[2] =  self.moipProblem.attributeMatrix[3] 
        self.moipProblem.attributeMatrix[3] = temp
        
        self.utopiaPlane = UtopiaPlane(self.moipProblem,self.solver);
        self.utopiaPlane.calculate();
        print("utopiaPlane done.");
        self.pGenerator= SolRep(self.utopiaPlane.y_up, self.varNv, 1000)
        objMatrix = np.array(self.moipProblem.attributeMatrix)
        self.pGenerator.setParas(objMatrix, self.origi_A , self.origi_B, self.origi_Aeq, self.origi_Beq)
        self.pGenerator.calculate()

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
        
        
        