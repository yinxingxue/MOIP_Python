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
        points = self.pGenerator.calculate()
        print("pGenerator done.")
        
        #for utopiaPlane calculation, to assure the completeness of resolving, we cannot set timeout for this.cplex. 
		 #then for NCPDG, we can give the timeout for intlinprog timeout for each execution 
		 #for non-Linux projects
        if self.varNv < 2000:
            self.solver.parameters.timelimit.set(BaseSol.TimeOut)
            self.solver.parameters.dettimelimit.set(BaseSol.DeterTimeOut)
        else :
            self.solver.parameters.workmem.set(SolRep.WorkMem)
            self.solver.parameters.dettimelimit.set(SolRep.DeterTimeOut) 
            
        counter = 0
        for p_k in points:
            counter += 1
            print ("using p_k: ", str(counter))
            calculate(p_k, self.utopiaPlane.y_up,self.utopiaPlane.y_ub, self.utopiaPlane.y_lb)
        print ("Find solution num: ", len(self.cplexSolutionSet))   
        
    def  calculate(self, p_k, y_up, y_ub, y_lb):
        fCWMOIP = float('nan')
        No = self.objNo
        Nv = self.varNv
        
        extra_A1 = []
        objMatrix = np.array(self.moipProblem.attributeMatrix)
        SolRep.appendToSparseMapList(extra_A1,objMatrix[0:No-1])
        extra_B1 = p_k[0:No-1]
        
        lb = np.zeros((1,Nv))
        lb = np.ones((1,Nv))
        
        ff = []
        for i in range(No-1,-1,-1):
            if i == No-1:
                ff = objMatrix[i]
            else:
                w = 1.0 /(y_ub[i]-y_lb[i]+1)
                ff = ff+ w*objMatrix[i]
        
        (rstStatusString, rstXvar, rstObj) = intlinprog (self.solver, self.xVar, objMatrix, extra_A1, extra_B1, [], [], lb, ub)
        if(rstStatusString.find("optimal")>=0): 
            newff = np.reshape(ff, (1, len(ff))
            new_A1 = []
            SolRep.appendToSparseMapList(new_A1,newff)
            new_b1 = np.dot(rstXvar, newff)
        if(rstStatusString.find("optimal")>=0): 
            newff = np.reshape(ff, (1, len(ff))
            cplexResults = CplexSolResult(rstXvar,rstStatusString,self.moipProblem)
            self.addTocplexSolutionSetMap(cplexResults)
            fCWMOIP = np.dot(rstXvar, newff)
        return fCWMOIP
    
    @classmethod  
    def intlinprog (cplex, xVar, objMatrix, extra_A1, extra_B1, extra_Aeq1, extra_Beq1, lb, ub):
        

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
        
        
        