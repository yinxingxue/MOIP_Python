# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 16:49:14 2018

@author: Yinxing Xue
"""
import moipProb
import numpy as np
import cplex
from cplex import Cplex
from cplex.exceptions import CplexError

class BaseSol:
    'define the basic solution of a MOBIP'
    solver = None
    
    moipProblem = None 
    
    cplexSolutionSet = [] 
    
    cplexResultMap= {}
    
    lb=[] 
    
    ub=[]
    
    types=[]
    
    xvar=[]
    
    constrIndexList =[]
    
    def __init__(self, moipProblem):
        self.moipProblem = moipProblem
        
    def execute(self):
        self.solver.solve()
        #debugging purpose
        #print ("Solution value  = ", self.solver.solution.get_objective_value())
        #debugging purpose
        #xsol = self.solver.solution.get_values()
        #debugging purpose
        #print ('xsol = ',  xsol )
        #if(self.solver.solution.get_status_string()!='optimal'):
        #    return
        cplexResults = CplexSolResult(self.solver.solution,self.moipProblem)
        self.cplexSolutionSet.append(cplexResults)
        self.cplexResultMap[cplexResults.getResultID()] = cplexResults
    
    """
    model the problem as a single objective problem, and preparation solver for this
    """
    def prepare(self):
        self.solver = cplex.Cplex()
        self.solver.objective.set_sense(self.solver.objective.sense.minimize)
        self.ub = [1]*self.moipProblem.featureCount
        self.lb = [0]*self.moipProblem.featureCount
        self.types = 'B'* self.moipProblem.featureCount
        self.xvar = ['x'+str(i) for i in range(0,self.moipProblem.featureCount) ]
        firstObj=self.moipProblem.attributeMatrix[0]
        #add the objective and variables to the solver
        self.solver.variables.add(obj = firstObj, lb = self.lb, ub = self.ub, types = self.types, names = self.xvar )
        variableCount = self.moipProblem.featureCount
        constCounter =0 
        self.constrIndexList =[]
        #add the inequation constraints to the solver
        for ineqlDic in self.moipProblem.sparseInequationsMapList:
            rows = []
            rs = float("-inf")
            variables = []
            coefficient = []
            for key in ineqlDic: 
                if key != variableCount:
                    variables.append('x'+str(key))
                    coefficient.append(ineqlDic[key])
                else: 
                    rs = ineqlDic[key]
            row=[]
            row.append(variables)
            row.append(coefficient)
            rows.append(row)       
            constCounter= constCounter+1
            constName= 'c'+str(constCounter)
            indices = self.solver.linear_constraints.add(lin_expr = rows, senses = 'L', rhs = [rs], names = [constName] )
            self.constrIndexList.append(indices)
        del(ineqlDic)
        #add the equation constraints to the solver
        for eqlDic in self.moipProblem.sparseEquationsMapList:
            rows = []
            rs = float("-inf")
            variables = []
            coefficient = []
            for key in eqlDic: 
                if key != variableCount:
                    variables.append('x'+str(key))
                    coefficient.append(eqlDic[key])
                else: 
                    rs = eqlDic[key]
            row=[]
            row.append(variables)
            row.append(coefficient)
            rows.append(row)       
            constCounter= constCounter+1
            constName= 'c'+str(constCounter)
            indices = self.solver.linear_constraints.add(lin_expr = rows, senses = 'E', rhs = [rs], names = [constName] )
            self.constrIndexList.append(indices)
        del(eqlDic)
        #for debugging purpose
        #print (self.constrIndexList)
        #for debugging purpose
        #self.__private_testConstraints__()
        
        
    def displayVariableLowerBound(self):
        print ("Variable Loweer Bounds: %s" % self.lb) 
    
    def displayVariableUpperBound(self):
        print ("Variable Upper Bounds: %s" % self.ub) 
        
    def displayVariableTypes(self):
        print ("Variable Types: %s" % self.types) 
        
    def displayVariableNames(self):
        print ("Variable Names: %s" % self.xvar) 
        
    def displayCplexSolutionSet(self):
        print ("Total Solution Set: %s" % self.cplexSolutionSet) 
    
    def displayCplexSolutionSetSize(self):
        print ("Total Solution Set Size: %s" % len(self.cplexSolutionSet)) 
        
    def displayCplexResultMap(self):
        print ("Cplex Results Map: %s" % self.cplexResultMap) 
        
    def __private_testConstraints__(self):
        for i in range(self.solver.linear_constraints.get_num()):
            print (self.solver.linear_constraints.get_rows(i), self.solver.linear_constraints.get_senses(i), self.solver.linear_constraints.get_rhs(i))

class CplexSolResult:
    'define the basic solution of a MOBIP'
    xvar = []
    objs = []
    solveStatus = ""
    ResultID = ""
    
    def __init__(self, solverSolution, moipProblem):
        self.xvar = solverSolution.get_values()
        objMatrix = moipProblem.attributeMatrix
        self.objs = self.getAllObjs(solverSolution,objMatrix)
        self.solveStatus = solverSolution.get_status_string()
        for i in range(0,len(self.objs)):
            value = self.objs[i]
            valueString = "%.2f" % value
            self.ResultID += valueString
            if i != len(self.objs) -1:
                self.ResultID += '_'
    
    def getAllObjs(self,solverSolution,objMatrix):
        # dimension: k* variableCount
        matA = np.array(objMatrix)
        twoDArray= []
        twoDArray.append(solverSolution.get_values())
        # dimension: variableCount * 1
        matB = np.array(twoDArray).transpose()
        allObjs= np.dot(matA, matB)
        allObjs = allObjs.transpose()
        allObjsList = allObjs.tolist()
        return allObjsList[0]
    
    def getResultID(self):
        return self.ResultID
    
if __name__ == "__main__":
    prob = MOIPProblem(4,43,3)  
    prob.displayObjectiveCount()
    prob.displayFeatureCount()
    prob.exetractFromFile("parameter.txt")
    prob.displayObjectives()
    prob.displayVariableNames()
    prob.displayObjectiveSparseMapList()
    prob.displaySparseInequationsMapList()
    prob.displaySparseEquationsMapList()
    prob.displayAttributeMatrix()
    
    sol= BaseSol(prob)
    sol.prepare()
    sol.execute()
    sol.displayCplexSolutionSetSize()
    sol.displayCplexResultMap()
    sol.displayVariableLowerBound()
    sol.displayVariableUpperBound()
    sol.displayVariableTypes()
    sol.displayVariableNames()
else:
    print("moipSol.py is being imported into another module")
