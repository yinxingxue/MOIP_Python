# -*- coding: utf-8 -*-
"""
Created on Sat Jun 30 13:44:12 2018

@author: Yinxing Xue
"""
import numpy as np
 

class UtopiaPlane():
    'define the calculation and implementation of Utopia Plane'
    def __init__(self, moipProblem, cplex):
        #input
        self.cplex = None
        self.attributeMatrix_in  = [[]]
        self.sparseInequationsMapList = [] 
        self.sparseEquationsMapList = []
        #output
        self.x_up = [[]]
        self.y_up = [[]]
        self.y_ub=  []
        self.y_lb=  []
        self.xVar = []
        self.__private_initialize__(moipProblem,cplex)
        
    def __private_initialize__(self,moipProblem,cplex):
        self.cplex = cplex
        self.attributeMatrix_in= moipProblem.attributeMatrix
        self.sparseInequationsMapList = moipProblem.sparseInequationsMapList
        self.sparseEquationsMapList = moipProblem.sparseEquationsMapList
        self.x_up = np.empty([len(self.attributeMatrix_in),len(self.attributeMatrix_in[0])])
        self.y_up = np.empty([len(self.attributeMatrix_in),len(self.attributeMatrix_in)])
        self.y_ub = np.empty(len(self.attributeMatrix_in))
        self.y_lb = np.empty(len(self.attributeMatrix_in))
        self.xVar = cplex.variables
        
    def calculate(self):
        utopiaSols = {}
        for i in range(0,len(self.attributeMatrix_in)):
            target = self.attributeMatrix_in[i]
            (rsltObj,rsltXvar,rsltSolString) = UtopiaPlane.intlinprog(self.cplex, self.xVar, target, self.sparseInequationsMapList, self.sparseEquationsMapList, None, None)
            
    
    @classmethod
    def intlinprog(cls, cplex, xVar, target, sparseInequationsMapList, sparseEquationsMapList, lb, up):
        variableCount =  xVar.get_num()
        constCounter = 0 
        tempConstrList = []
        #add the temp inequation constraints to the solver
        for ineqlDic in sparseInequationsMapList:
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
            constName= 'new_c'+str(constCounter)
            indices = cplex.linear_constraints.add(lin_expr = rows, senses = 'L', rhs = [rs], names = [constName] )
            tempConstrList.append(constName)
        #add the temp equation constraints to the solver
        for eqlDic in sparseEquationsMapList:
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
            constName= 'new_c'+str(constCounter)
            indices = cplex.linear_constraints.add(lin_expr = rows, senses = 'E', rhs = [rs], names = [constName] )
            tempConstrList.append(constName)
        #reset the objective
        cplex.objective.set_name("tempObj")
        coff= target.tolist()
        indics= list(range(0,variableCount))
        cplex.objective.set_linear(zip(indics,coff))
        cplex.objective.set_sense(cplex.objective.sense.minimize)
        #solve the problem
        cplex.solve()
      
        rsltXvar = []
        rsltObj = float("-inf")
        rsltSolString = cplex.solution.get_status_string()
        if(rsltSolString.find("optimal")>=0):
            #bug fixed here, rsltSol should not be returned as the constraints will be modified at the end of the method
            #rsltSol = self.solver.solution
            rsltXvar = cplex.solution.get_values()
            rsltObj =  cplex.solution.get_objective_value()
        #remove the temp constraints
        cplex.linear_constraints.delete(tempConstrList)
        return (rsltObj,rsltXvar,rsltSolString)
