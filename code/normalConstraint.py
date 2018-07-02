# -*- coding: utf-8 -*-
"""
Created on Sat Jun 30 13:44:12 2018

@author: Yinxing Xue
"""
import numpy as np
from moipSol import CplexSolResult

class UtopiaPlane():
    
    TOL = 1e-5
    
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
        self.moipProblem = moipProblem
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
            target = np.array(self.attributeMatrix_in[i])
            solutions = UtopiaPlane.intlinprog(self.cplex, self.xVar, target, self.sparseInequationsMapList, self.sparseEquationsMapList, None, None)
            solution = solutions[0]
            cplexResult = CplexSolResult(solution[1],"optimal",self.moipProblem)
            if cplexResult.getResultID() not in utopiaSols:
                utopiaSols[cplexResult.getResultID()] = cplexResult
            elif cplexResult.getResultID() in utopiaSols and len(solutions)>1:
                for j in (1,len(solutions)):
                    solution2 = solutions[j]
                    cplexResult2 = CplexSolResult(solution2[1],"optimal",self.moipProblem)
                    if cplexResult2.getResultID() not in utopiaSols:
                        utopiaSols[cplexResult2.getResultID()] = cplexResult2
                        break
                    #end of if
                #end of for
            #end of if-elif
            X = np.array(cplexResult.xvar)
            self.x_up[i] = X
            self.y_up[i] = UtopiaPlane.calculateObjs(cplexResult.xvar,self.attributeMatrix_in)
            y1 = solution[0]
            self.y_lb[i] = y1
            
            negTarget = target * (-1.0)
            negSolutions = UtopiaPlane.intlinprog(self.cplex, self.xVar, negTarget, self.sparseInequationsMapList, self.sparseEquationsMapList, None, None)
            negSolution = negSolutions[0]
            X2 = negSolution[1]
            y2 = negSolution[0]* (-1.0)
            self.y_ub[i] = y2;
        #end of for
    
    @classmethod
    def calculateObjs(cls,xval,objMatrix):
        # dimension: k* variableCount
        matA = np.array(objMatrix)
        twoDArray= []
        twoDArray.append(xval)
        # dimension: variableCount * 1
        matB = np.array(twoDArray).transpose()
        allObjs= np.dot(matA, matB)
        allObjs = allObjs.transpose()
        allObjsList = allObjs.tolist()
        return allObjsList[0]
    
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
            #indices: test purpose 
            indices = cplex.linear_constraints.add(lin_expr = rows, senses = 'L', rhs = [rs], names = [constName] )
            print (indices)
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
            #indices: test purpose 
            indices = cplex.linear_constraints.add(lin_expr = rows, senses = 'E', rhs = [rs], names = [constName] )
            print (indices)
            tempConstrList.append(constName)
        #reset the objective
        cplex.objective.set_name("tempObj")
        coff= target.tolist()
        indics= list(range(0,variableCount))
        cplex.objective.set_linear(zip(indics,coff))
        cplex.objective.set_sense(cplex.objective.sense.minimize)
        #reset the parameters 
        cplex.parameters.mip.pool.absgap.set(0.5)
        cplex.parameters.mip.pool.intensity.set(4)
        cplex.parameters.mip.pool.capacity.set(5)
        cplex.parameters.mip.limits.populate.set(10)
        cplex.parameters.mip.pool.replace.set(1)
        cplex.populate_solution_pool()
        
        solutionsNames = cplex.solution.pool.get_names()
        solutions = []
        opt = []
        #Create a container for the indices of optimal solutions.
        best = float("-inf")
        """
	     * Check which pool solutions are truly optimal; if the pool capacity
	     * exceeds the number of optimal solutions, there may be suboptimal
	     * solutions lingering in the pool.
        """
        for i in range(0,len(solutionsNames)):
            z = cplex.solution.pool.get_objective_value(i)
            # If this solution is better than the previous best, the previous
            # solutions must have been suboptimal; drop them all and count this one.
            if(z > best + UtopiaPlane.TOL):
                best = z 
                opt.clear()
                opt.append(i)
            #If this solution is within rounding tolerance of optimal, count it.
            elif z > best - UtopiaPlane.TOL: 
                opt.append(i)
                
        for i in opt:
            rsltObj = cplex.solution.pool.get_objective_value(i)
            rsltXvar =  cplex.solution.pool.get_values(i)
            solution = (rsltObj,rsltXvar)
            solutions.append(solution)
        
        cplex.linear_constraints.delete(tempConstrList)
        return solutions
