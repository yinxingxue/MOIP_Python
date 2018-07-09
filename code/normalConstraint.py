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
        self.extraInequationsMapList = [] 
        self.extraEquationsMapList = []
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
        self.extraInequationsMapList = []
        self.extraEquationsMapList = []
        self.x_up = np.empty([len(self.attributeMatrix_in),len(self.attributeMatrix_in[0])])
        self.y_up = np.empty([len(self.attributeMatrix_in),len(self.attributeMatrix_in)])
        self.y_ub = np.empty(len(self.attributeMatrix_in))
        self.y_lb = np.empty(len(self.attributeMatrix_in))
        self.xVar = cplex.variables
        
    def calculate(self):
        utopiaSols = {}
        for i in range(0,len(self.attributeMatrix_in)):
            target = np.array(self.attributeMatrix_in[i])
            solutions = UtopiaPlane.intlinprog(self.cplex, self.xVar, target, self.extraInequationsMapList, self.extraEquationsMapList, None, None)
            solution = solutions[0]
            optSolution = None
            cplexResult = CplexSolResult(solution[1],"optimal",self.moipProblem)
            if cplexResult.getResultID() not in utopiaSols or len(solutions)==1:
                optSolution = cplexResult
                utopiaSols[optSolution.getResultID()] = optSolution
            elif cplexResult.getResultID() in utopiaSols and len(solutions)>1:            
                best = float("+inf")
                for j in range(1,len(solutions)):
                    solution = solutions[j]
                    cplexResult2 = CplexSolResult(solution[1],"optimal",self.moipProblem)
                    if cplexResult2.getResultID() not in utopiaSols and cplexResult2.getThisObj() < best:
                        best = cplexResult2.getThisObj() 
                        optSolution = cplexResult2
                    #end of if
                #end of for
                utopiaSols[optSolution.getResultID()] = optSolution
            #end of if-elif
            X = np.array(optSolution.xvar)
            self.x_up[i] = X
            self.y_up[i] = UtopiaPlane.calculateObjs(optSolution.xvar,self.attributeMatrix_in)
            y1 = optSolution.getKthObj(i)
            self.y_lb[i] = y1
            
            negTarget = target * (-1.0)
            negSolutions = UtopiaPlane.intlinprog(self.cplex, self.xVar, negTarget, self.extraInequationsMapList, self.extraEquationsMapList, None, None)
            negSolution = negSolutions[0]
            X2 = negSolution[1]
            y2 = negSolution[0]* (-1.0)
            self.y_ub[i] = y2;
        #end of for
        return
    
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

class SolRep():
    'define the calculation and implementation of Representative Solutions'
    def __init__(self, y_up, varNo, n):
        #input
        self.attributeMatrix_in  = [[]]
        self.y_up = y_up
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
        self.varNo = varNo
        self.n = n
        #output
        self.P = [[]]
        self.xVar = []
        
    def calculate(self):
        No = len(self.y_up)
        Nv = self.varNo
        #generate the constant matrix V
        ConstantMatrix.initialize(self.y_up, No)
        V = ConstantMatrix.V
        #generate constant upper bounds and lower bounds
        ub = [1]*(Nv+No+1)
        ub[Nv:] = float("+inf")
        lb = [0]*(Nv+No+1)
        lb[Nv:] = float("-inf")
        
        self.extra_A = self.origi_A.copy()
        self.extra_B = self.origi_B.copy()
        self.extra_Aeq = self.origi_Aeq.copy()
        self.extra_Beq = self.origi_Beq.copy()
        negEye = np.eye(No,dtype= float) * (-1)
        zeroMat = np.zeros((No,1))
        zeroRightSide = np.zeros((No,1))
        extraEquationsMat= np.column_stack(self.attributeMatrix_in,negEye,zeroMat)
        SolRep.appendToSparseMapList(self.extra_Aeq,extraEquationsMat)
        SolRep.appendToSparseMapList(self.extra_Beq,zeroRightSide)
        cplex = SolRep.initializeCplex(Nv, No, self.extra_A, self.extra_B, self.extra_Aeq, self.extra_Beq, lb, ub) 
        
    def setParas(self, attributeMatrix, origi_A , origi_B, origi_Aeq, origi_Beq):
        self.attributeMatrix_in = attributeMatrix
        #need to clone
        self.origi_A = origi_A
        self.origi_B = origi_B
        self.origi_Aeq = origi_Aeq
        self.origi_Beq = origi_Beq
  
    @classmethod  
    def appendToSparseMapList(cls, mapList, matrix):
        
        return        
    
    @classmethod    
    def initializeCplex(cls, Nv, No, extraInequationsMapList, extraEquationsMapList, lb, ub):
        cplex = None
        return cplex

class ConstantMatrix():
    'define the calculation and implementation of Constant Matrix'
    V = None
    
    @classmethod
    def initialize(cls, y_up, No):
        V = np.zeros((No-1, No))
        for i in range(0,No-1):
            v_i = y_up[len(y_up)-1] - y_up[i]
            norm_2 = np.linalg.norm(v_i,ord=2,keepdims=True)
            if norm_2 != 0 :
                v_i = v_i  / norm_2
            V[i] = v_i
    