# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 16:46:37 2018

@author: Yinxing Xue
"""
import moipSol 
import math

class NaiveSol(BaseSol):  
    
    solveCounter = 0
    objConstrIndexList = []
    boundsDict = {}
 
    def __init__(self, moipProblem):  
        #调用父类的构函  
        BaseSol.__init__(self,moipProblem)  
       
    #覆写父类的方法  
    def execute(self):  
        print("%s" % ("Starting solving the problem with epislon-constraint!")) 
        constCounter = self.solver.linear_constraints.get_num()
        self.objConstrIndexList =[]
        
        #dictionary for the UB and LB of each objective
        self.boundsDict = {}
        for k in range(1,len(self.moipProblem.attributeMatrix)):
            kthObj= self.moipProblem.attributeMatrix[k]
            ub,lb = self.calculteUBLB(kthObj)
            self.boundsDict[k]= (ub,lb)
            
        #convert the k-th objective as constraint 
        for k in range(1,len(self.moipProblem.attributeMatrix)):
            kthObj= self.moipProblem.attributeMatrix[k]
            #add the constraint
            if  len(kthObj)!=  self.moipProblem.featureCount:
                 raise Exception('input not consistent', 'eggs')
            rows = []
            (ub,lb)=  self.boundsDict[k]
            rs1 = lb
            rs2 = ub
            variables = []
            coefficient = []
            for index in range(0,len(kthObj)): 
                variables.append('x'+str(index))
                coefficient.append(kthObj[index])
            row=[]
            row.append(variables)
            row.append(coefficient)
            rows.append(row)       
            #constCounter= constCounter+1
            #one objective constraint  has two parts, need to be added seperately
            constName= 'o'+str(k)+'_R'
            indices = self.solver.linear_constraints.add(lin_expr = rows, senses = 'L', rhs = [rs2], names = [constName])
            self.objConstrIndexList.append(constName)
            self.constrIndexList.append(indices)
            #add the left part
            constName= 'o'+str(k)+'_L'
            indices = self.solver.linear_constraints.add(lin_expr = rows, senses = 'G', rhs = [rs1], names = [constName])
            self.objConstrIndexList.append(constName)
            self.constrIndexList.append(indices)
        
        #start to solving
        self.travelAllObjConstr(1)
    
    """
    level:
    passDict: the dictionary to travese, the key is the k-th objective, the value is the adjusted UB
    """
    def travelAllObjConstr(self, level = 1, passDict={}):
        if level == self.moipProblem.objectiveCount-1:
            #no need to go deep
            (ub,lb)=  self.boundsDict[level]
            ub_relaxed= math.ceil(ub)
            lb_relaxed= math.floor(lb)
            for value in range(ub_relaxed,lb_relaxed-1,-1):
                passDict['o'+str(level)+'_R']=value
                self.updateSolver(passDict)
                self.solveCounter += 1
                print (self.solveCounter)
                continue
                self.solver.solve()
                #debugging purpose
                #print ("Solution value  = ", self.solver.solution.get_objective_value())
                #debugging purpose
                #xsol = self.solver.solution.get_values()
                #debugging purpose
                #print ('xsol = ',  xsol )
                if(self.solver.solution.get_status_string()!='optimal'):
                    continue
                cplexResults = CplexSolResult(self.solver.solution,self.moipProblem)
                self.cplexSolutionSet.append(cplexResults)
                self.cplexResultMap[cplexResults.getResultID()] = cplexResults
        else: 
            (ub,lb)=  self.boundsDict[level]
            ub_relaxed= math.ceil(ub)
            lb_relaxed= math.floor(lb)
            for value in range(ub_relaxed,lb_relaxed-1,-1):
                passDict['o'+str(level)+'_R']=value
                self.travelAllObjConstr(level+1,passDict) 
                
    def updateSolver(self,passDict):
        for constrName in passDict:
            self.solver.linear_constraints.set_rhs(constrName, passDict[constrName])
        
    def calculteUBLB(self,obj):
        ub = 0.0
        lb = 0.0
        for value in obj:
            if value > 0:
                ub = ub+ value
            else:
                lb = lb + value
        return ub, lb
    
    def displaySolvingAttempts(self):
        print ("Total Sovling Attempts: %s" % self.solveCounter) 
      
    def displayObjsBoundsDictionary(self):
        print ("Objectives' Bound Dictionary: %s" % self.boundsDict) 
        
    
if __name__ == "__main__":
    prob = MOIPProblem(4,12,3)  
    prob.displayObjectiveCount()
    prob.displayFeatureCount()
    prob.exetractFromFile("parameter_js.txt")
    prob.displayObjectives()
    prob.displayVariableNames()
    prob.displayObjectiveSparseMapList()
    prob.displaySparseInequationsMapList()
    prob.displaySparseEquationsMapList()
    prob.displayAttributeMatrix()
    
    sol= NaiveSol(prob)
    sol.prepare()
    sol.execute()
    sol.displaySolvingAttempts()
    sol.displayObjsBoundsDictionary()
    sol.displayCplexSolutionSetSize()
    sol.displayCplexResultMap()
    sol.displayVariableLowerBound()
    sol.displayVariableUpperBound()
    sol.displayVariableTypes()
    sol.displayVariableNames()
else:
    print("naiveSol.py is being imported into another module")
