# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 10:03:21 2018

@author: Yinxing Xue
"""

import moipProb
prob = MOIPProblem(4,43,3)  
prob.displayObjectiveCount()
prob.displayFeatureCount()
prob.exetractFromFile("parameter.txt")
prob.displayObjectives()
prob.displayVariableNames()
prob.displayObjectiveSparseMapList()
prob.displaySparseInequationsMapList()
prob.displaySparseEquationsMapList()
