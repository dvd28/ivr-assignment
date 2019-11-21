
import image1
import image2

import numpy as np
import math 
import scipy
import scipy.optimize
from scipy.optimize import least_squares
from scipy.optimize import minimize
import ast
import pickle



with open ('positions1.txt', 'rb') as fp:
    itemlist = pickle.load(fp)

p0=itemlist[3]
p1=itemlist[2]
p2=itemlist[1]
p3=itemlist[0]


with open ('positions2.txt', 'rb') as fp:
    itemlist = pickle.load(fp)

s0=itemlist[3]
s1=itemlist[2]
s2=itemlist[1]
s3=itemlist[0]



base=np.array([s0[0],p0[1],p0[2]])
final=np.array([s2[0],p2[1],p2[2]])
Base=np.append(base,1)
Final=np.append(final,1)
Final=np.transpose(Final)
Base = Base.astype(np.float)
Final = Final.astype(np.float)


def objective(theta):

	yellow_to_green=np.array([(np.cos(theta[0])*np.cos(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]))*Base[0]+((-1)*np.sin(theta[0])*np.cos(theta[1])+(-1)*np.cos(theta[0])*np.sin(theta[2]))*Base[1]+(np.sin(theta[0])*np.cos(theta[2])*np.sin(theta[1]))*Base[2],(np.sin(theta[0])*np.cos(theta[2])-np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]))*Base[0]+(np.cos(theta[0])*np.cos(theta[1]))*Base[1]+((-1)*np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.cos(theta[2])*np.sin(theta[1]))*Base[1],(np.sin(theta[2])*np.cos(theta[1]))*Base[0]+(np.sin(theta[1]))*Base[1]+(np.cos(theta[1])*np.cos(theta[2]))*Base[2]+5*Base[3],0+0+0+1*Base[3]])
	yellow=Final
	print(np.shape(yellow))

	dist = np.linalg.norm(yellow_to_green-Final)
	return dist

	
theta0=[0.0,0.0,0.0,0.0]
#print(objective)
sol=minimize(objective,theta0,method='SLSQP')

#print(res_1.x)
print(sol)
print(sol.x)
#print(objective(sol.x))
