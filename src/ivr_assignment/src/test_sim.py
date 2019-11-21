
import numpy as np
import math 
import scipy
import scipy.optimize
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


print(Base)
print(Final)

def objective(theta):
	
    M1=np.array([[np.cos(theta[0]),(-1)*np.sin(theta[0]),0,0],[np.sin(theta[0]),np.cos(theta[0]),0,0],[0,0,1.0,2.0],[0,0,0,1.0]])
    M2=np.array([[np.cos(theta[2]),0,(-1)*np.sin(theta[2]),0],[(-1.0)*np.sin(theta[1])*np.sin(theta[2]),np.cos(theta[1]),(-1.0)*np.sin(theta[1])*np.cos(theta[2]),0],[np.sin(theta[2])*np.cos(theta[1]),np.sin(theta[1]),np.cos(theta[1])*np.cos(theta[2]),3.0],[0,0,0,1.0]])
    M3=np.array([[1.0,0,0,0],[0,np.cos(theta[3]),(-1.0)*np.sin(theta[3]),(-2.0)*np.sin(theta[3])],[0.0,np.sin(theta[3]),np.cos(theta[3]),2.0*np.cos(theta[3])],[0.0,0.0,0.0,1.0]])
   
    yellow_to_green=np.dot(M1,M2)
    yellow_to_red=np.dot(yellow_to_green,M3)
    dist = np.linalg.norm(yellow_to_green-Final)
    return dist

	
theta0=[0.0,0.0,0.0,0.0]
#print(objective)
sol=minimize(objective,theta0,method='SLSQP')

#print(res_1.x)
print(sol)
print(sol.x)
#print(objective(sol.x))