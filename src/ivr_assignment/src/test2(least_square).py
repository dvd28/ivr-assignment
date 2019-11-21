
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


print(Base)
print(Final)


def mat(theta):
	

	yellow_to_green=np.array([(np.cos(theta[0])*np.cos(theta[2])+np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]))*Base[0]+((-1)*np.sin(theta[0])*np.cos(theta[1])+(-1)*np.cos(theta[0])*np.sin(theta[2]))*Base[1]+(np.sin(theta[0])*np.cos(theta[2])*np.sin(theta[1]))*Base[2]-Final[0],(np.sin(theta[0])*np.cos(theta[2])-np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]))*Base[0]+(np.cos(theta[0])*np.cos(theta[1]))*Base[1]+((-1)*np.sin(theta[0])*np.sin(theta[2])-np.cos(theta[0])*np.cos(theta[2])*np.sin(theta[1]))*Base[1]-Final[1],(np.sin(theta[2])*np.cos(theta[1]))*Base[0]+(np.sin(theta[1]))*Base[1]+(np.cos(theta[1])*np.cos(theta[2]))*Base[2]+5*Base[3]-Final[2],0+0+0+1*Base[3]-Final[3]])
	#print(yellow_to_green)
	yellow_to_red=np.array([0,0,0,0])
	Green=np.array([0.0,0.0,0.0,0.0])
	#print(yellow_to_green)
	

	return yellow_to_green

theta0=[0.0,0.0,0.0,0.0]
res_1 = least_squares(mat,theta0)

print(res_1.x)
print(res_1)

