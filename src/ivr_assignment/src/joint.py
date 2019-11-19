import numpy as np

import math



def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def rotation_matrix(angle, direction, point=None):
    """Return matrix to rotate about axis defined by point and direction.

    >>> R = rotation_matrix(math.pi/2, [0, 0, 1], [1, 0, 0])
    >>> numpy.allclose(numpy.dot(R, [0, 0, 0, 1]), [1, -1, 0, 1])
    True
    >>> angle = (random.random() - 0.5) * (2*math.pi)
    >>> direc = numpy.random.random(3) - 0.5
    >>> point = numpy.random.random(3) - 0.5
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> R0 = rotation_matrix(angle, direc, point)
    >>> R1 = rotation_matrix(-angle, -direc, point)
    >>> is_same_transform(R0, R1)
    True
    >>> I = numpy.identity(4, numpy.float64)
    >>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
    True
    >>> numpy.allclose(2, numpy.trace(rotation_matrix(math.pi/2,
    ...                                               direc, point)))
    True

    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([[ 0.0,         -direction[2],  direction[1]],
                      [ direction[2], 0.0,          -direction[0]],
                      [-direction[1], direction[0],  0.0]])
    M = np.identity(3)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M



def get_joint_angle(pos):
	#position vectors array 3*3Array, v1,v2,v3
    pos0=pos[0].flatten()
    pos1=pos[1].flatten()
    pos2=pos[2].flatten()
    pos3=pos[3].flatten()
    vector=[pos[1,:]-pos[0,:],pos[2,:]-pos[1,:],pos[3,:]-pos[2,:]]
    vectors=np.asarray(vector).reshape((3,3))
    theta=[0.0,0.0,0.0,0.0]
    theta[0]=angle_between(vectors[0],[0.0,0.0,1.0])
    R1=rotation_matrix(theta[0],np.array([0,0,1]))

    #this is the compared vector Zi
    Xi=np.dot(R1,np.transpose(np.array([1,0,0])))
    Yi=np.dot(R1,np.transpose(np.array([0,1,0])))
    Zi=np.dot(R1,np.transpose(np.array([0,0,1])))

    theta[1]=angle_between([vectors[1][1],vectors[1][2]],[Zi[1],Zi[2]])
    theta[2]=angle_between([vectors[1][0],vectors[1][2]],[Zi[0],Zi[2]])
    R21=rotation_matrix(theta[1],np.transpose(Xi))
    Yi1=np.dot(np.dot(R1,R21),np.transpose(np.array([0,1,0])))
    R22=rotation_matrix(theta[2],np.transpose(Yi1))

    Zi2=np.dot(np.dot(np.dot(R1,R21),R22),np.transpose(np.array([0,0,1])))
  
    theta[3]=angle_between(vectors[2],np.transpose(Zi2)) 
    return np.array(theta)

#comments below are test codes
#s=np.array([[0,0,0],[0,0,2],[0,-1.5,3.5],[0,-2.5,4.5]])
#k=get_joint_angle(s)
#print(k)

#when using angle_between to determine angles,we use the Z axis as the comparied axis
