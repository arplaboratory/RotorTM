import numpy as np
import numpy.linalg as LA

def vec2asym(vec):
  if  (type(vec) is np.ndarray):
    if len(vec.shape) == 1:
      row = vec.shape[0]
      col = 0
    elif len(vec.shape) == 2:
      row = vec.shape[0]
      col = vec.shape[1]
  elif type(vec) is list:
      row = len(vec)
      col = 0
  else:
    raise Exception("The vector type not list or numpy array")

  if row == 3: 
    if col == 0:
      mat = np.array([[0,-vec[2],vec[1]],
                      [vec[2],0,-vec[0]],
                      [-vec[1],vec[0],0]])
    elif col == 1:
      mat = np.array([[0,-vec[2][0],vec[1][0]],
                      [vec[2][0],0,-vec[0][0]],
                      [-vec[1][0],vec[0][0],0]])
  else:
    raise Exception("The vector shape is not 3")
  
  return mat

def QuatToRot(quat):
  if  (type(quat) is np.ndarray):
    q = quat
  elif type(quat) is list:
    q = np.array(quat)
  else:
    raise Exception("The vector type not list or numpy array")

  q = q/np.sqrt(np.sum(q**2))

  qahat = np.zeros((3,3))
  qahat[0,1] = -q[3]
  qahat[0,2] =  q[2]
  qahat[1,2] = -q[1]
  qahat[1,0] =  q[3]
  qahat[2,0] = -q[2]
  qahat[2,1] =  q[1]

  R = np.eye(3) + 2*np.matmul(qahat,qahat) + 2*q[0]*qahat

  return R

def quat_dot(quat,omg):

  qW = quat[0]
  qX = quat[1]
  qY = quat[2]
  qZ = quat[3]

  p = omg[0]
  q = omg[1]
  r = omg[2]

  K_quat = 2
  quaterror = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)
  #qLdot = - 1 / 2 * np.array([[0,- p,- q,- r],[p,0,- r,q],[q,r,0,- p],[r,- q,p,0]]) * quat + K_quat * quaterror * quat
  qLdot = 1 / 2 * np.matmul(np.array([[0,- p,- q,- r],[p,0,r,-q],[q,-r,0,p],[r,q,-p,0]]), quat) + K_quat * quaterror * quat

  return qLdot

def vecnorm(A, p=2, dim=0):
  # take 2-norm along 0 dimension of A
  if p==2:
    if dim==0:
      result = np.empty((1, A.shape[0]), dtype=float)
      for i in range(A.shape[1]):
        result[0, i] = np.linalg.norm(A[:, i])
    elif dim==1:
      result = np.empty((A.shape[0], 1), dtype=float)
      for i in range(A.shape[0]):
        result[i, 0] = np.linalg.norm(A[i, :])
