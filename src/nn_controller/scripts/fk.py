import math
import numpy as np
import time

#import PyKDL

class FK(object):
    
    def __init__(self):
        aa = (30.0*math.pi)/180.0
        D1 = 0.2755
        D2 = 0.29
        D3 = 0.1233
        D4 = 0.0741
        D5 = 0.0741
        D6 = 0.1600
        e2 = 0.0070
        d4b = D3 + math.sin(aa)/math.sin(2*aa)*D4
        d5b = math.sin(aa)/math.sin(2*aa)*D4 + math.sin(aa)/math.sin(2*aa)*D5
        d6b = math.sin(aa)/math.sin(2*aa)*D5 + D6
       
        # Modified DH (Craig) parameters
        self.alpha = [0, -math.pi/2, 0, -math.pi/2, 2*aa, 2*aa]
        self.a = [0, 0, D2, 0, 0, 0]
        self.d = [D1, 0, e2, d4b, d5b, d6b]        
        
        
    def getTransformMatrix(self, jointNr, beta):
        jointNr = jointNr - 1
        
        a = self.a[jointNr]
        alpha = self.alpha[jointNr]
        d = self.d[jointNr]
        # Create modified DH transformation matrix 
        matrix = [[math.cos(beta), -math.sin(beta), 0, a],
                  [math.sin(beta)*math.cos(alpha), math.cos(beta)*math.cos(alpha), -math.sin(alpha), -d*math.sin(alpha)],
                  [math.sin(beta)*math.sin(alpha), math.cos(beta)*math.sin(alpha), math.cos(alpha), d*math.cos(alpha)],
                  [0, 0, 0, 1]]
        return matrix
        
    
    def getEEFPosition(self, joint1 = 0, joint2 = 180.0, joint3 = 180.0, joint4 = 0, joint5 = 0, joint6 = 0):
      
        begin = time.time()
        T1 = np.asarray(self.getTransformMatrix(1, math.radians(-joint1) + math.radians(180)), dtype = np.float32)
        T2 = np.asarray(self.getTransformMatrix(2, math.radians(joint2) - math.radians(270)), dtype = np.float32)
        T3 = np.asarray(self.getTransformMatrix(3, math.radians(-joint3) + math.radians(90)), dtype = np.float64)
        T4 = np.asarray(self.getTransformMatrix(4, math.radians(-joint4) + math.radians(180)), dtype = np.float64)       
        T5 = np.asarray(self.getTransformMatrix(5, math.radians(-joint5) + math.radians(180)), dtype = np.float64)
        T6 = np.asarray(self.getTransformMatrix(6, math.radians(-joint6) + math.radians(180+90)), dtype = np.float64)
        
        Ttemp = np.dot(T5, T6)
        Ttemp = np.dot(T4,Ttemp)
        Ttemp = np.dot(T3,Ttemp)
        Ttemp = np.dot(T2,Ttemp)
        Tfinal = np.dot(T1,Ttemp)        
        
        return (Tfinal[0][3], Tfinal[1][3], Tfinal[2][3])


if __name__ =="__main__":
    fk = FK()
    print fk.getEEFPosition(joint1 = 90, joint2 = 270, joint3 = 220)        
        