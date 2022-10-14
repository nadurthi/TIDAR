import numpy as np
import matplotlib.pyplot as plt

objp_checker = np.zeros((5*4,3), np.float32)
objp_checker[:,:2] = np.mgrid[0:4,0:5].T.reshape(-1,2)
objp_checker = objp_checker*0.175




dia = 0.065
gap = 0.140
dy = gap*np.sqrt(2)
dx = dy/2
objp_acircular4_11 = np.zeros((44, 3), np.float32)

objp_acircular4_11[0]  = (0  , 0  , 0)
objp_acircular4_11[1]  = (0  , dy , 0)
objp_acircular4_11[2]  = (0  , 2*dy, 0)
objp_acircular4_11[3]  = (0  , 3*dy, 0)

objp_acircular4_11[4]  = (dx , dx , 0)
objp_acircular4_11[5]  = (dx , dx+dy, 0)
objp_acircular4_11[6]  = (dx , dx+2*dy, 0)
objp_acircular4_11[7]  = (dx , dx+3*dy, 0)


objp_acircular4_11[8]  = (2*dx , 0 , 0)
objp_acircular4_11[9]  = (2*dx , dy, 0)
objp_acircular4_11[10]  = (2*dx , 2*dy, 0)
objp_acircular4_11[11]  = (2*dx , 3*dy, 0)

objp_acircular4_11[12]  = (3*dx , dx, 0)
objp_acircular4_11[13]  = (3*dx , dx+dy, 0)
objp_acircular4_11[14]  = (3*dx , dx+2*dy, 0)
objp_acircular4_11[15]  = (3*dx , dx+3*dy, 0)

objp_acircular4_11[16]  = (4*dx , 0 , 0)
objp_acircular4_11[17]  = (4*dx , dy, 0)
objp_acircular4_11[18]  = (4*dx , 2*dy, 0)
objp_acircular4_11[19]  = (4*dx , 3*dy, 0)

objp_acircular4_11[20]  = (5*dx , dx , 0)
objp_acircular4_11[21]  = (5*dx , dx+dy, 0)
objp_acircular4_11[22]  = (5*dx , dx+2*dy, 0)
objp_acircular4_11[23]  = (5*dx , dx+3*dy, 0)

objp_acircular4_11[24]  = (6*dx , 0 , 0)
objp_acircular4_11[25]  = (6*dx , dy, 0)
objp_acircular4_11[26]  = (6*dx , 2*dy, 0)
objp_acircular4_11[27]  = (6*dx , 3*dy, 0)

objp_acircular4_11[28]  = (7*dx , dx , 0)
objp_acircular4_11[29]  = (7*dx , dx+dy, 0)
objp_acircular4_11[30]  = (7*dx , dx+2*dy, 0)
objp_acircular4_11[31]  = (7*dx , dx+3*dy, 0)

objp_acircular4_11[32]  = (8*dx , 0 , 0)
objp_acircular4_11[33]  = (8*dx , dy, 0)
objp_acircular4_11[34]  = (8*dx , 2*dy, 0)
objp_acircular4_11[35]  = (8*dx , 3*dy, 0)

objp_acircular4_11[36]  = (9*dx , dx , 0)
objp_acircular4_11[37]  = (9*dx , dx+dy, 0)
objp_acircular4_11[38]  = (9*dx , dx+2*dy, 0)
objp_acircular4_11[39]  = (9*dx , dx+3*dy, 0)

objp_acircular4_11[40]  = (10*dx , 0 , 0)
objp_acircular4_11[41]  = (10*dx , dy, 0)
objp_acircular4_11[42]  = (10*dx , 2*dy, 0)
objp_acircular4_11[43]  = (10*dx , 3*dy, 0)

if __name__=="__main__":
    fig,ax = plt.subplots()
    ax.plot(objp_acircular4_11[:,0],objp_acircular4_11[:,1],'ro')
    ax.axis('equal')