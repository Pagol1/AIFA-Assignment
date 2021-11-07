# Upscale Image
import numpy as np 

def upConstant(origImg: np.ndarray,scale: int=None)->np.ndarray:
    h,w = origImg.shape[0],origImg.shape[1]
    if len(origImg.shape) == 3:
        channel = origImg.shape[2]
    else:
        channel = 1
    temp = np.zeros((scale*h,scale*w,channel))
    for i in range(h):
        for j in range(w):
            for r in range(channel):
                temp[i*scale:(i+1)*scale,j*scale:(j+1)*scale,r] = origImg[i,j,r].copy()
    return temp

