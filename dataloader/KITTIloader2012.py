import torch.utils.data as data

from PIL import Image
import os
import os.path
import numpy as np
from . import kittiutils

IMG_EXTENSIONS = [
    '.jpg', '.JPG', '.jpeg', '.JPEG',
    '.png', '.PNG', '.ppm', '.PPM', '.bmp', '.BMP',
]


def is_image_file(filename):
    return any(filename.endswith(extension) for extension in IMG_EXTENSIONS)

def dataloader(filepath):

  left_fold  = 'colored_0/'
  right_fold = 'colored_1/'
  disp_noc   = 'disp_occ/'
  calib_folder = 'calib/'
  
  calib_files = os.listdir(os.path.join(filepath,calib_folder))
  
  image = [img for img in os.listdir(filepath+left_fold) if img.find('_10') > -1]

  train = image[:]

  left_train  = [filepath+left_fold+img for img in train]
  right_train = [filepath+right_fold+img for img in train]
  disp_train = [filepath+disp_noc+img for img in train]
  calib_train=[]
  for t in train:
      ff=t.split('_')[0]
      

      for cc in calib_files:
          if ff not in cc:
              continue

          data={}
          calib_filepath = os.path.join(filepath,calib_folder,cc)
          filedata = kittiutils.read_calib_file2(calib_filepath)
          P_rect_00 = np.reshape(filedata['P0'], (3, 4))
          P_rect_10 = np.reshape(filedata['P1'], (3, 4))
          P_rect_20 = np.reshape(filedata['P2'], (3, 4))
          P_rect_30 = np.reshape(filedata['P3'], (3, 4))
          data['P_rect_00'] = P_rect_00
          data['P_rect_10'] = P_rect_10
          data['P_rect_20'] = P_rect_20
          data['P_rect_30'] = P_rect_30
          
           # Compute the camera intrinsics
          data['K_cam0'] = P_rect_00[0:3, 0:3]
          data['K_cam1'] = P_rect_10[0:3, 0:3]
          data['K_cam2'] = P_rect_20[0:3, 0:3]
          data['K_cam3'] = P_rect_30[0:3, 0:3]
          
          a1=data['P_rect_20'][0,3]/data['P_rect_20'][0,0]
          a2=data['P_rect_30'][0,3]/data['P_rect_30'][0,0]
          b_color = np.linalg.norm(a1-a2)
          data['b_color'] = b_color
          Q=np.zeros((4,4))
          
          Q[0,0]=1
          Q[1,1]=1
          Q[0,3]=-data['K_cam2'][0,2]
          Q[1,3]=-data['K_cam2'][1,2]
          Q[2,3]=data['K_cam2'][0,0]
          Q[3,2]=1/data['b_color']
          Q[3,3] = -(data['K_cam2'][0,2]-data['K_cam3'][0,2])/data['b_color']
          
          data['Q'] = Q
          
          calib_train.append(data)


  return left_train, right_train, disp_train,calib_train
