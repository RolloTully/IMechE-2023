import cv2
import numpy as np
from random import choice, randint
import string
from tqdm import tqdm
sq = np.zeros((80,80))
kernel = np.array([[0, -1, 0],
               [-1, 10,-1],
               [0, -1, 0]])
sq[20:60,20:60] = 255
for x in tqdm(range(0,200000)):
    c = np.array(list(sq.shape))/2
    bg = np.zeros((1000,1000))
    r_mat = cv2.getRotationMatrix2D(c, randint(0,90),1)
    sq = cv2.warpAffine(sq, r_mat, sq.shape[1::-1])
    x_offset = randint(0,920)
    y_offset = randint(0,920)
    bg[x_offset:x_offset+80,y_offset:y_offset+80] = sq

    bg = cv2.filter2D(src=bg, ddepth=-1, kernel=kernel)
    r_str = ''.join(choice(string.ascii_lowercase) for x in range(0,20))
    name =   "/home/rollosubuntu/Documents/GitHub/IMechE-2023/Software/SITL/masks/"+str(r_str)+".png"
    cv2.imwrite(name, bg)
