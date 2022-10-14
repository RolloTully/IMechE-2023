from direct.showbase.ShowBase import ShowBase
import numpy as np
import cv2
from tqdm import tqdm
from perlin_noise import PerlinNoise



cv2.imshow("noise", np.array(pic))#cv2.resize(img,(1000,1000),interpolation = cv2.INTER_LANCZOS4  ))
cv2.waitKey(3000)






class Tools(object):
    def __init__(self):

    def gen_terrain(self):
        self.layers = np.array([PerlinNoise(octaves=oct) for oct in [3,6,12,24]])
        self.sampling = self.layers[:, i/xpix, j/ypix ]
        noise1 = PerlinNoise(octaves=3)
        noise2 = PerlinNoise(octaves=6)
        noise3 = PerlinNoise(octaves=12)
        noise4 = PerlinNoise(octaves=24)

        xpix, ypix = 1000,1000
        pic = []
        for i in range(xpix):
            row = []
            for j in range(ypix):
                noise_val = noise1([i/xpix, j/ypix])
                noise_val += 0.5 * noise2([i/xpix, j/ypix])
                noise_val += 0.25 * noise3([i/xpix, j/ypix])
                noise_val += 0.125 * noise4([i/xpix, j/ypix])

                row.append(noise_val)
            pic.append(row)
class MyApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)

        # Load the environment model.
        self.camLens.setFov(62.2,48.8)
        self.scene = self.loader.loadModel("models/environment")
        self.scene.reparentTo(self.render)

        self.terrain = GeoMipTerrain("mySimpleTerrain")
        self.terrain.setHeightfield("yourHeightField.png")
        self.terrain.setBruteforce(True)
        self.terrain.getRoot().reparentTo(self.render)
        self.terrain.generate()
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, -4)


app = MyApp()
app.run()
