from direct.showbase.ShowBase import ShowBase
from panda3d.core import GeoMipTerrain, Texture, TextureStage
from direct.task import Task
import numpy as np
import cv2
from tqdm import tqdm
from perlin_noise import PerlinNoise

class Tools(object):
    def gen_terrain(self):
        self.layers = np.array([PerlinNoise(octaves=self.oct) for self.oct in [3,6,12,24]])
        #self.sampling = self.layers[:, self.i/self.xpix, self.j/self.ypix ]
        self.noise1 = PerlinNoise(octaves=3)
        self.noise2 = PerlinNoise(octaves=6)
        self.noise3 = PerlinNoise(octaves=12)
        self.noise4 = PerlinNoise(octaves=24)
        self.xpix, self.ypix = 513*3,513*3
        self.pic = []
        for self.i in tqdm(range(self.xpix)):
            self.row = []
            for self.j in range(self.ypix):
                self.noise_val = self.noise1([self.i/self.xpix, self.j/self.ypix])
                self.noise_val += 0.5 * self.noise2([self.i/self.xpix, self.j/self.ypix])
                self.noise_val += 0.25 * self.noise3([self.i/self.xpix, self.j/self.ypix])
                self.noise_val += 0.125 * self.noise4([self.i/self.xpix, self.j/self.ypix])

                self.row.append(self.noise_val)
            self.pic.append(self.row)
        self.pic = np.array(self.pic)
        self.pic = self.pic + np.abs(np.min(self.pic))
        self.pic = (self.pic/np.max(self.pic))*255
        print(self.pic)
        print(self.pic.shape)
        cv2.imwrite("yourHeightField.png", self.pic)

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.camLens.setFov(62.2,48.8)
        #self.scene = self.loader.loadModel("models/enviroment")
        #self.scene.reparentTo(self.render)
        self.terrain = GeoMipTerrain("ProceduralTerrain")
        self.terrain.setHeightfield("yourHeightField.png")
        self.terrainTexture = self.loader.loadTexture("grass.jpg")
        self.terrain.setBlockSize(32)
        self.terrain.setNear(20)
        self.terrain.setFar(100)
        self.terrain.setFocalPoint(self.camera)
        # Store the root NodePath for convenience
        self.root = self.terrain.getRoot()
        self.root.setTexture(TextureStage.getDefault(), self.terrainTexture)
        self.root.setTexScale(TextureStage.getDefault(), 50)
        self.root.reparentTo(self.render)
        self.root.setSz(40)
        self.terrain.generate()
        self.taskMgr.add(self.updateTask)
        #self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

    # Define a procedure to move the camera.
    def updateTask(self, task):
        self.terrain.update()
        self.camera.setPos(0,0,120)
        self.camera.setHpr(-45,-1.5,0)
        return task.cont
#Tools().gen_terrain()
#print("here")
app = MyApp()
app.run()
