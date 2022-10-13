from direct.showbase.ShowBase import ShowBase

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
