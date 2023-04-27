from numpy import *
from matplotlib import ploty

class main(object):
    def __init__(self):
        self.Camber = self.gen_naca(1549)
        self.V_inf = 30 #m/s

    def mainloop(self):
        self.array = []
        for alpha in range(0,10,0.1):
            self.array.append(self.compute_Pressure_Distribution(self.V_inf, alpha))


    def compute_Pressure_Distribution(self, v, alpha):
        for i in range(0,2*pi):

    def gen_naca(self, foil_num): #genrates 4 digit naca air foils
        self.foil_num = str(foil_num)
        self.max_camber = int(self.foil_num[0])/100
        self.max_camber_pos = int(self.foil_num[1])/10
        self.thickness_ratio = int(self.foil_num[2:])/100
        self.camber_line = []
        for x in range(100,0,-1):
            self.pp = x/100
            if self.pp<=self.max_camber_pos:
                if self.max_camber != 0:
                    self.camber_offset = (self.max_camber/self.max_camber_pos**2)*(2*self.max_camber_pos*self.pp-self.pp**2)
                else:
                    self.camber_offset = 0
            else:
                if self.max_camber!=0:
                    self.camber_offset = (self.max_camber/(1-self.max_camber_pos)**2)*((1-2*self.max_camber_pos)+2*self.max_camber_pos*self.pp-self.pp**2)
                else:
                    self.camber_offset = 0
            self.camber_line.append([x, self.camber_offset])


        for x in range(0,100,1):
            self.pp = x/100

            if self.pp<=self.max_camber_pos:
                if self.max_camber!=0:
                    self.camber_offset = (self.max_camber/self.max_camber_pos**2)*(2*self.max_camber_pos*self.pp-self.pp**2)

                else:
                    self.camber_offset = 0

            else:
                if self.max_camber!=0:
                    self.camber_offset = (self.max_camber/(1-self.max_camber_pos)**2)*((1-2*self.max_camber_pos)+2*self.max_camber_pos*self.pp-self.pp**2)

                else:
                    self.camber_offset = 0
                self.camber_line.append([x, self.camber_offset])

        return self.camber_line

if __name__ == "-_main__":
    main()
