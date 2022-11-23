from tkinter import *
import folium
import numpy as np
import maptiles as mt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
#from tkinter import filedialog

#'''A Program to produce the JSON files to command the flight director'''
class Waypoint(object):
    def __init__(self, index, latitude = None, longditude = None, radius= 30, p_rad = 0, altitude = 60, flag = None, cont= None):
        self.mission_index = index   #Mission Item index, defines the order in which the items will be executed
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.radius = radius         #Way point acceptance radius
        self.pass_radius = r_rad        #Direction to pass the way point
        self.altitude = altitude     #Target waypoint altitude
        self.flag = flag             #Waypoint flag
    def position(self):
        return np.array(self.latitude, self.longditude)
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])
class Tools():
    def get_tile(self,box, axis):
        '''Download the map tiles needed'''
        box = (2.289, 48.871, 2.301, 48.876)
        mt.draw_map(bounds = box)
        plt.show()
    def compute_path(self,path):
        '''Finds the pass direction for each waypoint to ensure waypoint rounding'''
        for self.index in range(1,len(path)-1):
            self.path_angle_difference = np.diff([self.path[self.index-1].position()-self.path[self.index].position(), self.path[self.index].position()-self.path[self.index+1].position()])
            self.path_angle_difference[0]/self.path_angle_difference[1]






class GUI(Tk):
    def __init__(self,parent):
        Tk.__init__(self, parent)
        self.title("Mission Designer")
        self.geometry("1000x900+20+20")
        self.tools = Tools()


        '''Latitude input box'''
        Label(self,text="Latitude").place(x=10,y=10)
        self.wing_Diheadral_input= Entry(self, width = 7)
        self.wing_Diheadral_input.insert(END,"0")
        self.wing_Diheadral_input.place(x=10,y=30)

        '''Longditude input box'''
        Label(self,text="Longditude").place(x=100,y=10)
        self.wing_Diheadral_input= Entry(self, width = 7)
        self.wing_Diheadral_input.insert(END,"0")
        self.wing_Diheadral_input.place(x=100,y=30)

        '''Altitude input box'''
        Label(self,text="Altitude").place(x=200,y=10)
        self.wing_Diheadral_input= Entry(self, width = 7)
        self.wing_Diheadral_input.insert(END,"50")
        self.wing_Diheadral_input.place(x=200,y=30)

        '''Map figure'''
        self.map_figure = Figure(figsize=(2,3),dpi=100)
        self.map_plot = self.map_figure.add_subplot()
        print(self.map_plot)
        self.map_plot.set_aspect('equal')
        self.map_canvas = FigureCanvasTkAgg(self.map_figure, self)
        self.map_canvas.get_tk_widget().place(x=100,y=100)

        '''Add waypoint button'''
        self.add_waypoint_button = Button(self,text="Add",height = 2, width = 5, command = self.add_waypoint)
        self.add_waypoint_button.place(x=300,y=10)

        '''Compute path button'''
        self.compute_path_button = Button(self,text="Compute Path",height = 2, width = 10, command = self.compute_path)
        self.compute_path_button.place(x=780,y=800)

        '''Export path'''
        self.export_path_button = Button(self,text="Export Path",height = 2, width = 10, command = self.export_path)
        self.export_path_button.place(x=880,y=800)
        self.tools.get_tile([43,3], self.map_plot)

        self.mainloop()
    def add_waypoint(self):
        pass
    def compute_path(self):
        pass
    def export_path(self):
        pass



if __name__ == "__main__":
    GUI(None)
