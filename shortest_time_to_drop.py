import numpy as np


# u_x = #current speed in x axis
# u_y = #current speed in y axis
# x0 = self.longitude #x coordinate of current position
# y0 = self.latitude #y coordinate of current position
# a = 0 #current accelaration #Assusimg Average acceleration is 0
# j = #x coordinate of drop
# k = #y coordinate of drop

def distance(x,y):

        dist = np.sqrt((x-j)**2 + (y-k)**2)
        return dist
  
def time(x0,y0,u_x,u_y,a):
    
    t  = np.linspace(0,60,1000000) #array of different(1000000) time intervals between 0-60 seconds

    s_x = u_x*t + (1/2)*(a)*(t**2) #array of different displacement in x direction from current position to final position after time t traveled with speed u_x
    s_y = u_y*t + (1/2)*(a)*(t**2) #array of different displacement in y direction from current position to final position after time t traveled with speed u_x
    x = x0 + s_x #array of x distances from x axis at time t
    y = y0 + s_y #arrya of y distsnces from y axis at time t

    d = distance(x, y) #array of distance from (x,y) to (j,k)

    t_min = t[d.argmin()] #time at minimum distance

    return t_min

self.time_to_CA = time(x0,y0,u_x,u_y,a)
