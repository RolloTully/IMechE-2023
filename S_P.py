import numpy as np


def distance(x,y):

        dist = np.sqrt((x-j)**2 + (y-k)**2)
        return dist
  
def time(x0,y0,u_x,u_y,a):
    
    t  = np.linspace(0,60,1000000)

    s_x = u_x*t + (1/2)*(a)*(t**2)
    s_y = u_y*t + (1/2)*(a)*(t**2)
    moving_x = x0 + s_x
    moving_y = y0 + s_y

    d = distance(moving_x, moving_y)

    t_min = t[d.argmin()]

    return t_min


u_x = float(input("u_x : "))
u_y = float(input("u_y : "))
x0 = float(input("x : "))
y0 = float(input("y : "))
a = float(input("a : "))
j = float(input("j : "))
k = float(input("k : "))

t_min = time(x0,y0,u_x,u_y,a)
print(f"t_min = {t_min}")