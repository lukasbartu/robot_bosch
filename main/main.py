from utils import points2irc
import time
import numpy as np
from matplotlib import pyplot as plt
from interpolation import poly
from graph import Graph
import os

from CRS_commander import Commander
from robotCRS import robCRS97
from robotBosch import robotBosch

from PIL import Image, BmpImagePlugin
from svgpathtools import svg2paths, wsvg, Path


# part to file and bottom left corner
def draw_svg(path, blc, scale = 1.0, angle=0,italic=0):
    paths, attributes = svg2paths(path)
    xs, ys, lengths = [], [], []
    for path in paths:
        length = path.length(0, 1.0)
        N = int(length/20) if int(length/20) >= 50 else 50
        ts = np.linspace(0.0, 1.0, N)
        x, y, les = [],[],[]
        prev_t = 0
        for t in ts:
            les.append(path.length(prev_t, t))
            point = path.point(t)
            x.append(np.real(point)-np.imag(point)*italic*0.2)
            y.append(np.imag(point))  # 1052.36220 - height of A4 page
            prev_t = t
        temp = np.array([x,y])
        rotation = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
        temp =  rotation @ temp
        x = temp[0]
        y = temp[1]

        x = np.array(x) * scale + blc[0]
        y = np.array(y) * scale + blc[1]

    # this the start of what we used from the demo that was given to us
        temp = np.hstack(
            (-x[np.newaxis].T, y[np.newaxis].T, 325 * np.ones(len(x))[np.newaxis].T, np.zeros(len(x))[np.newaxis].T))
    # this is the end

        temp_2, sign = points2irc(c, temp)
        if sign >0:
            y -= 19
            x -= 5
        les = np.array(les) * scale
        xs.append(x)
        ys.append(y)
        lengths.append(les)
        
    params_list = []
    start_positions = []
    end_positions = []

    order = 3

# this the start of what we used from the demo that was given to us
    for x, y, l in zip(xs, ys, lengths):
        points = np.hstack(
            (-x[np.newaxis].T, y[np.newaxis].T, 325 * np.ones(len(x))[np.newaxis].T, np.zeros(len(x))[np.newaxis].T))
        #
        sol, sign = points2irc(c, points)
        params = poly.interpolate(sol)
        params_list.append(params)
        plt.plot(points[:, 0], points[:, 1])
        start_positions.append(np.copy(sol[0]))
        end_positions.append(np.copy(sol[len(sol) - 1]))

    for start, end, params, le in zip(start_positions, end_positions, params_list, lengths):
        c.move_to_pos([start[0], start[1], start[2] + 8000, start[3]])
        c.wait_ready(sync=True)
        time.sleep(1)
        for i in range(len(params)):
            c.splinemv(params[i], order=order, min_time=le[i+1]*100.0)
            
        c.wait_ready(sync=True)
        c.move_to_pos([end[0], end[1], end[2] + 8000, end[3]])
# this is the end

def draw_word():
    word = None
    print("GIVE US STARTING X-COORD:",end="")
    coord_x = int(input())
    print("GIVE US STARTING Y-COORD:",end="")
    coord_y = int(input())
    while(word == None):
        print("WHAT DO YOU WANNA WRITE:",end="")
        word = str(input())
        for letter in word:
            if letter not in ['L','E','F','T','Y','U','I','S','G','H','J','K','Z','X','C','V','N','M',' ','W']:
                print("SORRY WE CANT WRITE",letter, "TRY AGAIN")
                word = None
    print("WHAT SCALE DO YOU WANNA USE (Default=1):",end="")
    scale = float(input())
    print("AT WHAT ANGLE SHOULD WE WRITE [degrees]:",end="")
    angle = int(input()) / 180 * np.pi
    print("DO YOU WANT TO WRITE IN ITALIC (YES=1 | NO=0):",end="")
    italic = int(input())

    for letter in word:
        if letter is not ' ':
            path = letter+'.svg'
            svg_path,temp = svg2paths(path)
            [best_min, best_max] = [None, None]
            for p in svg_path:
                xmin, xmax, ymin, ymax = p.bbox()
                if best_max == None or best_max < xmax:
                    best_max = xmax
                if best_min == None or best_min < xmin:
                    best_min = xmin
            width = (best_max-best_min)+5
            draw_svg(path,[coord_x,coord_y],scale,angle,italic)
        else:
            width = 50
        coord_x += width*np.cos(angle)*scale
        coord_y += width*np.sin(angle)*scale
        

# font = Inder
if __name__ == '__main__':
    robot = robotBosch()
    c = Commander(robot)
    c.open_comm('/dev/ttyUSB0', speed=19200)
    c.init()
    cont = 1
    while(cont == 1):
        draw_word()
        print("DO YOU WANT TO WRITE SOMETHING? (YES=1 | NO=0):")
        cont = int(input())
