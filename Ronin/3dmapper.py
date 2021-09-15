### 41Robotics ###
# Author: Damian Monea
# E-mail: damian.monea98@gmail.com
# Date: 04-09-2021
##################

import json
import pyglet
import numpy as np
import pyglet.gl as gl
import open3d as o3d

CONFIG = {}
PYGLET_ENABLED = False
PYGLET_WINDOW_WIDTH = 0
PYGLET_WINDOW_HEIGHT = 0

with open("config.json", "r") as conf_file:
    CONFIG = json.load(conf_file)

if CONFIG["debug"]:
    PYGLET_ENABLED = True
    PYGLET_WINDOW_WIDTH = CONFIG["debug_window_width"]
    PYGLET_WINDOW_HEIGHT = CONFIG["debug_window_height"]

# window = pyglet.window.Window(PYGLET_WINDOW_WIDTH, PYGLET_WINDOW_HEIGHT, "DEBUG")
# pyglet.app.run()

input_path="./"
output_path="mesh.obj"
dataname="out.ply"
point_cloud= np.loadtxt(input_path+dataname,skiprows=1)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])
o3d.visualization.draw_geometries([pcd])