import open3d as o3d
import numpy as np

# o3d.t.io.RealSenseSensor.list_devices()

input_path="./"
output_path="./"
dataname="out.ply"
with open(input_path+dataname, "r", encoding="latin1") as in_file:
    point_cloud= np.loadtxt(in_file,skiprows=13)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])

o3d.visualization.draw_geometries([pcd])