import numpy as np

import pydrake
from pydrake.all import (
    RigidTransform, PiecewiseQuaternionSlerp, RotationMatrix
)

from manipulation.meshcat_utils import AddMeshcatTriad

"""
Simple trajectory
"""
# define the angle in x-y plane to convert 3D problem into 2D problem
def calculate_x_y_orientation(position_A, position_B):
    # the angle that robot will generate trajectory along this direction
    agnle_xy = np.arctan2((position_B[1]-position_A[1]),(position_B[0]-position_A[0]))
    return agnle_xy

def calculate_circular_radius(position_A, position_B):
    delta_x = np.sqrt((position_B[1]-position_A[1])**2 + (position_B[0]-position_A[0])**2)
    delta_z = position_B[2]-position_A[2]
    cir_radius = (delta_x**2 + delta_z**2)/(2*delta_z)
    return cir_radius



# solve orientation interpolation problem
# a trajectory for quaternions that are interpolated using piecewise slerp (spherical linear interpolation).
def interpolate_orientation(Rotation_A, Rotation_B):
    # assume scalar t: [0, 1]
    trajectory = PiecewiseQuaternionSlerp()
    # Given a new RotationMatrix, this method adds one segment to the end of this.
    # add the initial rotation matrix A
    trajectory.Append(0.0, Rotation_A)
    # add the final rotation matrix B
    trajectory.Append(1.0, Rotation_B)
    return trajectory

def interpolate_pose_linear(X_A, X_B, t):
    # X_A and X_B are RigidTransform Pose
    # X_A is the initial pose
    # X_B is the final pose
    # t:[0, 1] => it can be scaled from [0, T]
    
    p_A = X_A.translation() 
    p_B = X_B.translation()
    r_A = X_A.rotation()
    r_B = X_B.rotation()

    # interpolate rotation
    print(r_A)
    print(r_B)
    r_interpolate = interpolate_orientation(r_A, r_B)
    print(r_interpolate)

    r_cols = r_interpolate.cols()
    print(r_cols)
    r_rows = r_interpolate.rows()
    print(r_rows)
    
    # The interpolated quaternion at t
    r_quaternion = r_interpolate.orientation(t)
    print(r_quaternion)
    # print(r_quaternion.shape)
    
    r_current = RotationMatrix(r_interpolate.orientation(t))
    print(r_current)

    # position is calculated in a linear line segment
    p_current = p_A + t * (p_B - p_A)
    print(p_current)

    X_WO_current = RigidTransform(r_current, p_current)
    return X_WO_current

# circular interpolation
def interpolate_pose_circular(X_A, X_B, t):
    # X_A and X_B are RigidTransform Pose
    # X_A is the initial pose
    # X_B is the final pose
    # t:[0, 1] => it can be scaled from [0, T]

    p_A = X_A.translation() 
    p_B = X_B.translation()
    r_A = X_A.rotation()
    r_B = X_B.rotation()

    # interpolate rotation
    # print(r_A)
    # print(r_B)
    r_interpolate = interpolate_orientation(r_A, r_B)
    # print(r_interpolate)

    r_cols = r_interpolate.cols()
    # print(r_cols)
    r_rows = r_interpolate.rows()
    # print(r_rows)
    
    # The interpolated quaternion at t
    r_quaternion = r_interpolate.orientation(t)
    # print(r_quaternion)
    # print(r_quaternion.shape)
    
    r_current = RotationMatrix(r_quaternion)
    # print(r_current)

    # calculate the circular curve
    c_radius = calculate_circular_radius(p_A, p_B)
    phi = calculate_x_y_orientation(p_A, p_B)
    # theta will change from -90 to 0, but we will release the gripper at theta=45 degrees
    # t = 0.5 => release gripper
    theta = (t-1)*np.pi/2

    p_change = np.zeros((3,))
    p_change[0] = c_radius * np.cos(theta) * np.cos(phi)
    p_change[1] = c_radius * np.cos(theta) * np.sin(phi)
    p_change[2] = c_radius + c_radius * np.sin(theta)
    print(c_radius)
    print(p_change)

    p_current = p_A + p_change

    X_WO_current = RigidTransform(r_current, p_current)
    return X_WO_current


   
# # Create the pose trajectory
# T = 1.0
# time_list = np.linspace(0, T, 10, endpoint=False)
# print(len(time_list))
# Pose = []
# p_initial = [0, 0, 0]
# r_initial = RotationMatrix.MakeYRotation(np.pi/2)
# X_initial = RigidTransform(r_initial, p_initial)
# p_final = [0.5, 0.5, 1]
# r_final = RotationMatrix.MakeXRotation(np.pi/3)
# X_final = RigidTransform(r_final, p_final)

# for i in range(len(time_list)):
#     pose = interpolate_pose_linear(X_initial, X_final,time_list[i])
#     Pose.append(pose)
#     print(i)

# meshcat.Delete()
# builder = DiagramBuilder()

# plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
# iiwa = AddPlanarIiwa(plant)
# wsg = AddWsg(plant, iiwa, roll=0.0, welded=True, sphere=True)
# for i in range(10):
#     X_WStart = Pose[i]
#     meshcat.SetObject('start'+str(i)+'good', Sphere(0.01), rgba=Rgba(.9, .1, .1, 1))
#     meshcat.SetTransform('start'+str(i)+'good', X_WStart)
#     AddMeshcatTriad(meshcat,'start'+str(i)+'good',length=0.15, radius=0.006,X_PT=X_WStart)

# X_WStart = RigidTransform([0.8, 0, 0.65])
# meshcat.SetObject("start", Sphere(0.02), rgba=Rgba(.9, .1, .1, 1))
# meshcat.SetTransform("start", X_WStart)
# X_WGoal = RigidTransform([0.8, 0, 0.4])
# meshcat.SetObject("goal", Sphere(0.02), rgba=Rgba(.1, .9, .1, 1))
# meshcat.SetTransform("goal", X_WGoal)