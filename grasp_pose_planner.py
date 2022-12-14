import numpy as np
from pydrake.all import (AngleAxis, PiecewisePolynomial, PiecewisePose,
                         RigidTransform, RotationMatrix)


def MakeGripperFrames(X_G, t0=0,duration=20.0):
    """
    Takes a partial specification with X_G["initial"], X_G["pick"], and
    X_G["end"], and returns a X_G and times with all of the pick and end
    frames populated.
    """
    
    # Duration cap
    if duration < 10.0:
        duration = 10.0 

    X_GgraspGpregrasp = RigidTransform([0, -0.2, 0])
    X_G["prepick"] = X_G["pick"]
    X_G["preend"] = X_G["end"] 

    # I'll interpolate a halfway orientation by converting to axis angle and
    # halving the angle.

    print(X_G["initial"])

    X_GinitialGprepick = X_G["initial"].inverse() @ X_G ["prepick"] 
    X_GinitialGprepick = RigidTransform(X_GinitialGprepick.rotation(),X_GinitialGprepick.translation())


    angle_axis = X_GinitialGprepick.rotation().ToAngleAxis()
    X_GinitialGprepare = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GinitialGprepick.translation() / 2.0)

    X_G["prepare"] = X_G["initial"] @ X_GinitialGprepare
    p_G = np.array(X_G["prepare"].translation())
    
    '''
    p_G[2] = 0.5
    # To avoid hitting the cameras, make sure the point satisfies x - y < .5
    if abs(p_G[0]) - abs(p_G[1]) < .5:
        scale = .5 / (abs(p_G[0]) - abs(p_G[1]))
        p_G[:1] /= scale '''

    X_G["prepare"].set_translation(p_G)

    X_GprepickGpreend = X_G["prepick"].inverse() @ X_G["preend"]
    angle_axis = X_GprepickGpreend.rotation().ToAngleAxis()
    X_GprepickGclearance = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GprepickGpreend.translation() / 2.0)
    X_G["clearance"] = X_G["initial"] @ X_GinitialGprepare
    p_G = np.array(X_G["clearance"].translation())
    '''p_G[2] = 0.5
    # To avoid hitting the cameras, make sure the point satisfies x - y < .6
    
    if abs(p_G[0]) - abs(p_G[1]) < .6:
        scale = .6 / (abs(p_G[0]) - abs(p_G[1]))
        p_G[:1] /= scale'''
    X_G["clearance"].set_translation(p_G) 

    # Now let's set the timing
    times = {"initial": t0}
    prepare_time = 15.0 * np.linalg.norm(X_GinitialGprepare.translation())
    times["prepare"] = times["initial"] + prepare_time
    times["prepick"] = times["prepare"] + prepare_time
    # Allow some time for the gripper to close.
    times["pick_start"] = times["prepick"] + 1
    times["pick_end"] = times["pick_start"] + 0.5
    X_G["pick_start"] = X_G["pick"]
    X_G["pick_end"] = X_G["pick"]
    times["postpick"] = times["pick_end"] + 2.0
    X_G["postpick"] = X_G["prepick"]

    
    time_to_from_clearance = 10.0 * np.linalg.norm(
        X_GprepickGclearance.translation())
    times["clearance"] = times["postpick"] + time_to_from_clearance
    times["preend"] = times["clearance"] + time_to_from_clearance
    times["end_start"] = times["preend"] + 2.0
    times["end_end"] = times["end_start"] + 2.0
    X_G["end_start"] = X_G["end"]
    X_G["end_end"] = X_G["end"]
    times["postend"] = times["end_end"] + 2.0
    X_G["postend"] = X_G["end_end"] #originally preend 

    scale = duration/times["end_end"]
    for t in times:
        times[t] *= scale
        X_G[t] = X_G[t] 

    return X_G, times


def MakeGripperPoseTrajectory(X_G, times, steps=5,t0=0):
    """Constructs a gripper position trajectory from the plan "sketch"."""
    sample_times = []
    poses = []
    
    for name in [
            "initial", "prepare", "prepick", "pick_start", "pick_end",
            "postpick", "clearance", "preend", "end_start", "end_end",
            "postend"
    ]:
        sample_times.append(times[name])
        poses.append(X_G[name])

    linear_interpolation = PiecewisePose.MakeLinear(sample_times, poses)
    
    real_poses = []
    real_times = []
    for i in range(steps):
        step = times["postend"] / steps * i
        real_times.append(step)
        real_poses.append(linear_interpolation.GetPose(step))

    real_poses.append(linear_interpolation.GetPose(times["postend"]))
    real_times.append(times["postend"])
    real_times = [(x + t0) for x in real_times]
    print("grasping times" + str(real_times))
    return linear_interpolation, real_poses, real_times


def MakeGripperCommandTrajectory(times,t0=0):
    """Constructs a WSG command trajectory from the plan "sketch"."""
    opened = np.array([0.107])
    closed = np.array([0.0])
    print("wsg grasp times " + str(times))
    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [times["initial"]+t0, times["pick_start"]+t0], np.hstack([[opened],
                                                            [opened]]))
    traj_wsg_command.AppendFirstOrderSegment(times["pick_end"]+t0, closed)
    
    traj_wsg_command.AppendFirstOrderSegment(times["end_start"]+t0, closed)
    traj_wsg_command.AppendFirstOrderSegment(times["end_end"]+t0, closed)
    traj_wsg_command.AppendFirstOrderSegment(times["postend"]+t0, closed)
    
    return traj_wsg_command