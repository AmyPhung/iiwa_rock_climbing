import numpy as np
import os

from manipulation.meshcat_cpp_utils import (
    StartMeshcat, AddMeshcatTriad
)

import pydrake.all

from pydrake.all import (
    Adder,
    AddMultibodyPlantSceneGraph, BasicVector, DiagramBuilder, MeshcatVisualizerCpp, MeshcatVisualizerParams, Parser, 
    RollPitchYaw, RigidTransform, RevoluteJoint, Sphere, Simulator, InverseDynamicsController, MultibodyPlant, PiecewisePolynomial,
    PiecewiseQuaternionSlerp, TrajectorySource, Quaternion,
    RotationMatrix, Rgba, LeafSystem, Integrator, PassThrough, Demultiplexer, JacobianWrtVariable, Multiplexer, SceneGraph, StateInterpolatorWithDiscreteDerivative
)
from pydrake.geometry import (Cylinder, GeometryInstance,
                              MakePhongIllustrationProperties)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from manipulation.utils import FindResource

def CreatePoseFromVector(vec):
    # Assume qw qx qy qz px py pz ordering
    return CreatePoseFromVectors(vec[:4],vec[4:])

def CreatePoseFromVectors(q_vec, p_vec):
    q_vec = np.array(q_vec)
    norm = np.linalg.norm(q_vec)
    q_vec = q_vec/norm
    p_vec = np.array(p_vec)
    q_quat = Quaternion(q_vec)
    rot = RotationMatrix(q_quat)
    return RigidTransform(rot, p_vec)

   
def CreateVectorsFromPose(pose):
    p = pose.translation()
    p_vec = list(p)
    q = pose.rotation().ToQuaternion()
    q_vec = [q.w(), q.x(), q.y(), q.z()]
    q_vec + p_vec
    return np.array(q_vec), np.array(p_vec)

def CreateVectorFromPose(pose):
    q_vec, p_vec = CreateVectorsFromPose(pose)
    return np.concatenate((q_vec, p_vec))


def ComputeEulerFromQuat(q_vec):
    q_vec = np.array(q_vec)
    norm = np.linalg.norm(q_vec)
    q_vec = q_vec/norm
    q_quat = Quaternion(q_vec)
    rpy = RollPitchYaw(q_quat)
    return rpy.vector()

def ComputeQuatFromEuler(rpy_vec):
    rpy_vec = np.array(rpy_vec)
    rpy = RollPitchYaw(rpy_vec)
    quat = rpy.ToQuaternion()
    return np.array([quat.w(), quat.x(), quat.y(), quat.z()])

def ComputeAngleComplement(r_vec):
    """Ensures all angles are less than 180 degrees"""
    r_vec = np.array(r_vec)
    
    while max(r_vec)>np.pi:
        r_vec[r_vec>np.pi] -= 2*np.pi
    while min(r_vec)<-np.pi:
        r_vec[r_vec<-np.pi] += 2*np.pi
    return r_vec
    
def ComputeAngleOffset(r_vec1, r_vec2):
    """Computes delta between two euler angles, ensuring that the 
    delta remains 'continuous' 
    
    Usage note: set r_vec1 to desired, r_vec2 to estimated"""
    r_vec1 = ComputeAngleComplement(r_vec1)
    r_vec2 = ComputeAngleComplement(r_vec2)
    
    delta = ComputeAngleComplement(r_vec1 - r_vec2)
    return delta

def ComputeAngleBetweenQuaternions(q1, q2):
    #https://www.researchgate.net/post/How_do_I_calculate_the_smallest_angle_between_two_quaternions
    #https://www.johndcook.com/blog/2012/02/15/dot-cross-and-quaternion-products/
    q1q2 = q1.inverse() @ q2
    q_vec = np.array([q1q2.x(), q1q2.y(), q1q2.z()]) # Vector part of quaternion
    return 2*np.arcsin(np.linalg.norm(q_vec))

def ComputeAngleBetweenQuaternionVectors(q1, q2):
    q1 = Quaternion(q1/np.linalg.norm(q1))
    q2 = Quaternion(q2/np.linalg.norm(q2))
    return ComputeAngleBetweenQuaternions(q1, q2)

def VectorToQuaternion(q_vec):
    return Quaternion(q_vec/np.linalg.norm(q_vec))

# Visualization functions
def AddMeshcatSphere(meshcat,
                     path,
                     radius=0.01,
                     opacity=1.,
                     p_WP=np.array([0,0,0])):
    
    X_WP = RigidTransform(RotationMatrix(),
                          p_WP)
    meshcat.SetTransform(path, X_WP)
    meshcat.SetObject(path, Sphere(radius),
                      Rgba(1, 0, 0, opacity))
    
def visualize_frame(meshcat, name, X_WF, length=0.15, radius=0.006):
    """
    visualize imaginary frame that are not attached to existing bodies

    Input: 
        name: the name of the frame (str)
        X_WF: a RigidTransform to from frame F to world.

    Frames whose names already exist will be overwritten by the new frame
    """
    AddMeshcatTriad(meshcat, "climber/" + name,
                    length=length, radius=radius, X_PT=X_WF)

def visualize_point(meshcat, name, p_WP, radius=0.006):
    """
    visualize a point

    Input: 
        name: the name of the frame (str)
        p_WP: a position from a point P to world.

    Frames whose names already exist will be overwritten by the new frame
    """
    AddMeshcatSphere(meshcat, "climber/" + name,
                     radius=radius, p_WP=p_WP)