import numpy as np
import os

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