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

def CreatePoseFromVector(q_vec, p_vec):
    q_vec = np.array(q_vec)
    p_vec = np.array(p_vec)
    q_quat = Quaternion(q_vec)
    rot = RotationMatrix(q_quat)
    return RigidTransform(rot, p_vec)