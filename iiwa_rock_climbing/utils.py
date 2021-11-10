import numpy as np
import os

import pydrake.all
from pydrake.all import (AbstractValue, BaseField, ModelInstanceIndex,
                         DepthRenderCamera, RenderCameraCore, RgbdSensor,
                         CameraInfo, ClippingRange, DepthRange,
                         DepthImageToPointCloud, LeafSystem,
                         MakeRenderEngineVtk, RenderEngineVtkParams, RgbdSensor,
                         PrismaticJoint, BallRpyJoint, SpatialInertia)
from pydrake.geometry import (Cylinder, GeometryInstance,
                              MakePhongIllustrationProperties)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from manipulation.utils import FindResource

def AddFloatingIiwa(plant, collision_model="no_collision"):
    print("ADSFD")
    asdg
    sdf_path = pydrake.common.FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7/"
        f"iiwa7_{collision_model}.sdf")

    parser = pydrake.multibody.parsing.Parser(plant)
    iiwa = parser.AddModelFromFile(sdf_path)
    # plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))

    # Set default positions:
    # q0 = [0.0, 0.1, 0, -1.2, 0, 1.6, 0]
    index = 0
    for joint_index in plant.GetJointIndices(iiwa):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, pydrake.multibody.tree.RevoluteJoint):
            joint.set_default_angle(q0[index])
            index += 1

    return iiwa
