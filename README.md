# iiwa_rock_climbing

### Setup
+ Follow the instructions on this repo to set up a docker container to run the simulator
https://github.com/cameronwp/jupyter-drake-meshcat

+ To start the container and run the code in this repo, cd to the base folder of this repo and run this command:
```
docker run -it \
    -p 8888:8888 -p 7000-7100:7000-7100 \
    -v $PWD:/jupyter/iiwa_rock_climbing \
    --user $(id -u):$(id -g) \
    jdm:latest
```


# Notes

ManipulationStation<double> station;
Parser parser(&station.get_mutable_multibody_plant(),
               &station.get_mutable_scene_graph());
parser.AddModelFromFile("my.sdf", "my_model");
...
// coming soon -- sugar API for adding additional objects.
station.Finalize()



// Add default iiwa.
template <typename T>
void ManipulationStation<T>::AddDefaultIiwa(
    const IiwaCollisionModel collision_model) {
  std::string sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_no_collision.sdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_with_box_collision.sdf");
      break;
  }
  const auto X_WI = RigidTransform<double>::Identity();
  auto iiwa_instance = internal::AddAndWeldModelFrom(
      sdf_path, "iiwa", plant_->world_frame(), "iiwa_link_0", X_WI, plant_);
  RegisterIiwaControllerModel(
      sdf_path, iiwa_instance, plant_->world_frame(),
      plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);
}


template <typename T>
void ManipulationStation<T>::SetupClutterClearingStation(
    const std::optional<const math::RigidTransform<double>>& X_WCameraBody,
    IiwaCollisionModel collision_model, SchunkCollisionModel schunk_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kClutterClearing;

  // Add the bins.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/bin.sdf");

    RigidTransform<double> X_WC(RotationMatrix<double>::MakeZRotation(M_PI_2),
                                Vector3d(-0.145, -0.63, 0.075));
    internal::AddAndWeldModelFrom(sdf_path, "bin1", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);

    X_WC = RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI),
                                  Vector3d(0.5, -0.1, 0.075));
    internal::AddAndWeldModelFrom(sdf_path, "bin2", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);
  }










  from manipulation.scenarios import AddIiwa, AddWsg, AddRgbdSensors
from manipulation.utils import FindResource

def MakeManipulationStation(time_step=0.002):
    builder = pydrake.systems.framework.DiagramBuilder()

    # Add (only) the iiwa, WSG, and cameras to the scene.
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, time_step=time_step)
    iiwa = AddIiwa(plant)
    wsg = AddWsg(plant, iiwa)


    Note: wsg is the gripper



See https://github.com/RussTedrake/manipulation/blob/master/manipulation/scenarios.py for python example

def AddIiwa(plant, collision_model="no_collision"):
    sdf_path = pydrake.common.FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7/"
        f"iiwa7_{collision_model}.sdf")

    parser = pydrake.multibody.parsing.Parser(plant)
    iiwa = parser.AddModelFromFile(sdf_path)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))

    # Set default positions:
    q0 = [0.0, 0.1, 0, -1.2, 0, 1.6, 0]
    index = 0
    for joint_index in plant.GetJointIndices(iiwa):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, pydrake.multibody.tree.RevoluteJoint):
            joint.set_default_angle(q0[index])
            index += 1

    return iiwa
