"""Isaac Sim scene for the MTC pick-and-place demo (hardware:=isaac).

Builds a cell-free scene (UR5e + RH56DFX from the repo URDF, tabletop, 7.5 cm physics cubes),
bridges it to ROS 2 for topic_based_ros2_control. Cube positions come from
ur5e_rh56dfx_manipulation/config/pick_place.yaml, the same file the ROS 2 side reads.

  isaac side: isaac/run_scene.sh [--side left] [--task stack] [--headless]   (expands the URDF, sets up ROS 2)
  ros2 side:  ros2 launch ur5e_rh56dfx_manipulation mtc_demo.launch.py hardware:=isaac [task:=stack]

ROS 2 interface
  /joint_states_raw   (pub)  every joint incl. mimics; joint_state_filter.py relays it to /joint_states
  /joint_command      (sub)  position targets from topic_based_ros2_control
  /clock              (pub)  simulation time
  /tf                 (pub)  world → each cube (cube_pose_mode:=isaac reads pick_cube from here)
  /isaac_attach_cmd   (sub)  "ATTACH <cube>" / "DETACH <cube>" from isaac_attach_manager.py
"""
import argparse
import os
import time

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", required=True, help="xacro-expanded URDF (hardware:=isaac)")
parser.add_argument("--side", default="left")
parser.add_argument("--usd-dir", default=os.path.expanduser("~/.cache/moveit_mtc_pick_and_place/usd"),
                    help="where the URDF importer writes the robot USD")
parser.add_argument("--task", choices=["pick_place", "stack"], default="pick_place",
                    help="stack: one cube per stack_cube_positions entry of the task config")
parser.add_argument("--config", default="", help="task config (default: the installed config/pick_place.yaml)")
parser.add_argument("--cube", default="",
                    help="cube x,y on the table (world frame; default cube_fixed_position from the config), "
                         "or 'random' for a reachable spot MoveIt is not told about")
parser.add_argument("--seed", type=int, default=0, help="seed for --cube random (0 = different every run)")
parser.add_argument("--grasp", choices=["fixed", "friction"], default="fixed",
                    help="friction: the cube is held by finger contact only; fixed: ATTACH adds a fixed joint")
parser.add_argument("--duration", type=float, default=0.0, help="stop after this many sim seconds (0 = run forever)")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

W, H = 1280, 720
simulation_app = SimulationApp({"headless": args.headless, "width": W, "height": H})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.app
import omni.usd
import usdrt.Sdf
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ext = omni.kit.app.get_app().get_extension_manager()
for e in ("omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf", "isaacsim.ros2.bridge"):
    ext.set_extension_enabled_immediate(e, True)
simulation_app.update()

import rclpy
from rosgraph_msgs.msg import Clock  # noqa: F401  (forces the bundled message libs to load)
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from isaacsim.core.simulation_manager import SimulationManager

PREFIX = f"{args.side}_"


# ---------------------------------------------------------------- robot
def share(pkg):
    # resolved from AMENT_PREFIX_PATH: the `ros2` CLI would run under Isaac's Python here
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix and os.path.isdir(os.path.join(prefix, "share", pkg)):
            return os.path.join(prefix, "share", pkg)
    raise SystemExit(f"package '{pkg}' not found: source /opt/ros/jazzy and the workspace first")


import yaml

cfg_path = args.config or os.path.join(share("ur5e_rh56dfx_manipulation"), "config", "pick_place.yaml")
task_cfg = yaml.safe_load(open(cfg_path))["/**"]["ros__parameters"]
CUBE_SIZE = task_cfg["cube_size"]
TABLE_TOP = task_cfg["table_height"]                 # top face of the URDF table_top collision box

os.makedirs(args.usd_dir, exist_ok=True)
pkgs = ["ur_description", "rh56dfx_description", "ur5e_rh56dfx_description"]
cfg = URDFImporterConfig()
cfg.urdf_path = args.urdf
cfg.usd_path = args.usd_dir
cfg.merge_fixed_joints = False                       # keep <side>_hand_base as its own prim for ATTACH
cfg.fix_base = True
cfg.allow_self_collision = False
cfg.ros_package_paths = [{"name": p, "path": share(p)} for p in pkgs]
robot_usd = URDFImporter(cfg).import_urdf()

stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
stage_utils.add_reference_to_stage(robot_usd, "/World/robot")
simulation_app.update()


def find(root, pred):
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if pred(p):
            return p
    return None


art_root = find("/World/robot", lambda p: p.HasAPI(UsdPhysics.ArticulationRootAPI))
hand_base = find("/World/robot", lambda p: p.GetName() == f"{PREFIX}hand_base" and p.HasAPI(UsdPhysics.RigidBodyAPI))
assert art_root and hand_base, "articulation root / hand_base not found in imported robot"
print(f"[scene] articulation: {art_root.GetPath()}  hand: {hand_base.GetPath()}")

# actuated joints start at `home` (initial_positions.yaml); mimic joints follow their leader
import xml.etree.ElementTree as ET

urdf_joints = ET.parse(args.urdf).getroot().findall("joint")
mimics = {j.get("name"): j.find("mimic") for j in urdf_joints if j.find("mimic") is not None}
home = yaml.safe_load(open(os.path.join(share("ur5e_rh56dfx_moveit_config"), "config/initial_positions.yaml")))
home = home["initial_positions"]
FINGERS = ("thumb", "index", "middle", "ring", "little")
joints = {p.GetName(): p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/robot")) if p.IsA(UsdPhysics.RevoluteJoint)}

# stiff position drives: the arm and wrist track MoveIt, fingers squeeze with bounded force
n_drives = 0
for name, p in joints.items():
    if name in mimics:
        continue
    finger = name.startswith(PREFIX) and any(f in name for f in FINGERS)
    wrist = name.startswith(PREFIX) and not finger
    stiffness, damping, max_force = (80.0, 20.0, 10.0) if finger else (500.0, 50.0, 10.0) if wrist else (2.0e4, 2.0e2, 1.0e4)
    drive = UsdPhysics.DriveAPI.Apply(p, "angular")             # USD angular units are degrees
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr().Set(stiffness)                   # per degree
    drive.CreateDampingAttr().Set(damping)
    drive.CreateMaxForceAttr().Set(max_force)
    q0 = float(np.degrees(home.get(name, 0.0)))
    drive.CreateTargetPositionAttr().Set(q0)
    PhysxSchema.JointStateAPI.Apply(p, "angular").CreatePositionAttr().Set(q0)
    n_drives += 1

# the importer writes URDF <mimic> as NewtonMimicAPI only, which PhysX ignores: couple them with PhysxMimicJointAPI
# (q + gearing * q_ref + offset = 0). In the importer's joint frames the leader's axis is rotX and the offset keeps
# its URDF sign; the limits get a little slack so the constraint never fights them.
for name, m in mimics.items():
    p, ref = joints[name], joints[m.get("joint")]
    p.RemoveAPI(UsdPhysics.DriveAPI, "angular")
    mult, off = float(m.get("multiplier", 1)), float(m.get("offset", 0))
    api = PhysxSchema.PhysxMimicJointAPI.Apply(p, "rotZ")
    api.CreateReferenceJointRel().SetTargets([ref.GetPath()])
    api.CreateReferenceJointAxisAttr().Set("rotX")
    api.CreateGearingAttr().Set(-mult)
    api.CreateOffsetAttr().Set(off)                              # sign as measured in the imported joint frames
    rj = UsdPhysics.RevoluteJoint(p)
    lo, hi = rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()
    span = hi - lo
    rj.GetLowerLimitAttr().Set(lo - 0.1 * span)
    rj.GetUpperLimitAttr().Set(hi + 0.1 * span)
print(f"[scene] drives on {n_drives} actuated joints, {len(mimics)} mimic joints coupled")

# the importer only sets Newton's articulation flags; PhysX needs its own (self-collision is on by default)
art = PhysxSchema.PhysxArticulationAPI.Apply(art_root)
art.CreateEnabledSelfCollisionsAttr().Set(False)
art.CreateSolverPositionIterationCountAttr().Set(32)
art.CreateSolverVelocityIterationCountAttr().Set(1)

# grippy finger pads
pad_mat = UsdShade.Material.Define(stage, "/World/Materials/rubber")
pm = UsdPhysics.MaterialAPI.Apply(pad_mat.GetPrim())
pm.CreateStaticFrictionAttr(2.0); pm.CreateDynamicFrictionAttr(1.5); pm.CreateRestitutionAttr(0.0)
for p in Usd.PrimRange(stage.GetPrimAtPath("/World/robot")):
    if p.HasAPI(UsdPhysics.CollisionAPI) and any(k in p.GetPath().pathString for k in ("rubber", "pad", "tip")):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(pad_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

# ---------------------------------------------------------------- scene
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(900)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(2500); sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))

ground = UsdGeom.Mesh.Define(stage, "/World/ground")
ground.CreatePointsAttr([(-6, -6, 0), (6, -6, 0), (6, 6, 0), (-6, 6, 0)])
ground.CreateFaceVertexCountsAttr([4]); ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
ground.CreateDisplayColorAttr([(0.16, 0.17, 0.19)])

if args.task == "stack":
    xy = task_cfg["stack_cube_positions"]
    spots = {f"cube_{i}": (xy[2 * i], xy[2 * i + 1]) for i in range(len(xy) // 2)}
elif args.cube == "random":                              # same region as scene_publisher's random mode
    x0, x1, y0, y1 = task_cfg["cube_random_region"]
    rng = np.random.default_rng(args.seed or None)
    while True:
        cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
        if np.hypot(cx, cy) <= task_cfg["cube_random_max_radius"]:
            break
    print(f"[scene] random cube at x={cx:.3f} y={cy:.3f}", flush=True)
    spots = {"pick_cube": (cx, cy)}
else:
    spots = {"pick_cube": tuple(map(float, args.cube.split(","))) if args.cube else tuple(task_cfg["cube_fixed_position"])}

cube_mat = UsdShade.Material.Define(stage, "/World/Materials/cube")
cm = UsdPhysics.MaterialAPI.Apply(cube_mat.GetPrim())
cm.CreateStaticFrictionAttr(1.0); cm.CreateDynamicFrictionAttr(0.9); cm.CreateRestitutionAttr(0.0)
COLOURS = [(0.15, 0.62, 0.86), (0.95, 0.55, 0.15), (0.35, 0.72, 0.36), (0.80, 0.30, 0.45)]
hand_links = [p.GetPath() for p in Usd.PrimRange(hand_base) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
cubes = {}
for i, (name, (cx, cy)) in enumerate(spots.items()):
    cube = UsdGeom.Cube.Define(stage, f"/World/{name}")
    cube.CreateSizeAttr(CUBE_SIZE)
    cube.CreateDisplayColorAttr([COLOURS[i % len(COLOURS)]])
    UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(cx, cy, TABLE_TOP + CUBE_SIZE / 2 + 0.001))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(0.01)    # light (hollow print), so the fingertips can hold it
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(cube_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    if args.grasp == "fixed":
        # the hand never pushes on the cube: MoveIt stops the fingers at its surface and ATTACH welds it to the palm.
        # (finger contact on a cube resting on the table drives the gram-scale pad bodies unstable in PhysX)
        UsdPhysics.FilteredPairsAPI.Apply(cube.GetPrim()).CreateFilteredPairsRel().SetTargets(hand_links)
    cubes[name] = cube.GetPrim()
print(f"[scene] cubes: {', '.join(f'{n} at {x:.2f},{y:.2f}' for n, (x, y) in spots.items())}", flush=True)


def world_pose(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


# ---------------------------------------------------------------- ROS 2 graph
og.Controller.edit(
    {"graph_path": "/World/ros2_bridge", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("read_js", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("pub_js", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("sub_cmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "read_js.inputs:execIn"),
            ("read_js.outputs:execOut", "pub_js.inputs:execIn"),
            ("read_js.outputs:jointNames", "pub_js.inputs:jointNames"),
            ("read_js.outputs:jointPositions", "pub_js.inputs:jointPositions"),
            ("read_js.outputs:jointVelocities", "pub_js.inputs:jointVelocities"),
            ("read_js.outputs:jointEfforts", "pub_js.inputs:jointEfforts"),
            ("read_js.outputs:jointDofTypes", "pub_js.inputs:jointDofTypes"),
            ("read_js.outputs:stageMetersPerUnit", "pub_js.inputs:stageMetersPerUnit"),
            ("read_js.outputs:sensorTime", "pub_js.inputs:sensorTime"),
            ("tick.outputs:tick", "sub_cmd.inputs:execIn"),
            ("tick.outputs:tick", "ctrl.inputs:execIn"),
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("ctx.outputs:context", "pub_js.inputs:context"),
            ("ctx.outputs:context", "sub_cmd.inputs:context"),
            ("ctx.outputs:context", "clock.inputs:context"),
            ("sim_time.outputs:simulationTime", "clock.inputs:timeStamp"),
            ("sub_cmd.outputs:jointNames", "ctrl.inputs:jointNames"),
            ("sub_cmd.outputs:positionCommand", "ctrl.inputs:positionCommand"),
            ("sub_cmd.outputs:velocityCommand", "ctrl.inputs:velocityCommand"),
            ("sub_cmd.outputs:effortCommand", "ctrl.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ctrl.inputs:robotPath", str(art_root.GetPath())),
            ("read_js.inputs:prim", [usdrt.Sdf.Path(str(art_root.GetPath()))]),
            ("pub_js.inputs:topicName", "joint_states_raw"),
            ("sub_cmd.inputs:topicName", "joint_command"),
        ],
    },
)

# ---------------------------------------------------------------- attach + cube TF (rclpy)
rclpy.init()
node = rclpy.create_node("isaac_pick_place_scene")
tf_pub = node.create_publisher(TFMessage, "/tf", 10)
fixed_path = Sdf.Path("/World/grasp_joint")


def on_attach(msg):
    verb, _, name = msg.data.partition(" ")
    name = name or "pick_cube"
    if args.grasp != "fixed":
        node.get_logger().info(f"{msg.data}: friction grasp, nothing to do")
        return
    cube = cubes.get(name)
    if cube is None:
        node.get_logger().warn(f"{msg.data}: no cube named {name}")
        return
    if verb == "ATTACH" and not stage.GetPrimAtPath(fixed_path):
        rel = world_pose(cube) * world_pose(hand_base).GetInverse()   # cube in hand frame
        j = UsdPhysics.FixedJoint.Define(stage, fixed_path)
        j.CreateBody0Rel().SetTargets([hand_base.GetPath()])
        j.CreateBody1Rel().SetTargets([cube.GetPath()])
        t, r = rel.ExtractTranslation(), rel.ExtractRotationQuat()
        j.CreateLocalPos0Attr(Gf.Vec3f(t)); j.CreateLocalRot0Attr(Gf.Quatf(r))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        node.get_logger().info(f"ATTACH: fixed joint hand ↔ {name}")
    elif verb == "DETACH" and stage.GetPrimAtPath(fixed_path):
        stage.RemovePrim(fixed_path)
        UsdPhysics.RigidBodyAPI(cube).CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))   # wakes it: it was asleep
        node.get_logger().info(f"DETACH: fixed joint removed from {name}")


node.create_subscription(String, "/isaac_attach_cmd", on_attach, 10)



def publish_cube_tf(sim_t):
    msg = TFMessage()
    for name, prim in cubes.items():
        m = world_pose(prim)
        t, q = m.ExtractTranslation(), m.ExtractRotationQuat()
        ts = TransformStamped()
        ts.header.stamp.sec = int(sim_t); ts.header.stamp.nanosec = int((sim_t % 1) * 1e9)
        ts.header.frame_id, ts.child_frame_id = "world", name
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = t
        ts.transform.rotation.w = q.GetReal()
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z = q.GetImaginary()
        msg.transforms.append(ts)
    tf_pub.publish(msg)


# ---------------------------------------------------------------- run
PHYSICS_HZ = 120
SimulationManager.setup_simulation(dt=1.0 / PHYSICS_HZ)
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(p)
        scene_api.CreateSolverTypeAttr().Set("TGS")
        scene_api.CreateTimeStepsPerSecondAttr().Set(PHYSICS_HZ)
app_utils.play()
simulation_app.update()

step = 0
print("[scene] running — start the ROS 2 side now", flush=True)
wall0 = time.monotonic()
try:
    while simulation_app.is_running():
        simulation_app.update()
        step += 1
        sim_t = step / PHYSICS_HZ
        lag = sim_t - (time.monotonic() - wall0)      # never run ahead of real time: MoveIt plans on the wall clock
        if lag > 0:
            time.sleep(lag)
        rclpy.spin_once(node, timeout_sec=0.0)
        if step % (PHYSICS_HZ * 5) == 0:
            print(f"[scene] sim {sim_t:6.1f}s  wall {time.monotonic() - wall0:6.1f}s", flush=True)
        publish_cube_tf(sim_t)
        if args.duration and sim_t >= args.duration:
            break
except KeyboardInterrupt:
    pass

node.destroy_node(); rclpy.shutdown()
app_utils.stop()
simulation_app.close()
