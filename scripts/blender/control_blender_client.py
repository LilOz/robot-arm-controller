"""
Blender Robot Control Client - Integrated with existing visualization
Combines your existing robot loading/animation with real-time server control
"""

import colorsys
import json
import math
import socket
import threading
import time

import bpy
from mathutils import Euler, Matrix, Vector

# =============================================================================
# Configuration
# =============================================================================

CONFIG_PATH = "/Users/ayman/dev/projects/robot-arm-controller/config/models/6dof_spherical_model.json"
SERVER_HOST = "localhost"
SERVER_PORT = 8888

AXIS_INDEX = {
    "X": 0,
    "Y": 1,
    "Z": 2,
}

# =============================================================================
# Robot Model Builder (from your existing code)
# =============================================================================


class RobotModelBuilder:
    """Builds Blender robot model from JSON config"""

    def __init__(self, config_path):
        with open(config_path) as f:
            self.config = json.load(f)
        self.links = self.config["links"]
        self.joints = []
        self.materials = []

    def make_material(self, name, idx, total):
        """Create colored material for link"""
        mat = bpy.data.materials.new(name=name)
        mat.use_nodes = True

        # Evenly spaced hues
        h = idx / max(1, total)
        r, g, b = colorsys.hsv_to_rgb(h, 0.8, 0.8)

        bsdf = mat.node_tree.nodes["Principled BSDF"]
        bsdf.inputs["Base Color"].default_value = (r, g, b, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.4

        return mat

    def clear_existing_robot(self):
        """Remove existing robot objects"""
        # Remove objects with robot link names
        for link in self.links:
            name = link["name"]
            if name in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)

            link_name = f"{name}_link"
            if link_name in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[link_name], do_unlink=True)

            flag_name = f"{name}_flag"
            if flag_name in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[flag_name], do_unlink=True)

        # Clear old materials
        for i in range(len(self.links)):
            mat_name = f"LinkMat_{i}"
            if mat_name in bpy.data.materials:
                bpy.data.materials.remove(bpy.data.materials[mat_name])

    def build_robot(self):
        """Build robot model in Blender scene"""
        self.clear_existing_robot()

        # Create materials
        self.materials = [
            self.make_material(f"LinkMat_{i}", i, len(self.links))
            for i in range(len(self.links))
        ]

        cursor = Vector((0, 0, 0))
        self.joints = []

        # Build each link
        for i, link in enumerate(self.links):
            name = link["name"]
            length = link["length"]
            mat = self.materials[i]

            # Create joint empty
            bpy.ops.object.empty_add(type="PLAIN_AXES", location=cursor)
            joint = bpy.context.object
            joint.name = name
            joint.rotation_mode = "XYZ"
            joint.empty_display_size = 0.05
            self.joints.append(joint)

            # Create link mesh
            if length > 0:
                # Cylindrical link
                bpy.ops.mesh.primitive_cylinder_add(radius=0.02, depth=length)
                mesh = bpy.context.object
                mesh.name = f"{name}_link"
                mesh.parent = joint
                mesh.location = (0, 0, length / 2)
                mesh.rotation_euler = (0, 0, 0)
                mesh.data.materials.append(mat)
            else:
                # Zero-length wrist joint → thin offset "flag"
                bpy.ops.mesh.primitive_cube_add(size=1)
                mesh = bpy.context.object
                mesh.name = f"{name}_flag"
                mesh.parent = joint

                # Thin rectangle based on axis
                if link["axis"] == "X":
                    mesh.scale = (0.05, 0.025, 0.01)
                elif link["axis"] == "Y":
                    mesh.scale = (0.025, 0.05, 0.01)
                else:
                    mesh.scale = (0.01, 0.01, 0.05)
                    mesh.location = (0, 0, 0.025)

                mesh.data.materials.append(mat)

            # Advance cursor
            cursor += Vector((0, 0, length))

        # Parent joints (LAST)
        for parent, child in zip(self.joints[:-1], self.joints[1:]):
            child.parent = parent
            child.matrix_parent_inverse = parent.matrix_world.inverted()

        print(f"✓ Built robot model with {len(self.joints)} joints")
        return self.joints


# =============================================================================
# Robot Control Client (Server Communication)
# =============================================================================


class RobotControlClient:
    """Handles communication with C++ control server"""

    def __init__(self, host="localhost", port=8888):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False

        # Store robot state
        self.joint_angles = []
        self.end_effector_pos = Vector((0, 0, 0))
        self.end_effector_rot = Euler((0, 0, 0))

    def connect(self):
        """Connect to the C++ control server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"✓ Connected to robot server at {self.host}:{self.port}")

            # Get initial state
            self.get_state()
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from server"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        self.connected = False
        print("✓ Disconnected from server")

    def send_command(self, command):
        """Send command to server and receive updated state"""
        if not self.connected or not self.socket:
            print("✗ Not connected to server")
            return None

        try:
            self.socket.send(command.encode())
            response = self.socket.recv(4096).decode()
            return self.parse_state(response)
        except Exception as e:
            print(f"✗ Communication error: {e}")
            self.connected = False
            return None

    def parse_state(self, state_str):
        """Parse state string from server"""
        # Format: STATE|x,y,z|roll,pitch,yaw|j1,j2,j3,j4,j5,j6
        try:
            parts = state_str.split("|")
            if len(parts) < 4 or parts[0] != "STATE":
                return None

            # Position
            pos = [float(x) for x in parts[1].split(",")]
            self.end_effector_pos = Vector(pos)

            # Orientation (RPY in degrees)
            rpy = [float(x) * math.pi / 180.0 for x in parts[2].split(",")]
            self.end_effector_rot = Euler(rpy, "XYZ")

            # Joint angles (in degrees -> radians)
            self.joint_angles = [
                float(x) * math.pi / 180.0 for x in parts[3].split(",")
            ]

            return {
                "position": self.end_effector_pos,
                "rotation": self.end_effector_rot,
                "joints": self.joint_angles,
            }
        except Exception as e:
            print(f"✗ Parse error: {e}")
            return None

    def move_relative(self, dx, dy, dz):
        """Move end-effector by delta"""
        cmd = f"MOVE_REL|{dx},{dy},{dz}"
        return self.send_command(cmd)

    def move_absolute(self, x, y, z):
        """Move end-effector to absolute position"""
        cmd = f"MOVE_ABS|{x},{y},{z}"
        return self.send_command(cmd)

    def rotate(self, axis_x, axis_y, axis_z, angle_deg):
        """Rotate end-effector around axis by angle"""
        cmd = f"ROTATE|{axis_x},{axis_y},{axis_z},{angle_deg}"
        return self.send_command(cmd)

    def reset(self):
        """Reset robot to home position"""
        return self.send_command("RESET|")

    def get_state(self):
        """Query current robot state"""
        return self.send_command("GET_STATE|")


# =============================================================================
# Robot Visualizer (Updates Blender from Server State)
# =============================================================================


class RobotVisualizer:
    """Updates Blender robot based on server state"""

    def __init__(self, client, config_path):
        self.client = client
        self.config_path = config_path

        # Load config
        with open(config_path) as f:
            self.config = json.load(f)
        self.links = self.config["links"]

        # Build robot model
        self.builder = RobotModelBuilder(config_path)
        self.joints = self.builder.build_robot()

        # End-effector marker
        self.create_end_effector_marker()

        # Trajectory visualization
        self.trajectory_points = []
        self.create_trajectory_curve()

    def create_end_effector_marker(self):
        """Create red sphere for end-effector position"""
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.03)
        self.end_effector = bpy.context.object
        self.end_effector.name = "EndEffector_Marker"

        # Red material
        mat = bpy.data.materials.new(name="EndEffector_Mat")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 0, 0, 1)
        self.end_effector.data.materials.append(mat)

    def create_trajectory_curve(self):
        """Create curve for trajectory visualization"""
        curve_data = bpy.data.curves.new("Trajectory", type="CURVE")
        curve_data.dimensions = "3D"
        curve_data.bevel_depth = 0.002

        # Create a new spline
        spline = curve_data.splines.new("POLY")

        self.trajectory_obj = bpy.data.objects.new("Trajectory", curve_data)
        bpy.context.collection.objects.link(self.trajectory_obj)

        # Yellow material
        mat = bpy.data.materials.new(name="Trajectory_Mat")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 1, 0, 1)
        self.trajectory_obj.data.materials.append(mat)

    def update_from_server(self):
        """Update Blender visualization from server state"""
        state = self.client.get_state()
        if not state:
            return False

        # Update joint rotations
        for i, (joint_obj, link) in enumerate(zip(self.joints, self.links)):
            if i < len(state["joints"]):
                axis = link["axis"]
                axis_idx = AXIS_INDEX[axis]
                angle = state["joints"][i]

                # Set rotation
                joint_obj.rotation_euler[axis_idx] = angle

        # Update end-effector marker
        self.end_effector.location = state["position"]

        # Add to trajectory
        self.trajectory_points.append(state["position"].copy())

        # Update trajectory curve (only keep last 100 points for performance)
        if len(self.trajectory_points) > 100:
            self.trajectory_points.pop(0)

        self.update_trajectory_curve()

        return True

    def update_trajectory_curve(self):
        """Rebuild trajectory curve from points"""
        if len(self.trajectory_points) < 2:
            return

        curve_data = self.trajectory_obj.data
        curve_data.splines.clear()

        spline = curve_data.splines.new("POLY")
        spline.points.add(len(self.trajectory_points) - 1)

        for i, point in enumerate(self.trajectory_points):
            spline.points[i].co = (*point, 1)

    def clear_trajectory(self):
        """Clear trajectory visualization"""
        self.trajectory_points.clear()
        self.update_trajectory_curve()


# =============================================================================
# Blender UI Panel
# =============================================================================


class ROBOT_PT_control_panel(bpy.types.Panel):
    """Robot Control Panel in 3D Viewport"""

    bl_label = "Robot Server Control"
    bl_idname = "ROBOT_PT_control_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot"

    def draw(self, context):
        layout = self.layout

        # Connection status
        if "robot_viz" in bpy.app.driver_namespace:
            viz = bpy.app.driver_namespace["robot_viz"]
            if viz.client.connected:
                layout.label(text="Status: Connected ✓", icon="LINKED")
            else:
                layout.label(text="Status: Disconnected", icon="UNLINKED")
        else:
            layout.label(text="Status: Not Initialized", icon="ERROR")

        layout.separator()

        # Connection controls
        layout.label(text="Connection:")
        row = layout.row()
        row.operator("robot.connect", text="Connect")
        row.operator("robot.disconnect", text="Disconnect")

        layout.separator()

        # Movement controls
        layout.label(text="Position Control:")

        col = layout.column(align=True)
        row = col.row(align=True)
        row.operator("robot.move", text="← Y-").axis = "Y-"
        row.operator("robot.move", text="Y+ →").axis = "Y+"

        row = col.row(align=True)
        row.operator("robot.move", text="← X-").axis = "X-"
        row.operator("robot.move", text="X+ →").axis = "X+"

        row = col.row(align=True)
        row.operator("robot.move", text="↓ Z-").axis = "Z-"
        row.operator("robot.move", text="Z+ ↑").axis = "Z+"

        layout.separator()

        # Rotation controls
        layout.label(text="Rotation Control:")
        row = layout.row(align=True)
        row.operator("robot.rotate", text="Roll -").axis = "X-"
        row.operator("robot.rotate", text="Roll +").axis = "X+"

        row = layout.row(align=True)
        row.operator("robot.rotate", text="Pitch -").axis = "Y-"
        row.operator("robot.rotate", text="Pitch +").axis = "Y+"

        row = layout.row(align=True)
        row.operator("robot.rotate", text="Yaw -").axis = "Z-"
        row.operator("robot.rotate", text="Yaw +").axis = "Z+"

        layout.separator()

        # Utility controls
        layout.label(text="Utilities:")
        layout.operator("robot.reset", text="Reset to Home")
        layout.operator("robot.clear_trajectory", text="Clear Trajectory")
        layout.operator("robot.rebuild", text="Rebuild Robot Model")


# =============================================================================
# Blender Operators
# =============================================================================


class ROBOT_OT_connect(bpy.types.Operator):
    bl_idname = "robot.connect"
    bl_label = "Connect to Robot Server"

    def execute(self, context):
        if "robot_viz" not in bpy.app.driver_namespace:
            # Initialize visualizer
            client = RobotControlClient(SERVER_HOST, SERVER_PORT)
            if client.connect():
                viz = RobotVisualizer(client, CONFIG_PATH)
                bpy.app.driver_namespace["robot_viz"] = viz

                # Start update timer
                if not bpy.app.timers.is_registered(update_robot_visualization):
                    bpy.app.timers.register(update_robot_visualization)

                self.report({"INFO"}, "Connected to robot server")
            else:
                self.report({"ERROR"}, "Failed to connect to server")
                return {"CANCELLED"}
        else:
            viz = bpy.app.driver_namespace["robot_viz"]
            if viz.client.connect():
                self.report({"INFO"}, "Reconnected to robot server")
            else:
                self.report({"ERROR"}, "Failed to reconnect")
                return {"CANCELLED"}

        return {"FINISHED"}


class ROBOT_OT_disconnect(bpy.types.Operator):
    bl_idname = "robot.disconnect"
    bl_label = "Disconnect from Robot Server"

    def execute(self, context):
        if "robot_viz" in bpy.app.driver_namespace:
            viz = bpy.app.driver_namespace["robot_viz"]
            viz.client.disconnect()
            self.report({"INFO"}, "Disconnected from server")

        return {"FINISHED"}


class ROBOT_OT_move(bpy.types.Operator):
    bl_idname = "robot.move"
    bl_label = "Move Robot"

    axis: bpy.props.StringProperty()

    def execute(self, context):
        if "robot_viz" not in bpy.app.driver_namespace:
            self.report({"ERROR"}, "Not connected to robot")
            return {"CANCELLED"}

        viz = bpy.app.driver_namespace["robot_viz"]
        if not viz.client.connected:
            self.report({"ERROR"}, "Not connected to server")
            return {"CANCELLED"}

        step = 0.05  # 5cm movements

        move_map = {
            "X+": (step, 0, 0),
            "X-": (-step, 0, 0),
            "Y+": (0, step, 0),
            "Y-": (0, -step, 0),
            "Z+": (0, 0, step),
            "Z-": (0, 0, -step),
        }

        if self.axis in move_map:
            dx, dy, dz = move_map[self.axis]
            result = viz.client.move_relative(dx, dy, dz)
            if result:
                viz.update_from_server()
                self.report({"INFO"}, f"Moved {self.axis}")
            else:
                self.report({"WARNING"}, "Move failed")

        return {"FINISHED"}


class ROBOT_OT_rotate(bpy.types.Operator):
    bl_idname = "robot.rotate"
    bl_label = "Rotate Robot"

    axis: bpy.props.StringProperty()

    def execute(self, context):
        if "robot_viz" not in bpy.app.driver_namespace:
            self.report({"ERROR"}, "Not connected to robot")
            return {"CANCELLED"}

        viz = bpy.app.driver_namespace["robot_viz"]
        if not viz.client.connected:
            self.report({"ERROR"}, "Not connected to server")
            return {"CANCELLED"}

        angle = 15.0  # 15 degree rotations

        rotate_map = {
            "X+": (1, 0, 0, angle),
            "X-": (1, 0, 0, -angle),
            "Y+": (0, 1, 0, angle),
            "Y-": (0, 1, 0, -angle),
            "Z+": (0, 0, 1, angle),
            "Z-": (0, 0, 1, -angle),
        }

        if self.axis in rotate_map:
            ax, ay, az, ang = rotate_map[self.axis]
            result = viz.client.rotate(ax, ay, az, ang)
            if result:
                viz.update_from_server()
                self.report({"INFO"}, f"Rotated {self.axis}")
            else:
                self.report({"WARNING"}, "Rotation failed")

        return {"FINISHED"}


class ROBOT_OT_reset(bpy.types.Operator):
    bl_idname = "robot.reset"
    bl_label = "Reset Robot"

    def execute(self, context):
        if "robot_viz" not in bpy.app.driver_namespace:
            self.report({"ERROR"}, "Not connected to robot")
            return {"CANCELLED"}

        viz = bpy.app.driver_namespace["robot_viz"]
        if not viz.client.connected:
            self.report({"ERROR"}, "Not connected to server")
            return {"CANCELLED"}

        viz.client.reset()
        viz.update_from_server()
        self.report({"INFO"}, "Robot reset to home position")

        return {"FINISHED"}


class ROBOT_OT_clear_trajectory(bpy.types.Operator):
    bl_idname = "robot.clear_trajectory"
    bl_label = "Clear Trajectory"

    def execute(self, context):
        if "robot_viz" in bpy.app.driver_namespace:
            viz = bpy.app.driver_namespace["robot_viz"]
            viz.clear_trajectory()
            self.report({"INFO"}, "Trajectory cleared")

        return {"FINISHED"}


class ROBOT_OT_rebuild(bpy.types.Operator):
    bl_idname = "robot.rebuild"
    bl_label = "Rebuild Robot Model"

    def execute(self, context):
        if "robot_viz" in bpy.app.driver_namespace:
            viz = bpy.app.driver_namespace["robot_viz"]
            viz.builder.build_robot()
            viz.joints = viz.builder.joints
            self.report({"INFO"}, "Robot model rebuilt")
        else:
            # Build without connection
            builder = RobotModelBuilder(CONFIG_PATH)
            builder.build_robot()
            self.report({"INFO"}, "Robot model built (not connected)")

        return {"FINISHED"}


# =============================================================================
# Timer Function
# =============================================================================


def update_robot_visualization():
    """Timer function to continuously update visualization"""
    if "robot_viz" in bpy.app.driver_namespace:
        viz = bpy.app.driver_namespace["robot_viz"]
        if viz.client.connected:
            viz.update_from_server()
    return 0.1  # Update every 0.1 seconds


# =============================================================================
# Registration
# =============================================================================

classes = (
    ROBOT_PT_control_panel,
    ROBOT_OT_connect,
    ROBOT_OT_disconnect,
    ROBOT_OT_move,
    ROBOT_OT_rotate,
    ROBOT_OT_reset,
    ROBOT_OT_clear_trajectory,
    ROBOT_OT_rebuild,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    print("✓ Robot Server Control registered!")
    print("  → Look for 'Robot' tab in 3D Viewport sidebar (N key)")
    print("  → Click 'Rebuild Robot Model' to build without server")
    print("  → Click 'Connect' to link with C++ control server")


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    if bpy.app.timers.is_registered(update_robot_visualization):
        bpy.app.timers.unregister(update_robot_visualization)

    if "robot_viz" in bpy.app.driver_namespace:
        viz = bpy.app.driver_namespace["robot_viz"]
        viz.client.disconnect()
        del bpy.app.driver_namespace["robot_viz"]

    print("✓ Robot Server Control unregistered")


# =============================================================================
# Auto-run
# =============================================================================

if __name__ == "__main__":
    register()

    # Optionally build robot model immediately
    # (even without server connection, for visualization)
    if "robot_viz" not in bpy.app.driver_namespace:
        builder = RobotModelBuilder(CONFIG_PATH)
        builder.build_robot()
        print("✓ Robot model built (offline mode)")
        print("  → Click 'Connect' in Robot panel to enable control")
