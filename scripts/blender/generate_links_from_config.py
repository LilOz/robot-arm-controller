import bpy
import json
import math
from mathutils import Vector
import colorsys

CONFIG_PATH = "/Users/ayman/dev/projects/robot-arm-controller/config/models/6dof_spherical_model.json"

AXIS_INDEX = {
    "X": 0,
    "Y": 1,
    "Z": 2,
}

with open(CONFIG_PATH) as f:
    config = json.load(f)

links = config["links"]

cursor = Vector((0, 0, 0))
joints = []

# ---------- utility: material ----------
def make_material(name, idx, total):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True

    # Evenly spaced hues
    h = idx / max(1, total)
    r, g, b = colorsys.hsv_to_rgb(h, 0.8, 0.8)

    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (r, g, b, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.4
    return mat


materials = [
    make_material(f"LinkMat_{i}", i, len(links))
    for i in range(len(links))
]

# ---------- build chain ----------
for i, link in enumerate(links):
    name = link["name"]
    length = link["length"]

    # --- Joint empty ---
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=cursor)
    joint = bpy.context.object
    joint.name = name
    joint.rotation_mode = 'XYZ'
    joints.append(joint)

    mat = materials[i]

    # --- Link mesh ---
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
        # Zero-length wrist joint → thin offset plane ("flag")
        bpy.ops.mesh.primitive_cube_add(size=1)
        mesh = bpy.context.object
        mesh.name = f"{name}_flag"

        mesh.parent = joint

        # Thin rectangle
        if link['axis'] == 'X':
            mesh.scale = (0.05, 0.025, 0.01)
        elif link['axis'] == 'Y':
            mesh.scale = (0.025, 0.05, 0.01)
        else:
            mesh.scale = (0.01, 0.01, 0.05)
            mesh.location = (0, 0, 0.025)

        mesh.data.materials.append(mat)

    # --- Advance frame ---
    cursor += Vector((0, 0, length))

# ---------- empty display scale ----------
for obj in bpy.data.objects:
    if obj.type == 'EMPTY':
        obj.empty_display_size = 0.05

# ---------- parent joints LAST ----------
for parent, child in zip(joints[:-1], joints[1:]):
    child.parent = parent
    child.matrix_parent_inverse = parent.matrix_world.inverted()
