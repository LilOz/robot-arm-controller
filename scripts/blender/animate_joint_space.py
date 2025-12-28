import bpy
import csv
import json

CONFIG_PATH = "/Users/ayman/dev/projects/robot-arm-controller/config/models/6dof_spherical_model.json"
CSV_PATH = "/Users/ayman/dev/projects/robot-arm-controller/build/examples/trajectory_joint_space.csv"

FPS = bpy.context.scene.render.fps
START_FRAME = 1

AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}

with open(CONFIG_PATH) as f:
    config = json.load(f)

for link in config["links"]:
    obj = bpy.data.objects[link["name"]]
    obj.animation_data_clear()
    obj.rotation_mode = 'XYZ'

with open(CSV_PATH) as f:
    reader = csv.DictReader(f)

    for row in reader:
        t_ms = float(row["t_ms"])
        frame = START_FRAME + round(t_ms * FPS / 1000.0)
        bpy.context.scene.frame_set(int(frame))

        for i, link in enumerate(config["links"]):
            name = link["name"]
            axis = link["axis"]

            joint = bpy.data.objects[name]
            axis_i = AXIS_INDEX[axis]

            angle = float(row[("joint_"+str(i+1))])  # radians

            joint.rotation_euler[axis_i] = angle
            joint.keyframe_insert(
                data_path="rotation_euler",
                index=axis_i
            )

for link in config["links"]:
    obj = bpy.data.objects[link["name"]]
    if obj.animation_data and obj.animation_data.action:
        for fcurve in obj.animation_data.action.fcurves:
            for kp in fcurve.keyframe_points:
                kp.interpolation = 'LINEAR'
