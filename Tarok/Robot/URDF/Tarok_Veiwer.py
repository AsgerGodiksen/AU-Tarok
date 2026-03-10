import yourdfpy
import os

# Always resolve paths relative to this script's location
script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path  = os.path.join(script_dir, "robot.urdf")

robot = yourdfpy.URDF.load(
    urdf_path,
    filename_handler=lambda fname: fname.replace(
        "package://", script_dir + "/"
    ),
)
robot.show()