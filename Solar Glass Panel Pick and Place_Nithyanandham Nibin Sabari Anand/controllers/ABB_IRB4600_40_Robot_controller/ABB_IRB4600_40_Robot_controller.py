"""Robot_controller controller."""

from controller import Supervisor
import ikpy
from ikpy.chain import Chain
import tempfile
import os

# Initialize the Robot Supervisor
robot = Supervisor()

# Get the timestep of the current world
timestep = int(robot.getBasicTimeStep())

# Load the robot's URDF file
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(robot.getUrdf().encode('utf-8'))

# Initialize the robot's arm chain using the URDF file
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, False, True, True, False])

# Robot parameters and devices
joint_names = ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']
init_positions = [0, 0, 0, 0, 0, 0]
robot_joints = []
velocity = 5
touch_sensor = robot.getDevice("touch sensor")
vacuum_gripper = robot.getDevice("vacuum gripper")
distance_sensor = robot.getDevice("distance sensor")
glass_count = 0

# Enable sensors
touch_sensor.enable(timestep)
distance_sensor.enable(timestep)

# Initialize joints
for i in range(6):
    joint = robot.getDevice(joint_names[i])
    robot_joints.append(joint)
    joint.setPosition(init_positions[i])
    joint.setVelocity(velocity)

# Functions in sequence of simulation operations

def pick_glass(z):
    """
    Move the arm to pick up a glass panel at height z.
    """
    x = 0
    y = -1.4
    ikSolution = armChain.inverse_kinematics([x, y, z])
    robot_joints[4].setPosition(-(ikSolution[2] + ikSolution[3]))
    robot_joints[5].setPosition(0)
    for i in range(3):
        robot_joints[i].setPosition(ikSolution[i + 1])

def attach_glass():
    """
    Turn on the vacuum gripper to attach the glass panel.
    """
    vacuum_gripper.turnOn()
    vacuum_gripper.enablePresence(timestep)

def lift_glass():
    """
    Lift the glass panel to a default position.
    """
    x = 0
    y = -1.4
    z = 1
    ikSolution = armChain.inverse_kinematics([x, y, z])
    robot_joints[4].setPosition(-(ikSolution[2] + ikSolution[3]))
    robot_joints[5].setPosition(0)
    for i in range(3):
        robot_joints[i].setPosition(ikSolution[i + 1])

def rotate_glass():
    """
    Rotate the glass panel to align with the placement position.
    """
    x = -1.4
    y = 0
    z = 1
    ikSolution = armChain.inverse_kinematics([x, y, z])
    robot_joints[4].setPosition((ikSolution[2] + ikSolution[3]) * -1)
    robot_joints[5].setPosition(-1.57)
    for i in range(3):
        robot_joints[i].setPosition(ikSolution[i + 1])

def place_glass(z):
    """
    Place the glass panel at height z.
    """
    x = -1.4
    y = 0
    ikSolution = armChain.inverse_kinematics([x, y, z])
    robot_joints[4].setPosition(-(ikSolution[2] + ikSolution[3]))
    robot_joints[5].setPosition(-1.57)
    for i in range(3):
        robot_joints[i].setPosition(ikSolution[i + 1])

def detach_glass():
    """
    Turn off the vacuum gripper to release the glass panel.
    """
    vacuum_gripper.turnOff()
    vacuum_gripper.disablePresence()

def reset_position():
    """
    Reset the robot arm to its default position.
    """
    x = -1.10
    y = 0
    z = 0.425
    ikSolution = armChain.inverse_kinematics([x, y, z])
    robot_joints[4].setPosition((ikSolution[2] + ikSolution[3]) * -1)
    robot_joints[5].setPosition(-1.57)
    for i in range(3):
        robot_joints[i].setPosition(ikSolution[i + 1])

# Main loop
lift_glass()  # Move to a default position
robot.step(2000)  # Wait to initialize everything
previous_time = robot.getTime()
distance_value = distance_sensor.getValue()

while robot.step(timestep) != -1 and glass_count < 25:
    current_time = robot.getTime()
    elapsed_time = current_time - previous_time

    if 0 < elapsed_time <= 1:
        z = round(1 - (((1 - 0.327) * distance_value) / 590.100), 5)  # Calculate z height
        pick_glass(z)
        attach_glass()
    elif 1 < elapsed_time <= 2:
        lift_glass()
    elif 2 < elapsed_time <= 3:
        rotate_glass()
    elif 3 < elapsed_time <= 4:
        place_glass(0.6634)
    elif 4 < elapsed_time <= 4.1:
        detach_glass()
    elif 4.1 < elapsed_time <= 5.1:
        lift_glass()
    elif elapsed_time > 5.3:
        previous_time = robot.getTime()
        distance_value = distance_sensor.getValue()
        glass_count += 1
        print(f"Processed glass sheet {glass_count}")

if os.path.exists(filename):
    os.remove(filename)
    print("Temporary URDF file deleted:", filename)
