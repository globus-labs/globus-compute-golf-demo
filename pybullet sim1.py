import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Loading a plain plane as the ground
plane_id = p.loadURDF("plane.urdf")

# Creating the Sphere(need to figure out how to make it a bowl shape)
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=1,
    rgbaColor=[0, 1, 0, 0.5],
    specularColor=[0.4, 0.4, 0]
)

collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_SPHERE,
    radius=1
)

#Bowl starting transformation
bowl_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 0, 0.5]  # Half of radius to keep the bowl above the plane
)

# Creating the golf ball
ball_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05,
    rgbaColor=[1, 0, 0, 1],
    specularColor=[0.4, 0.4, 0]
)

ball_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05
)


ball_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=ball_collision_shape_id,
    baseVisualShapeIndex=ball_visual_shape_id,
    basePosition=[0.5, 0.5, 2]  # Drop from a position above and to the side of the bowl
)

# Setting to gravity
p.setGravity(0, 0, -9.81)

# Run the simulation
for _ in range(2400):  # Run for 10 seconds (240 steps per second)
    p.stepSimulation()
    time.sleep(1/240)


time.sleep(5)

# Disconnect from PyBullet
p.disconnect()
