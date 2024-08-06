Golf Ball Simulation

This project is a physics-based simulation of golf balls on a procedurally generated terrain using PyBullet. The simulation creates a visual representation of a golf green where multiple golf balls are dropped and their movements are tracked to study their final positions on the terrain.

Dependencies:
Ensure you have the following dependencies installed to run this project:

Python 3.8+
PyBullet
NumPy
Matplotlib
SciPy

You can install the required packages using pip:
pip install pybullet numpy matplotlib scipy

Running the Simulation:
To run the simulation, you need to execute the run.py script which interfaces with the functionalities provided in simulate.py. The script can be run in two modes:

Headless Mode (No GUI): For automated runs, useful for data collection or running on servers.
GUI Mode: For visualizing the simulation in real-time.

Commands:

Run Simulation with GUI:
python run.py --gui

Run Simulation Headless (No GUI):
python run.py

Configuration Parameters
You can adjust the following parameters via command-line arguments:

width (int): Width of the golf green.
height (int): Height of the golf green.
scale (float): Scale of the noise map affecting terrain roughness.
octaves (int): Number of octaves for noise generation.
persistence (float): Persistence factor of the noise.
lacunarity (float): Lacunarity of the noise.
height_multiplier (float): Multiplier for the terrain height to exaggerate vertical features.
num_balls (int): Number of golf balls to simulate.

Example:
python run.py --width 100 --height 100 --scale 10.0 --octaves 2 --persistence 0.5 --lacunarity 2.0 --height_multiplier 2.0 --num_balls 10 --gui