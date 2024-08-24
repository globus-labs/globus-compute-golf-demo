# Golf Ball Simulation

This project is a physics-based simulation of golf balls on a procedurally generated terrain using PyBullet. The simulation creates a visual representation of a golf green where multiple golf balls are dropped, and their movements are tracked to study their final positions on the terrain.

## Project Structure

This project uses a `pyproject.toml` file to manage dependencies and build configurations. All necessary packages will be automatically installed when setting up the project.

## Installation Instructions

1. **Clone the repository and navigate to the project directory:**
    ```bash
    git clone <repository-url>
    cd Pybullet\ Simulations
    ```

2. **Create a virtual environment:**
    ```bash
    python -m venv globus_compute_venv
    source globus_compute_venv/bin/activate
    ```

3. **Install the package and dependencies using the pyproject.toml:**
    ```bash
    pip install .
    ```

   This command will install all required dependencies listed in the `pyproject.toml` file, including `numpy`, `pybullet`, `matplotlib`, `globus-compute-sdk`, and `globus-compute-endpoint`.

## Running the Simulation

To run the simulation, execute the `run.py` or `run_globus_compute.py` script, which interfaces with the functionalities provided in the `simulation` package. The script can be run in two modes:

- **Headless Mode (No GUI):** For automated runs, useful for data collection or running on servers.
- **GUI Mode:** For visualizing the simulation in real-time.

### Commands

- **Run Simulation with GUI:**
    ```bash
    python run.py --gui
    ```

- **Run Simulation Headless (No GUI):**
    ```bash
    python run.py
    ```

### Configuration Parameters

You can adjust the following parameters via command-line arguments:

- `width` (int): Width of the golf green.
- `height` (int): Height of the golf green.
- `scale` (float): Scale of the noise map affecting terrain roughness.
- `octaves` (int): Number of octaves for noise generation.
- `persistence` (float): Persistence factor of the noise.
- `lacunarity` (float): Lacunarity of the noise.
- `height_multiplier` (float): Multiplier for the terrain height to exaggerate vertical features.
- `num_balls` (int): Number of golf balls to simulate.

### Example
```bash
python run.py --width 100 --height 100 --scale 10.0 --octaves 2 --persistence 0.5 --lacunarity 2.0 --height_multiplier 2.0 --num_balls 10 --gui
```

## Configure and Start Globus Compute Endpoint
The Globus Compute endpoint can only be configured and run on Linux-based systems, as it requires Linux-specific dependencies and environment settings.

To use Globus Compute for distributed execution:

1. **Configure your endpoint:**
    ```bash
    globus-compute-endpoint configure my_endpoint
    ```

2. **Start the endpoint:**
    ```bash
    globus-compute-endpoint start my_endpoint
    ```
### Running the Simulation with Globus Compute

- **Once your endpoint is active:**
    ```bash
    python run_globus_compute.py --globus_compute --endpoint_id <your-endpoint-id>

    ```
- Replace <your-endpoint-id> with the actual ID of your Globus Compute endpoint.

- This will submit the task to the specified Globus Compute endpoint, enabling scalable distributed execution of your simulation.

## Example Outputs

- Simulation:
  ![alt text](<Images/Golf green simulation.jpg>)

- 2-D Contour Plot:
  ![alt text](<Images/2-d Contour plot.png>)

- 3-D Heatmap:
  ![alt text](<Images/3-d heatmap.jpg>)