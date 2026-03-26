# Gazebo Drone Simulator (S-W-A-R-M)

This repository provides a containerized development environment for drone swarm coordination research. It is configured to use **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic** to ensure a modern, high-fidelity simulation environment.

##  Prerequisites

Before starting, ensure you have the following installed:
*   **Docker** and **Docker Compose**
*   **WSL 2** (if using Windows)
*   **NVIDIA Container Toolkit** (Optional, for GPU acceleration)

---

##  Getting Started

### 1. Build the Environment
Build the Docker image. This will install all ROS 2 and Gazebo dependencies.

```bash
docker compose build
```
### 2. Start the Container
Launch the container in the background (detached mode) so it stays active while you work.
```
docker compose up -d
```
### 3. Enter the Workspace
Open an interactive bash shell inside the running container. This is where you will run your ROS nodes and build your packages.
```
docker exec -it swarm_dev bash
```
Note: Once inside, your workspace is located at ~/swarm_ws.
##  Development Workflow
Opening Multiple Terminals
To open additional terminal sessions (e.g., for monitoring topics or running multiple nodes), simply run the exec command again in a new window:
```
docker exec -it swarm_dev bash
```
##  Shutting Down
When you are finished with your session, stop and remove the containers:
```
docker compose down
```
