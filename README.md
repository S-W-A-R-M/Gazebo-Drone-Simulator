# Project SWARM
### Swarm-based Wildfire Autonomous Reconnaissance & Mapping

A distributed autonomous drone swarm system for mapping heat signatures in dynamic forest fire environments. Built on ROS 2 Jazzy and Gazebo Harmonic, designed for edge AI deployment on NVIDIA Jetson hardware.

> Developed at the **UW Bothell Distributed Systems Laboratory (DSLab)**  
> University of Washington — Computer Engineering Undergraduate Research

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS 2 Jazzy Jalisco |
| Simulation | Gazebo Harmonic |
| Language | C++ |
| Environment | Docker + WSL2 (Windows) / VNC (Mac) |
| Target Hardware | Holybro X500 V2 + NVIDIA Jetson Orin |

---

## Repository Structure

```
testSWARM/
├── docker/
│   ├── mac/
│   │   ├── docker-compose.yml
│   │   ├── dockerfile
│   │   └── entrypoint.sh
│   └── windows/
│       ├── docker-compose.yml
│       ├── dockerfile
│       └── entrypoint.sh
├── src/                          # ROS 2 packages (shared across platforms)
│   └── swarm_drone/
│       ├── src/
│       │   └── thermal_sensor_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

---

## Prerequisites

### Windows
- Windows 11 with WSL2 enabled
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) with WSL2 backend enabled
- WSLg (included with WSL2 updates — run `wsl --update` to ensure latest)
- Ubuntu distro installed via WSL2

### Mac
- [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
- A VNC viewer (e.g. [RealVNC](https://www.realvnc.com/en/connect/download/viewer/))

---

## Setup — Windows

### 1. Clone the Repository (in WSL2 terminal)
```bash
git clone https://github.com/YOUR_USERNAME/testSWARM.git
cd testSWARM
```

### 2. Build the Docker Image
```bash
docker compose -f docker/windows/docker-compose.yml build
```
> First build takes ~5–10 minutes. Subsequent builds use cached layers and are much faster.

### 3. Start the Container
```bash
docker compose -f docker/windows/docker-compose.yml up -d
```

### 4. Open a Shell Inside the Container
```bash
docker exec -it swarm_dev bash
```

### 5. Verify the Installation
```bash
# Check ROS 2
ros2 --version

# Check Gazebo
gz sim --version

# Launch an empty Gazebo world (GUI should open on your Windows desktop via WSLg)
gz sim empty.sdf
```

### Subsequent Sessions
```bash
# Start container
docker compose -f docker/windows/docker-compose.yml up -d

# Attach a shell
docker exec -it swarm_dev bash

# Open additional terminals (run this in each new terminal)
docker exec -it swarm_dev bash
```

### Stop the Container
```bash
docker compose -f docker/windows/docker-compose.yml down
```

---

## Setup — Mac

### 1. Clone the Repository
```bash
git clone https://github.com/YOUR_USERNAME/testSWARM.git
cd testSWARM
```

### 2. Build the Docker Image
```bash
docker compose -f docker/mac/docker-compose.yml build
```

### 3. Start the Container
```bash
docker compose -f docker/mac/docker-compose.yml up -d
```

### 4. Connect via VNC
1. Open your VNC viewer
2. Connect to `localhost:5900`
3. You now have a desktop environment running inside the container — Gazebo GUI will render here

### 5. Open a Shell Inside the Container
```bash
docker exec -it swarm_dev bash
```

### 6. Verify the Installation
```bash
# Check ROS 2
ros2 --version

# Check Gazebo
gz sim --version

# Launch an empty Gazebo world (GUI will appear in your VNC window)
gz sim empty.sdf
```

### Subsequent Sessions
```bash
# Start container
docker compose -f docker/mac/docker-compose.yml up -d

# Connect VNC viewer to localhost:5900

# Attach a shell
docker exec -it swarm_dev bash
```

### Stop the Container
```bash
docker compose -f docker/mac/docker-compose.yml down
```

---

## Development Workflow

All ROS 2 source code lives in `src/` and is volume-mounted into the container at `/swarm_ws/src`. Edits made in VS Code on your host machine are immediately reflected inside the container — no rebuild required.

### Build the ROS 2 Workspace
```bash
# Inside the container
cd /swarm_ws
colcon build --symlink-install
source install/setup.bash
```

### Run a Node
```bash
ros2 run swarm_drone thermal_sensor_node
```

### Monitor Topics (second terminal)
```bash
docker exec -it swarm_dev bash
ros2 topic echo /thermal_data
```

### Useful ROS 2 Commands
```bash
ros2 topic list              # list all active topics
ros2 topic echo /topic_name  # print live messages on a topic
ros2 node list               # list all running nodes
ros2 node info /node_name    # inspect a node's publishers/subscribers
rqt_graph                    # visualize the node/topic graph
rviz2                        # open the ROS visualization tool
```

---

## Roadmap

- [x] **Phase 0** — Development environment (ROS 2 Jazzy + Gazebo Harmonic, Docker, Windows + Mac)
- [ ] **Phase 1 — Reflexes** — Autonomous flight and thermal sensing stack
- [ ] **Phase 2 — Social** — Distributed messaging for real-time data sharing between drones
- [ ] **Phase 3 — Brain** — CUDA-accelerated evolutionary algorithms via MASS for swarm behavior optimization

---

## Notes

- `ROS_DOMAIN_ID` is set to `42` across all containers — all nodes on the same machine will discover each other automatically
- Build artifacts (`build/`, `install/`, `log/`) are stored in Docker named volumes and persist across container restarts
- Source code in `src/` is the only thing you need to back up — everything else can be rebuilt