# Project SWARM

Project **SWARM** is an undergraduate research initiative at the **University of Washington Bothell Distributed Systems Laboratory (DSLab)**. This project focuses on developing an autonomous drone swarm capable of high-fidelity heat mapping and perimeter identification in dynamic forest fire environments using distributed computing and evolutionary algorithms.

---

##  Project Roadmap

The development is structured into three phases to bridge the gap between simulation and hardware:

1. **Phase 1: Reflexes** – Core autonomous flight, obstacle avoidance, and basic sensing using **ROS 2** and **Gazebo Harmonic**.
2. **Phase 2: Social** – Distributed messaging protocols for real-time temperature data sharing and swarm coordination.
3. **Phase 3: Brain** – Integration of the **MASS CUDA** library to run parallelized evolutionary algorithms on **NVIDIA Jetson Orin** hardware for strategic behavior selection.

---

## File Structure

This repository uses a dual-environment Docker setup to ensure seamless collaboration across different operating systems.

```text
TESTSWARM/
├── docker/
│   ├── mac/               # MacOS environment (TigerVNC for Apple Silicon)
│   │   ├── docker-compose.mac.yml
│   │   ├── dockerfile
│   │   └── entrypoint.sh
│   └── windows/           # Windows environment (WSLg for native GUI)
│       ├── docker-compose.windows.yml
│       ├── dockerfile
│       └── entrypoint.sh
├── src/                   # ROS 2 Source Code
│   └── swarm_test/        # Primary research package
└── README.md
```
## Installation & Setup
Prerequisites

- Docker Desktop (Installed and running)

- TigerVNC Viewer (Required for macOS users)

## Windows (WSL2/WSLg)

Windows 11 users can leverage WSLg to forward the Gazebo GUI directly to the Windows desktop.

1. **Launch the container:**

```
cd docker/windows
docker compose up -d
```

2. **Access the Shell:**

```
docker exec -it swarm_dev bash
```

## MacOS (Apple Silicon) Setup

This setup uses a VNC server inside the container to handle Gazebo's 3D rendering on Apple Silicon hardware.

1. **Launch the container:**

```
cd docker/mac
docker compose -f docker-compose.mac.yml up -d
```

2. **Visual Interface:**
 Open TigerVNC Viewer and connect to ``localhost:5900``. Leave the password blank.

Access the Shell:

```
docker exec -it swarm_dev bash
```


## Development Workflow
Once inside the container, follow these steps to build and run your nodes:

#### Initialize and Build (should only need to build once)

```
# Ensure workspace permissions
sudo chown -R swarm:swarm /swarm_ws
cd /swarm_ws
```

#### Build the workspace
```
colcon build
source install/setup.bash
```

```
# Start Gazebo Harmonic
gz sim -r empty.sdf
```

#### In a separate terminal tab
```
ros2 run swarm_test my_first_node
```
