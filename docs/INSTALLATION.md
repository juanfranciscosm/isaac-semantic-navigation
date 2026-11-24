# Installation and Environment

## Development environment

- Ubuntu 24.04.
- ROS 2 Jazzy Jalisco.
- NVIDIA Isaac Sim 5.1.0.
- NVIDIA GeForce RTX 3090.
- Fast DDS.
- Nav2.
- Python.
- Separate Python and Conda environments.

## Repository

    git clone https://github.com/juanfranciscosm/isaac-semantic-navigation.git ~/indoor

## Workspace compilation

    cd ~/indoor
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

## ROS communication

    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

## CLIPSeg dependencies

    src/clipseg_ros/requirements.txt

Use the dedicated Python environment described in README.md.

## Execution sequence

1. Start Isaac Sim.
2. Open the warehouse scene.
3. Start simulation.
4. Launch Nav2.
5. Configure RViz.
6. Start CLIPSeg.
7. Start the semantic localizer.
8. Start the semantic navigator.
9. Verify and enable the navigation goal.

See README.md for detailed execution commands.

See VLMAPS_INTEGRATION.md for the alternative mapping approach.


