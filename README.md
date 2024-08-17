# Sparkie Firmware

This is the firmware that runs on the custom made embedded raspberry pico board. It handles every interaction with sensors and actuators by connecting it to micro ros.

## Installation

- Clone the repository and all submodules.
```bash
git clone --recurse-submodules https://github.com/sparkiebot/sparkie_fw.git
```
- Modify `config.hpp` to match your pin configuration and change other options

- Compile it
```bash
mkdir build
cd build
cmake ..
make -j
```
- Turn your pico into programming mode and deploy the `sparkie_fw.uf2` file located in `build/src/sparkie_fw.uf2` either using `picotool` or by copying file into pico storage.

Once done deploying, the board will turn on the status led waiting for a new micro ros connection.

## Architecture

For convenience, a real time operating system (specifically [FreeRTOS](https://github.com/FreeRTOS/FreeRTOS)) has been used, not only to 
better manage multiple sensors and actuators concurrently, but also to make use of the other core supplied in the RP2040 (rapsberry pico chip). Some tasks (process are called tasks in the freertos ecosystem) run one core and some on the other one. This has been done to manually balance the workload.

### MicroROS

As ROS is being used on the robot, I've decided to also integrate it directly into the board. Actually, the board uses a framework called [MicroROS](https://micro.ros.org/) that communicates with the main computer with a special ros node, named **micro ros agent**, that will publish and subscribes to the required topics and services. 

### Component based system

Inspired by [this repo](https://github.com/jondurrant/RPIPicoFreeRTOSuROSPubSub), a component based system has been used.
The main idea is that every sensor, actuator, etc is a component, specifically a micro ros component.
A **micro ros component** is a freertos task, with some handful functions to easily publish and subscribe to topics and services.

When a component tries to interact with the ros network, it actually sends the data to the corresponding queue.
All the queues are then handled by a special component named "*AgentComponent*", which manages the actual connection with the micro ros agent. It sends and receives all the ros messages by manipulating the queues of every micro ros components previously added to it.

### ROS Topics

The board once connected, spawns a node named `/sparkie/board` and makes available / subscribes to the following topics:
- `/sparkie/board/cpu` and `/sparkie/board/memory`, show some stats about the tasks running
- `/sparkie/board/battery`
- `/sparkie/board/head/tilt`, publishing a float32 value between the specified range, will make the head servo move at the specific angle.
- `/sparkie/board/temperature`, `/sparkie/board/temperature/humidity` and `/sparkie/board/air/co2` are supposed to show environmental data.
- `/sparkie/board/imu` and `/sparkie/board/mag` show data from a chinese [GY-85](https://github.com/mattsays/gy85) module.
- `/sparkie/board/us/*`, publish ultrasonic sensor data
- `/sparkie/board/wheels/vel` publishes `irobot_create_msgs/WheelVels`, all the values are expressed in RPM.
- `/cmd_vel`, subscribes to it and changes motors speed according to the constrains configured in `config.hpp`
- `/sparkie/board/system` subscribes to it and reboots the board or enters programming mode (0 for reboot, 1 for programming mode)

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Special thanks to @Filippo-Galli for helping me optimize the firmware software with his C++ optimization magic.

## License

[MIT](https://choosealicense.com/licenses/mit/)