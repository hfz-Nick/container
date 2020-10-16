# `navatics_can_interface`

## Briefs

This package contains code that allows Upboard to communicate with main controller, which interfaces with the CAN BUS network. This document shows how it interfaces with main controller board.

## Dependencies

This package requires:

1. rclcpp
2. [serial](https://github.com/RoverRobotics-forks/serial-ros2) -> this is needed in place of ros-serial package in ROS1
3. [`navatics_msgs`](http://192.168.69.40/uusv/ros2/navatics_msgs)

## Building the Docker Image and ROS packages

Follow instructions in `uusv_core_container` repository for more dtails

## Running the ROS2 node

The parameters needed to run this code can be found in `uusv_core_container` repository.

```
ros2 run navatics_can_interface can_interface_node --ros-args --params-file ~/params/can_interface_params.yaml
```

## Message Protocol

This code subscribes to the CANMessage.msg topic, which allows the node to construct USART message to be sent to the F1 main controller. By default, the `can_interface_node` will subscribe to `can/transmit` topic. In order to send CAN message, user need to publish to this topic following the format of CANMessage.msg as seen in [`navatics_msgs`](http://192.168.69.40/uusv/ros2/navatics_msgs)

Specifically for motor driver, user can use the following as a sample message:

```
auto msg = navatics_msgs::msg::CanBus()
msg.std_id = 512
msg.length = 8
msg.data = [0, 255, 0, 255, 0, 255, 0, 255]
```

This will publish a message with ID = 512 (0x200) with message[0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF], which will cause the motor that subscribes to 0x200 message to rotate at 255 RPM.

The command used to publish this message is:

```
ros2 topic pub /can/transmit navatics_msgs/CanBus "{std_id: 512, length: 8, data: [0, 255, 0, 255, 0, 255, 0, 255]}"
```

The message received by USARTx will look like the following:

`0xBE 0xEF 0x06 0x00 0x20 0x00 0xFF 0x00 0xFF 0x00 0xFF 0x00 0xFF 0xCA 0xFE`

Where:

1. `0xBE 0xEF`: Start Bytes
2. `0x06`: Message Length (In this case, 2 ID and 4 Data)
3. `0x07 0xFF`: SFF ID (11 Bytes, 2048 possible IDs)
4. `0x00 0xFF 0x00 0xFF 0x00 0xFF 0x00 0xFF` : Data (Maximum length: 8 Bytes)
5. `0xCA 0xFE`: Stop Bytes

Hook the USART up with the STM32F1 main controller running [`navatics_usart_can_controller`](http://192.168.69.40/alpha_bot_r/navatics_usart_can_controller) to send out the CAN message to other CAN devices in the bus
