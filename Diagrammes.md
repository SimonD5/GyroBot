**Use Case**

The goal of this project is to create an exploration and mapping robot. This system is designed to meet the needs of two main types of users. First, it can be used by the military as a guide for reconnaissance missions and topographic surveys on rough terrains. Second, it is highly useful for architects and the construction industry to map the inside of buildings and to help organize spaces. The main problem this robot solves is the ability to easily access and map areas that are usually hard to reach.

The user will pilot the robot with a remote and will not have to worry about keeping it balanced. The interaction with the system works through two different interfaces. The pilot uses a wireless remote control to manage the movements of the robot, make it jump over obstacles, and check essential data. At the same time, a computer interface will receive the data wirelessly. This computer software will process the images and display the detailed maps created by the onboard camera and the Lidar sensor.

To make all of this possible, the robot must operate under specific technical constraints. The robot moves on only two wheels which makes it more maneuverable in small and tight places. To stay perfectly stable on its two contact points, it uses a gyroscope and carefully manages its own weight. The robot is also designed to drive on different types of outdoor surfaces like grass and dirt. The system must be compact with a height of about thirty centimeters and it needs to have a battery life of at least forty minutes to complete its missions.




In this project, the product is a two-wheeled robot that can either operate autonomously or be remotely controlled. The robot is configured and monitored through a Human-Machine Interface (HMI) that we designed and developed ourselves.

The main objective of this project is to enable a user to explore an unknown environment using the robot and to create a map of this environment through our HMI. The robot is equipped with several sensors, including a LiDAR and a camera, which allow it to collect information about its surroundings. This data is then transmitted to the HMI, where it can be processed and displayed to the user.

The HMI provides the user with an overview of the robot's environment and allows them to monitor important information, control the robot remotely, and visualize the generated map. The project therefore combines robotics, autonomous navigation,  data processing, image processing, and human-machine interaction.

*fig 1 : Functional Analysis Diagram (Horned Beast Diagram)*

*fig 2 : Functional Interaction Diagram (Octopus Diagram)*

| Reference  | Constraint function | Critical | Level |  |
| ----- | ----- | ----- | ----- | ----- |
| CF1 | Maintain balance during movement | High | ≥ 10 min |  |
| CF2 | Map its environment | Medium | ≥ 10 min |  |
| CF3 | Be remotely controlled | High | ≥ 5 m |  |

