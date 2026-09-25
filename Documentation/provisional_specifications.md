

| Ref. | Function | Criterion | Expected level | Tolerance / constraint |
| ----- | ----- | ----- | ----- | ----- |
| **FP1** | **Map the environment** | Mapping accuracy | **5 cm** | ± 5 cm |
|  |  | LiDAR range | **5 m** | ± 2 m |
| **FP2** | **Move through the environment** | Wheel slippage | **All-terrain** | . |
|  |  | Tipping / falling | **None** | . |
|  |  | Maximum speed | **4 km/h** | ± 2 km/h |
| **FP3** | **Display information on the HMI** | Display latency | **\< 100 ms** | . |
|  |  | Map refresh rate | **2 Hz** | ± 1 Hz |
| **FP4** | **Enable remote control** | Indoor range | **\> 30 m** | . |
|  |  | Outdoor range | **\> 150 m** | . |
|  |  | Command response time | **\< 50 ms** | . |
| **FC1** | **Maintain balance on two wheels** | Maximum climbable slope | **\> 30°** | . |
| **FS1** | **Manage the robot's weight** | Center of gravity position | **Centered above the wheel axis** | . |
|  |  | Maximum total mass | **\< 1 kg** | . |
| **FS2** | **Overcome obstacles by jumping** | Maximum obstacle height | **20 cm** | ± 5 cm |
|  |  | Landing success rate | **80%** | ± 10% |
| **FS3** | **Ensure wireless communication** | Robot → remote control data rate | **\> 30 kbit/s** | . |
|  |  | Robot → HMI data rate | **\> 2 Mbit/s** | . |
|  |  | Communication frequency | **2.4 GHz Wi-Fi** | . |

| Ref. | Constraint | Requirement |
| ----- | ----- | ----- |
| **C1** | **Battery life** | **≥ 40 min** |
| **C2** | **Dimensions** | Height of approximately **30 cm** |
| **C3** | **Mobility** | Operation on grass, soil, and various outdoor terrains |
| **C4** | **Electronic architecture** | Two separate boards: one for balancing/movement and one for mapping data processing |
| **C5** | **Data processing** | One board must provide sufficient computing power for LiDAR and camera processing |
| **C6** | **Communication** | Wireless transmission to the remote control and HMI |
| **C7** | **HMI** | Display of robot data and map visualization |
| **C8** | **Safety** | Robot shutdown in case of connection loss |
| **C9** | **Sensors** | Use of a LiDAR and a camera for exploration and mapping |
| **C10** | **Robustness** | Resistance to stresses associated with movement and different terrains |

