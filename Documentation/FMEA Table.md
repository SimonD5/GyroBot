**FMEA Table**

FMEA is an analysis table that helps us figure out what could go wrong when our robot is exploring. Since the robot only uses two wheels, keeping its balance and staying connected to the remote are extremely important. This table lists the main problems the system might face. We give each risk a score based on how bad it is (Severity), how often it happens (Frequency) and how hard it is to spot (Detection). This will help us to focus on the biggest dangers so we can figure out the best ways to prevent them.

| Risks | Potential Effects | Probable Causes | G | F | D | C | Preventive Actions |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| *Computer overload or balancing system failure* | The robot stops balancing and crashes, breaking the Lidar and camera. | The robot's microcontroller gets too busy processing maps and images causing a small freeze that stops the balancing system. | 4 | 3 | 4 | 48 | Use two separate microcontrollers: a small one just to keep the robot balanced and another one for the map and camera. |
| *Wheels slipping* | The robot falls right away. If the wheels slide, the robot cannot balance itself and will get damaged. | Driving on mud, wet grass or loose dirt. The two wheels need a good grip on the ground to keep the robot upright. | 4 | 3 | 2 | 24 | Use tires with strong grip. Add physical stands that drop down to catch the robot if it starts to slip. |
| *Falling down stairs or holes* | The robot breaks completely, destroying the expensive sensors. | The Lidar only looks straight ahead and cannot see holes or stairs. The robot drives over the edge. | 4 | 2 | 2 | 16 | Rely on the real-time camera video feed on the HMI so the pilot can visually detect stairs and drop-offs. Mount the camera with a slight angle to maximize ground visibility. |
| *Losing connection to the remote* | The user cannot control the robot anymore, and the computer stops receiving the map. | The robot is too far away or thick walls block the signal. | 3 | 2 | 2 | 12 | Make the robot stop moving automatically if it loses the signal. Show the connection strength on the screen. |
| *Battery dying too fast* | The robot turns off and gets stuck in an area you cannot reach before the 40 minutes are over. | Driving on tough surfaces like thick grass or mud uses too much energy. | 3 | 2 | 2 | 12 | Show the battery level on the remote and make it flash red when it is almost empty. |
| *Map system not working* | The robot cannot create the map or show it on the computer screen. | Something is blocking the Lidar sensor, or the computer software crashes. | 2 | 1 | 2 | 4 | Put a protective cover over the sensors and test the computer software many times. |

**Legend:**
* *G (Severity)*: 1 to 4
* *F (Frequency)*: 1 (Rare) to 4 (Frequent)
* *D (Non-Detection)*: 1 (Immediately detectable) to 4 (Hard to predict)
* *C (Criticality)* = G × F × D
