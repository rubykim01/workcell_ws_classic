<!--
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
-->

<div id="top"></div>

<p>
  <strong>This implementation is based on the original work:</strong>
  <br />
  <strong>IFRA-Cranfield (2023) Gazebo-ROS2 Link Attacher.</strong>
  URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
  <br />
  <br />
  <strong>Modifications made:</strong>
  <br />
  - Enhanced multi-link attachment capabilities for complex assembly operations
  <br />
  - Improved physics stabilization to prevent object floating in Gazebo
</p>

<br />

## USAGE

The plugin offers the link attachment/detachment feature though 2 different ROS2 Services, which can be called by executing the following commands:

```sh
# To ATTACH an OBJECT to a ROBOT's END-EFFECTOR:
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'model1', link1_name: 'link1', model2_name: 'model2', link2_name: 'link2'}"
# To DETACH the OBJECT from the END-EFFECTOR:
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{model1_name: 'model1', link1_name: 'link1', model2_name: 'model2', link2_name: 'link2'}"
```

Elements are defined as follows:
* model1 -> Name of the model/robot (defined in the robot .urdf). 
* link1 -> Name of the end-effector link that the object will be attached to.
* model2 -> Name of the object to be attached (defined in the object .urdf). 
* link2 -> Name of the object link.

__MAIN REQUIREMENT to execute the plugin: Gazebo .world file__

In order for the /ATTACHLINK and /DETACHLINK ROS2 services to be available in simulation, the LinkAttacher Plugin must be initialised in Gazebo. This is done by adding the following line to the Gazebo world file:
```
<plugin name="gazebo_link_attacher" filename="libgazebo_link_attacher.so"/>
```





