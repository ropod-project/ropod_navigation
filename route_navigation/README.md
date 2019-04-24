## ED-plugin for ropod-navigation
For the ED-core packge check <https://github.com/tue-robotics/ed>. The corresponding tutorials can be found at <https://github.com/tue-robotics/ed_tutorials>. 

#### Usage
Ensure that the repository found at <https://github.com/tue-ropod/ropod-project-software> is checked out in your catkin_workspace. At this moment, the Development-branch should suffice. 

Add the plugin in the ED-configuration file. The following lines should be added:
```
  - name: route_navigation
    lib: libed_route_navigation_plugin.so
    frequency: 5
```
    
A full example is shown at <https://github.com/tue-ropod/ropod-project-software>. Check the `model-example-ropod-navigation-ED.yaml`-file in the ropod_navigation_test/config package.

#### Mobidik-docing: prerequisites
  * Create a map and provide the same map in an ed_object_model (as in https://github.com/ropod-project/ed_object_models/tree/ropod-master/models/tue_hallway_RLtoAL) to ED. Ensure that the resolution and initial position are similar for both maps.
  * Ensure that the laser-tracking, as shown in https://github.com/ropod-project/ed_sensor_integration (currently, use the feature/tracking-branch) runs
  * Define areas where to collect and find the mobidik in the ed_object_models. This areas are used later to indicate where a mobidik can be found. Examples are shown in https://github.com/ropod-project/ed_object_models/tree/ropod-master/models/tue_hallway_RLtoAL (look in the model.yaml). First, define the position according to the follwing example:
   <pre>
    - type: "indoor_navigation_areas/MobidikArea"
      id: "MobidikArea1"
      pose: { x: 2.43, y: 3.01, z: 0, rz: 0 }
      shape:
          compound:
            - polygon:
                  points:
                    - { x: 0, y: 0 }
                    - { x: 2.26, y: -0.72 }
                    - { x: 2.90, y: 1.84 }
                    - { x: 0.52, y: 2.30 }
                  height: 0.1
  </pre>
Here, the pose is in global coordinates (rz is the yaw in radians), while the polygonal points are relative to the first coordinate, in **counter-clockwise**-order. The _id_ must be unique for each area.
  * Related to each area, a waypoint should be added. This waypoint is used for 2 purposes, namely (1) the orientation of the mobidik during the docking phase and (2) the point to place the center of the mobidik back during the undocking phase.
  * The first part must consist of "orient_wp_", followed by the name of the related mobidik-area.
  <pre>
    - type: "waypoint"
      id: "orient_wp_MobidikArea1"
      pose: { x: 3.1, y: 3.9, z: 0, Z: -0.3 }
</pre>
* Eamples of the docking and undocking command can be found in yaml_cmds/collect_mobidik.yaml. These commands are published on the "/ropod_task_executor/GOTO"-topic having a "ropod_ros_msgs/Action"-message. Change the type (DOCK or UNDOCK) and change the id to the area where the mobidik is placed in, while the id must set equal to the name of the waypoint.
  

#### Launch
Ensure that ED is launched in your launch-file with a reference to the configuration file mentioned above. An example looks like this:
```
        <rosparam command="load" ns="areas" file="$(find route_navigation)/config/navigation_goals.yaml"/>
        <rosparam command="load" ns="area_types" file="$(find route_navigation)/config/navigation_goal_types.yaml"/>

        <node pkg="ed" type="ed" name="ed" args="$(find ropod_navigation_test)/config/model-example-ropod-navigation-ED.yaml">
                 <param name="move_base_server" value="move_base" />
                 <param name="move_base_feedback_topic" value="/move_base/feedback" />
                 <param name="move_base_cancel_topic" value="/move_base/cancel" />
                 <remap from="~goto_action" to="/ropod_task_executor/GOTO" />
        </node>-->
```
A full example is shown at <https://github.com/tue-ropod/ropod-project-software>. Check the `ROPOD_navigation_ED_example.launch`-file in the ropod_navigation_test/launch package.

#### Subscribed to
`planned_route`: not used?

`waypoint_cmd`: semantic waypoint command

`door`: door status
