
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
