
## ED-plugin for ropod-navigation

#### Usage
```
Ensure that the repository found at https://github.com/tue-ropod/ropod-project-software is checked out in your catkin_workspace. At this moment, the Development-branch should suffice. 

Add the plugin in the ED-configuration file. The following lines should be added:
  - name: route_navigation
    lib: libed_route_navigation_plugin.so
    frequency: 5
    
A full example is shown at https://github.com/tue-ropod/ropod-project-software. Check the `model-example-ropod-navigation-ED.yaml`-file in the ropod_navigation_test package.
```

#### Subscribed to
`planned_route`: not used?

`waypoint_cmd`: semantic waypoint command

`door`: door status
