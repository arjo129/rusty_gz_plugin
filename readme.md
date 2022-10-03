# Gazebo Plugin Written in Rust: Crowd sim integration

![](doc/rustling_march.gif)
This repo contains my experiments with rust + gazebo.

You will need Gazebo Garden installed. To build this repo simply run:
```
colcon build
```

`corrosion-rs` and `cargo` will take care of the rest of the magic.

## C-API documentation
Complete documentation for the C-API  can be found [here](rusty_gz_plugin/src/rust_interface.h)

## Plugin documentations

The plugin can be loaded like so:
```
 <plugin
      filename="libRustySystem.so"
      name="rusty::RustySystem">
      <!-- TODO: use relative paths -->
    <path>/home/arjo/workspaces/chartsim/src/chart_sim_maps/maps/ward45/ward45.building.yaml</path>
    <sources>
        <source_sink>
            <rate>1</rate>
            <start>-23 -1 0</start>
            <waypoints>
              <waypoint>9 -3 0</waypoint>
            </waypoints>
        </source_sink>
    </sources>
</plugin>
```


### SDFormat Arguments
* `<path>` - Parameter specifies the path to the RMF building YAML.
* `<sources>` - List of crowd sources.
    * `<source_sink>` - An individual crowd source-sink (agents are spawned and follow a fixed path).
        * `<rate>` -Poisson rate at which to generate agents.
        * `<start>` - 3-Vector of start position. (Note: currently z is ignored)
        * `<waypoints>` - List of waypoints agent should go to . Agent is destroyed after reaching last waypoint
            * `<waypoint>` - Waypoint to visit. 3-vector also but z is ignored.
