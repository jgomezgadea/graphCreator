# fms_routes

Component that keeps a graph of the positions where the robot can move and the magnets that interconnect them.

## Params

* graph_file (string)
  * path of the xml file with the graph
 
## Topic
### Publishers
* /fms_routes_node/state (fms_routes_node/State)
  * publishes the state of the component

## Services
* /fms_routes_node/get_route (fms_routes_node/GetRoute)
  * finds the shortest path between two nodes and returns the list of nodes & magnets that are in the route
* /fms_routes_node/reload_graph (fms_routes_node/ReloadGraph)
  * reloads the file with the routes graph
* /fms_routes_node/get_node_info (fms_routes_node/GetNodeInfo)
  * gets the node detailed information
 
 
## Bringup

```
$> roslaunch robotnik_fms_bringup fms_routes.launch
```

## Examples 

### Requesting the route from node 1 to node 3:
```
$> rosservice call /fms_routes_node/get_route "from_node: 1 to_node: 3"
```
Returns:

```
nodes: [1, 2, 3]
node_positions: 
  - 
    x: 0.0
    y: 0.0
    theta: 0.0
  - 
    x: 4.2
    y: 0.0
    theta: 0.0
  - 
    x: 4.2
    y: 3.2
    theta: 0.0
velocities: [0.3, 0.3]
magnets: [1, 2, 3, 4]
magnet_positions: 
  - 
    x: 0.9922
    y: 0.0001
    theta: 0.0
  - 
    x: 1.7068
    y: 0.0001
    theta: 0.0
  - 
    x: 4.2001
    y: 1.5104
    theta: 0.0
  - 
    x: 4.2001
    y: 2.2083
    theta: 0.0
```

### Getting node info:
```
$> rosservice call /fms_routes_node/get_node_info "node_id: 1" 
node_info: 
  id: 1
  position: 
    position: 
      x: -0.1
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  laser_range: 1
  acoustic_signal: True
  recalibration: True
ret: True
msg: Node found
```

### Reloading the graph:
Case 1. No param is provided. It will reload the current xml file loaded when startup.
```
$> rosservice call /fms_routes_node/reload_graph "file: ''" 
ret: True
```
Case 2. Absolute path is provided. It will load the new xml graph in memory.
```
$> rosservice call /fms_routes_node/reload_graph "file: '/home/user/Desktop/new_graph.xml'" 
> ret: True
```
