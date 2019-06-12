/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var MAPVIEW = MAPVIEW || {
  REVISION: '0.1.0'
};

MAPVIEW.MapView = function (options) {
  options = options || {}
  var ros = options.ros;
  var topic = options.topic || '/map'
  var divID = options.divID || 'map';
  var width = options.width || 308;
  var height = options.height || 250;

  var self = this;

  // Create the main viewer
  this.viewer = new ROS2D.Viewer({
    divID: divID,
    width: width,
    height: height
  });

  var panView = new ROS2D.PanView({
    rootObject: viewer.scene
  })

  var zoomView = new ROS2D.ZoomView({
    rootObject: viewer.scene
  })

  // Setup the map client.
  this.gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    topic: topic,
    rootObject: self.viewer.scene,
    // Use this property in case of continuous updates			
    continuous: true
  });
  // Scale the canvas to fit to the map
  this.gridClient.on('change', function () {
    self.viewer.scaleToDimensions(self.gridClient.currentGrid.width, self.gridClient.currentGrid.height);
    self.viewer.shift(self.gridClient.currentGrid.pose.position.x, self.gridClient.currentGrid.pose.position.y);
  });

  // Callback functions when there is mouse interaction with a point
  var clickedPoint = false;
  var selectedPointIndex = null;
  var movingMap = false;       // We are moving the map
  var movedMouse = false;      // Moved mouse since last click
  var mapPos = { x: 0, y: 0 }; // map position relative to canvas
  var posRos; // Mouse position relative to ros
  var pos;    // Mouse position on canvas

  var pointCallBack = function (type, event, index) {
    if (type === 'mousedown') {
      if (event.nativeEvent.shiftKey === true) {
        graph.remPoint(index);
      } else {
        selectedPointIndex = index;
      }
    }
    clickedPoint = true;
  };

  // Create the graph
  var graph = new ROS2D.Graph({
    pointCallBack: pointCallBack,
  });

  // Add the graph to the viewer
  this.viewer.scene.addChild(graph);

  this.viewer.scene.addEventListener('stagemousemove', function (event) {
    movedMouse = true;
    pos = { x: event.stageX, y: event.stageY };
    posRos = self.viewer.scene.globalToRos(pos.x, pos.y);
    // Move point when it's dragged and ctrl pressed
    if (selectedPointIndex !== null && event.nativeEvent.ctrlKey === true) {
      graph.movePoint(selectedPointIndex, posRos);
    }
    // Move map when it's dragged
    else if (movingMap === true) {
      panView.pan(pos.x - mapPos.x, pos.y - mapPos.y);
    }
  });

  this.viewer.scene.addEventListener('stagemouseup', function (event) {
    // Stop moving point when mouse up
    selectedPointIndex = null;
    // If we were moving the map, save new map position
    if (movingMap === true) {
      mapPos.x = pos.x - mapPos.x;
      mapPos.y = pos.y - mapPos.y;
      movingMap = false;
    }
    // Add point when not clicked on the graph ...
    else if (self.viewer.scene.mouseInBounds === true && clickedPoint === false) {
      // ... only if we aren't doing any other action
      if (event.nativeEvent.shiftKey === false && movedMouse === false) {
        graph.addPoint(posRos);
      }
    }
    clickedPoint = false;
  });

  this.viewer.scene.addEventListener('stagemousedown', function (event) {
    movedMouse = false;
    // We can move the map with drag and drop
    if (event.nativeEvent.ctrlKey !== true && self.viewer.scene.mouseInBounds === true) {
      mapPos.x = pos.x - mapPos.x;
      mapPos.y = pos.y - mapPos.y;
      movingMap = true;
    }
  })

  document.getElementById('map').addEventListener('wheel', function (event) {
    if (self.viewer.scene.mouseInBounds === true) {
      // Start zoom from mouse position
      zoomView.startZoom(pos.x, pos.y);
      // Add zoom
      if (event.deltaY < 0) {
        // Zoom in
        zoomView.zoom(1.05, 1.05);
      } else {
        // Zoom out
        zoomView.zoom(0.95, 0.95);
      }
      event.preventDefault();
    }
  })

  //return this.viewer;

}
