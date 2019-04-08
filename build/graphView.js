/**
 * @author Jose Gómez - jgomez@robotnik.es
 */

var GRAPHVIEW = GRAPHVIEW || {
  REVISION: '0.1.0'
};

GRAPHVIEW.GraphView = function (options) {
  options = options || {}
  var divID = options.divID || 'graph';
  var width = options.width || 600;
  var height = options.height || 400;
  var nodes = options.nodes || new vis.DataSet([
    // Example
    { id: 1, label: 'Node 1' },
    { id: 2, label: 'Node 2' },
    { id: 3, label: 'Node 3' },
    { id: 4, label: 'Node 4' },
    { id: 5, label: 'Node 5' },
    { id: 6, label: 'Node 6' },
    { id: 7, label: 'Node 7' },
    { id: 8, label: 'Node 8' }
  ]);
  var edges = options.edges || new vis.DataSet([
    // Example
    { from: 1, to: 8, arrows: 'to', dashes: true },
    { from: 1, to: 3, arrows: 'to' },
    { from: 1, to: 2, arrows: 'to, from' },
    { from: 2, to: 4, arrows: 'to, middle' },
    { from: 2, to: 5, arrows: 'to, middle, from' },
    { from: 5, to: 6, arrows: { to: { scaleFactor: 2 } } },
    { from: 6, to: 7, arrows: { middle: { scaleFactor: 0.5 }, from: true } }
  ]);

  // Create a graph
  var container = document.getElementById(divID);
  var data = {
    nodes: nodes,
    edges: edges
  };
  var options = {}; // TODO add options
  var graph = new vis.Network(container, data, options);

  return graph;
}
