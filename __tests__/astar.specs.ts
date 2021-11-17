/**
 * Copyright 2021 Jim Armstrong (www.algorithmist.net)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * A* for Waypoints specs
 */

/** Copyright 2016 Jim Armstrong (www.algorithmist.net)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import { AStarGraph    } from "../src/astar-graph";
import { AStarGraphArc } from "../src/astar-graph-arc";
import { AStarWaypoint } from "../src/astar-waypoint";
import { AstarMinHeap  } from "../src/astar-min-heap";
import { AStar         } from "../src/astar";

import { graph1 } from "../src/data/graph1";

import { createWaypoint } from "../src/astar-waypoint";

function graphNodeFactory(id: string, h: number, x: number = 0, y: number = 0): AStarWaypoint
{
  const w: AStarWaypoint = new AStarWaypoint(id);
  w.heuristic            = h;
  w.setCoords(x, y);

  return w;
};

function waypointFactory(id: string, h: number): AStarWaypoint
{
  const w: AStarWaypoint = new AStarWaypoint(id);
  w.heuristic            = h;

  return w;
};

// Test Suites
describe('A* Graph Tests', () => {

  it('correctly constructs new A* Graph', function() {
    const graph: AStarGraph = new AStarGraph();

    expect(graph.size).toBe(0);
    expect(graph.edgeCount).toBe(0);
    expect(graph.isCartesian).toBe(true);
  });

  it('does not add a null node', function() {
    const graph: AStarGraph = new AStarGraph();

    let temp: any = null;

    graph.addNode(temp);
    expect(graph.size).toBe(0);
    expect(graph.edgeCount).toBe(0);
    expect(graph.isCartesian).toBe(true);
  });

  it('adds a singleton node', function() {
    const graph: AStarGraph = new AStarGraph();

    const g: AStarWaypoint = graphNodeFactory('1', 1, 1, 2);

    graph.addNode(g);

    expect(graph.size).toBe(1);
    expect(graph.edgeCount).toBe(0);
    expect(graph.isCartesian).toBe(true);
  });

  it('adds multiple nodes', function() {
    const graph: AStarGraph = new AStarGraph();

    const g: AStarWaypoint = graphNodeFactory('1', 1, 1, 2);
    graph.addNode(g);

    const g2: AStarWaypoint = graphNodeFactory('2', 2, 3, 4);
    graph.addNode(g2);

    const g3: AStarWaypoint = graphNodeFactory('3', 3, 4, 5);
    graph.addNode(g3);

    expect(graph.size).toBe(3);
    expect(graph.edgeCount).toBe(0);
    expect(graph.isCartesian).toBe(true);
  });

  it('adds multiple nodes and edges', function() {
    const graph: AStarGraph = new AStarGraph();

    const g: AStarWaypoint = graphNodeFactory('1', 1, 1, 2);
    graph.addNode(g);

    const g2: AStarWaypoint = graphNodeFactory('2', 2, 3, 4);
    graph.addNode(g2);

    const g3: AStarWaypoint = graphNodeFactory('3', 3, 4, 5);
    graph.addNode(g3);

    graph.addEdge({
      key: 'e1',
      v1: g,
      v2: g2,
      w: 1.7
    });

    graph.addEdge({
      key: 'e2',
      v1: g2,
      v2: g3,
      w: 2.4
    });

    expect(graph.size).toBe(3);
    expect(graph.edgeCount).toBe(2);
    expect(graph.isCartesian).toBe(true);
  });

  it('creates a graph from a data provider', function() {
    const graph: AStarGraph = new AStarGraph();

    graph.fromObject(graph1);

    expect(graph.size).toBe(8);
    expect(graph.edgeCount).toBe(11);
    expect(graph.isCartesian).toBe(true);

    const node: AStarWaypoint = graph.nodeList;
    expect(node.key).toBe('8');
  });

  it('properly modifies edge cost', function() {
    const graph: AStarGraph = new AStarGraph();

    graph.fromObject(graph1);

    expect(graph.size).toBe(8);
    expect(graph.edgeCount).toBe(11);
    expect(graph.isCartesian).toBe(true);

    graph.updateEdgeCost('1', 3.45);

    const edge: AStarGraphArc = graph.getEdge('1') as AStarGraphArc;

    expect(typeof edge?.cost).toBe('number');
    expect(Number(edge?.cost)).toBe(3.45);
  });

  it('fetch node by waypoint works on null input', function() {
    const graph: AStarGraph = new AStarGraph();

    let temp: any = null;

    expect(graph.getNode(temp)).toBe(null);
  });

  it('fetch waypoint by key returns null on incorrect input', function() {
    const graph: AStarGraph = new AStarGraph();

    const g: AStarWaypoint = graphNodeFactory('1', 1, 1, 2);
    graph.addNode(g);

    const g2: AStarWaypoint = graphNodeFactory('2', 2, 3, 4);
    graph.addNode(g2);

    const g3: AStarWaypoint = graphNodeFactory('3', 3, 4, 5);
    graph.addNode(g3);

    expect(graph.getNode('4')).toBe(null);
  });

  it('fetch waypoint by key', function() {
    const graph: AStarGraph = new AStarGraph();

    const g: AStarWaypoint = graphNodeFactory('1', 1, 1, 2);
    graph.addNode(g);

    const g2: AStarWaypoint = graphNodeFactory('2', 2, 3, 4);
    graph.addNode(g2);

    const g3: AStarWaypoint = graphNodeFactory('3', 3, 4, 5);
    graph.addNode(g3);

    const result: AStarWaypoint = graph.getNode('1') as AStarWaypoint;

    expect(result.equals(g)).toBe(true);
  });
});

describe('A* Min-Heap Tests', () => {

  it('newly constructed Heap has size of zero', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    expect(heap.size).toBe(0);
  });

  it('properly assigns a singleton element', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w: AStarWaypoint = new AStarWaypoint('1');
    w.heuristic            = 1.0;

    heap.insert(w);

    const element: number = heap.peek();

    expect(heap.size).toBe(1);
    expect(element).toBe(1.0);
  });

  it('2-element insert test #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w1: AStarWaypoint = new AStarWaypoint('1');
    w1.heuristic            = 1.0;

    const w2: AStarWaypoint = new AStarWaypoint('2');
    w2.heuristic            = 2.0;

    heap.insert(w1);
    heap.insert(w2);

    const element: number = heap.peek();

    expect(heap.size).toBe(2);
    expect(element).toBe(1.0);
  });

  it('2-element insert test #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w1: AStarWaypoint = new AStarWaypoint('1');
    w1.heuristic            = 2.0;

    const w2: AStarWaypoint = new AStarWaypoint('2');
    w2.heuristic            = 1.0;

    heap.insert(w1);
    heap.insert(w2);

    const element: number = heap.peek();

    expect(heap.size).toBe(2);
    expect(element).toBe(1.0);
  });

  it('3-element insert test #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w1: AStarWaypoint = new AStarWaypoint('1');
    w1.heuristic            = 1.0;

    const w2: AStarWaypoint = new AStarWaypoint('2');
    w2.heuristic            = 2.0;

    const w3: AStarWaypoint = new AStarWaypoint('3');
    w3.heuristic            = 3.0;

    heap.insert(w1);
    heap.insert(w2);
    heap.insert(w3);

    const element: number = heap.peek();

    expect(heap.size).toBe(3);
    expect(element).toBe(1.0);
  });

  it('3-element insert test #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w1: AStarWaypoint = new AStarWaypoint('3');
    w1.heuristic            = 3.0;

    const w2: AStarWaypoint = new AStarWaypoint('2');
    w2.heuristic            = 2.0;

    const w3: AStarWaypoint = new AStarWaypoint('1');
    w3.heuristic            = 1.0;

    heap.insert(w1);
    heap.insert(w2);
    heap.insert(w3);

    const element: number = heap.peek();

    expect(heap.size).toBe(3);
    expect(element).toBe(1.0);
  });

  it('3-element insert test #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    const w1: AStarWaypoint = new AStarWaypoint('2');
    w1.heuristic            = 2.0;

    const w2: AStarWaypoint = new AStarWaypoint('1');
    w2.heuristic            = 1.0;

    const w3: AStarWaypoint = new AStarWaypoint('3');
    w3.heuristic            = 3.0;

    heap.insert(w1);
    heap.insert(w2);
    heap.insert(w3);

    const element: number = heap.peek();

    expect(heap.size).toBe(3);
    expect(element).toBe(1.0);
  });

  it('multi-element insert test #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('8', 8.0));

    const element: number = heap.peek();

    expect(heap.size).toBe(6);
    expect(element).toBe(1.0);
  });

  it('toArray returns copy of heap', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('8', 8.0));

    const curHeap: Array<AStarWaypoint> = heap.toArray();

    expect(curHeap.length).toBe(6);
    expect(curHeap[0].heuristic).toBe(1.0);
  });

  it('multi-element insert test #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('7', 7.0));

    const element: number = heap.peek();

    expect(heap.size).toBe(7);
    expect(element).toBe(1.0);

    const curHeap: Array<AStarWaypoint> = heap.toArray();
    expect(curHeap[heap.size-1].heuristic).toBe(7);
  });

  it('multi-element insert test #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('10', 10.0));
    heap.insert(waypointFactory('14', 14.0));
    heap.insert(waypointFactory('6', 6.0));

    const element: number = heap.peek();

    expect(heap.size).toBe(9);
    expect(element).toBe(1.0);
  });

  it('multi-element insert test #4', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('7', 7.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('9', 9.0));

    const element: number = heap.peek();

    expect(heap.size).toBe(9);
    expect(element).toBe(1.0);
  });

  it('multi-element insert test #5', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('7', 7.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('1', 1.0));

    const element: number = heap.peek();

    expect(heap.size).toBe(9);
    expect(element).toBe(1.0);
  });

  it('levels accessor test #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    expect(heap.levels).toBe(0);
  });

  it('levels accessor test #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));

    expect(heap.levels).toBe(1);
  });

  it('levels accessor test #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));

    expect(heap.levels).toBe(2);
  });

  it('levels accessor test #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 1.0));
    heap.insert(waypointFactory('3', 3.0));

    expect(heap.levels).toBe(2);
  });

  it('levels accessor test #4', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('4', 4.0));

    expect(heap.levels).toBe(3);
  });

  it('levels accessor test #5', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('5', 5.0));

    expect(heap.levels).toBe(3);
  });

  it('levels accessor test #6', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('6', 6.0));

    expect(heap.levels).toBe(3);
  });

  it('levels accessor test #8', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('4', 4.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('7', 7.0));
    heap.insert(waypointFactory('8', 8.0));

    expect(heap.levels).toBe(4);
  });

  it('properly removes min element from heap #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('8', 8.0));

    let element: number = heap.peek();
    expect(element).toBe(1.0);

    const min: AStarWaypoint = heap.extractRoot() as AStarWaypoint;
    expect(min.key).toBe('1');
    element = heap.peek();

    expect(heap.size).toBe(5);
    expect(element).toBe(3);
  });

  it('properly removes min element from heap #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('3', 3.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('8', 8.0));

    let element: number = heap.peek();
    expect(element).toBe(1.0);

    let min: AStarWaypoint = heap.extractRoot() as AStarWaypoint;
    element = heap.peek();

    expect(heap.size).toBe(5);
    expect(element).toBe(3);

    min     = heap.extractRoot() as AStarWaypoint;
    element = heap.peek();

    expect(heap.size).toBe(4);
    expect(element).toBe(5);
  });

  it('properly removes min element from heap #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('9', 9.0));

    let min: AStarWaypoint   = heap.extractRoot() as AStarWaypoint;
    let element: number = heap.peek();

    expect(heap.size).toBe(3);
    expect(element).toBe(6);

    min     = heap.extractRoot() as AStarWaypoint;
    element = heap.peek();

    expect(heap.size).toBe(2);
    expect(element).toBe(8);

    min     = heap.extractRoot() as AStarWaypoint;
    element = heap.peek();

    expect(heap.size).toBe(1);
    expect(element).toBe(9);

    min     = heap.extractRoot() as AStarWaypoint;
    element = heap.peek();

    expect(heap.size).toBe(0);
    expect(element).toBe(0);
  });

  it('properly extracts and inserts', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('9', 9.0));

    const min: AStarWaypoint   = heap.extractRoot() as AStarWaypoint;
    const element: number = heap.peek();

    expect(heap.size).toBe(3);
    expect(element).toBe(6);

    heap.insert(waypointFactory('1', 1.0));

    expect(heap.size).toBe(4);
    expect(heap.peek()).toBe(1);
  });

  it('does nothing on deleting from an empty heap', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.delete(1.0);

    expect(heap.size).toBe(0);
  });

  it('properly deletes element from a singleton heap', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));

    heap.delete(5.0);
    expect(heap.size).toBe(1);

    heap.delete(1.0);
    expect(heap.size).toBe(0);
  });

  it('arbitrary element delete test #1', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));

    heap.delete(1.0);

    expect(heap.size).toBe(1);
    expect(heap.peek()).toBe(2);
  });

  it('arbitrary element delete test #2', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));

    heap.delete(2.0);

    expect(heap.size).toBe(1);
    expect(heap.peek()).toBe(1);
  });

  it('arbitrary element delete test #3', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));

    heap.delete(1.0);

    expect(heap.size).toBe(2);
    expect(heap.peek()).toBe(2);
  });

  it('arbitrary element delete test #4', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));

    heap.delete(2.0);

    expect(heap.size).toBe(2);
    expect(heap.peek()).toBe(1);
  });

  it('arbitrary element delete test #5', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('2', 2.0));
    heap.insert(waypointFactory('3', 3.0));

    heap.delete(3.0);

    expect(heap.size).toBe(2);
    expect(heap.peek()).toBe(1);
  });

  it('arbitrary element delete test #6', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('11', 11.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('15', 15.0));
    heap.insert(waypointFactory('17', 17.0));
    heap.insert(waypointFactory('21', 21.0));

    heap.delete(5.0);

    expect(heap.size).toBe(8);
    expect(heap.peek()).toBe(1);

    const arr: Array<AStarWaypoint> = heap.toArray();
    expect(arr[1].heuristic).toBe(9);
    expect(arr[2].heuristic).toBe(6);
    expect(arr[3].heuristic).toBe(17);
    expect(arr[4].heuristic).toBe(11);
    expect(arr[5].heuristic).toBe(8);
    expect(arr[6].heuristic).toBe(15);
    expect(arr[7].heuristic).toBe(21);
  });

  it('arbitrary element delete test #7', function() {
    const heap: AstarMinHeap = new AstarMinHeap();

    heap.insert(waypointFactory('1', 1.0));
    heap.insert(waypointFactory('5', 5.0));
    heap.insert(waypointFactory('6', 6.0));
    heap.insert(waypointFactory('9', 9.0));
    heap.insert(waypointFactory('11', 11.0));
    heap.insert(waypointFactory('8', 8.0));
    heap.insert(waypointFactory('15', 15.0));
    heap.insert(waypointFactory('17', 17.0));
    heap.insert(waypointFactory('21', 21.0));

    heap.delete(8.0);

    expect(heap.size).toBe(8);
    expect(heap.peek()).toBe(1);

    const arr: Array<AStarWaypoint> = heap.toArray();
    expect(arr[1].heuristic).toBe(5);
    expect(arr[2].heuristic).toBe(6);
    expect(arr[3].heuristic).toBe(9);
    expect(arr[4].heuristic).toBe(11);
    expect(arr[5].heuristic).toBe(21);
    expect(arr[6].heuristic).toBe(15);
    expect(arr[7].heuristic).toBe(17);
  });
});

describe('A* Waypoint Tests', () => {

  it('correctly constructs new A* Waypoint', function() {
    const waypoint: AStarWaypoint = new AStarWaypoint('1');

    expect(waypoint.key).toBe('1');
    expect(waypoint.x).toBe(0);
    expect(waypoint.y).toBe(0);
    expect(waypoint.latitude).toBe(0);
    expect(waypoint.longitude).toBe(0);
    expect(waypoint.isCartesian).toBe(true);
  });

  it('correctly sets cartesian coordinates', function() {
    const waypoint: AStarWaypoint = new AStarWaypoint('1');

    expect(waypoint.key).toBe('1');
    expect(waypoint.x).toBe(0);
    expect(waypoint.y).toBe(0);

    waypoint.setCoords(7, 2);
    expect(waypoint.isCartesian).toBe(true);
    expect(waypoint.x).toBe(7);
    expect(waypoint.y).toBe(2);
  });

  it('correctly sets geo coordinates', function() {
    const waypoint: AStarWaypoint = new AStarWaypoint('1');

    expect(waypoint.key).toBe('1');
    expect(waypoint.longitude).toBe(0);
    expect(waypoint.latitude).toBe(0);

    waypoint.setGeoCoords(-97.3456, 32.3307);
    expect(waypoint.isCartesian).toBe(false);
    expect(waypoint.longitude).toBe(-97.3456);
    expect(waypoint.latitude).toBe(32.3307);
  });

  it('computes correct euclidean distance', function() {
    const w1: AStarWaypoint = new AStarWaypoint('1');
    const w2: AStarWaypoint = new AStarWaypoint('2');

    expect(w1.key).toBe('1');
    w1.setCoords(2, 4);

    expect(w2.key).toBe('2');
    w2.setCoords(-3, 8);

    const d: number = w1.distanceTo(w2);
    expect(Math.abs(d - 6.403)).toBeLessThan(0.001);
  });
});

describe('A* Pathfinding Tests', () => {

  it('correctly constructs new A* pathfinder', function() {
    const astar: AStar = new AStar();

    expect(astar).toBeTruthy();
  });

  it('path is empty for invalid arguments', function() {
    const astar: AStar      = new AStar();
    const graph: AStarGraph = new AStarGraph();

    let temp1: any = null;
    let temp2: any = null;
    let temp3: any = null;

    let waypoints: Array<AStarWaypoint> = astar.find(temp1, temp2, temp3);
    expect(waypoints.length).toBe(0);

    waypoints = astar.find(graph, temp1, temp2);
    expect(waypoints.length).toBe(0);

    const w: AStarWaypoint = createWaypoint('1', 1, 4, true, 3);
    waypoints              = astar.find(graph, w, temp1);
    expect(waypoints.length).toBe(0);

    waypoints = astar.find(graph, temp1, w);
    expect(waypoints.length).toBe(0);
  });

  it('path is empty for empty graph', function() {
    const astar: AStar      = new AStar();
    const graph: AStarGraph = new AStarGraph();

    const w: AStarWaypoint                = createWaypoint('1', 1, 4, true, 3);
    const waypoints: Array<AStarWaypoint> = astar.find(graph, w, w);

    expect(waypoints.length).toBe(0);
  });

  it('path is correct for singleton graph', function() {
    const astar: AStar      = new AStar();
    const graph: AStarGraph = new AStarGraph();

    const w: AStarWaypoint = createWaypoint('1', 1, 4, true, 3);
    graph.addNode(w);

    const waypoints: Array<AStarWaypoint> = astar.find(graph, w, w);
    expect(waypoints.length).toBe(1);

    expect(waypoints[0].equals(w)).toBe(true);
  });

  it('path is correct for two-node graph', function() {
    const astar: AStar      = new AStar();
    const graph: AStarGraph = new AStarGraph();

    const w: AStarWaypoint = createWaypoint('1', 1, 4, true, 3);
    graph.addNode(w);

    const w2: AStarWaypoint = createWaypoint('2', 2, 6, true, 4);
    graph.addNode(w2);

    const waypoints: Array<AStarWaypoint> = astar.find(graph, w, w);
    expect(waypoints.length).toBe(2);

    expect(waypoints[0].equals(w)).toBe(true);
    expect(waypoints[1].equals(w2)).toBe(true);
  });

  it('general graph test #1', function() {
    const astar: AStar      = new AStar();
    const graph: AStarGraph = new AStarGraph();

    graph.fromObject(graph1);

    const waypoints: Array<AStarWaypoint> = astar.find(graph, '1', '7');

    expect(waypoints.length).toBe(4);

    expect(waypoints[0].key).toBe('1');
    expect(waypoints[1].key).toBe('2');
    expect(waypoints[2].key).toBe('4');
    expect(waypoints[3].key).toBe('7');

    expect(Math.abs(waypoints[3].distance - 9.576491222541476) < 0.001).toBe(true);
  });
});

