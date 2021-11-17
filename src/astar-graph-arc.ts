/**
 * Copyright 2020 Jim Armstrong (www.algorithmist.net)
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
 *
 * Customized graph arc for A* waypoints
 *
 * @author Jim Armstrong, www.algorithmist.net
 *
 * @version 1.0
 *
 */

import { CostFunction  } from './astar-graph';
import { AStarWaypoint } from './astar-waypoint';

export class AStarGraphArc
{
  public prev: AStarGraphArc | null;         // previous arc in linkage
  public next: AStarGraphArc | null;         // next arc in linkage
  public node: AStarWaypoint | null;         // terminal node in arc

  protected _cost: number;
  protected _costFcn: CostFunction | null;

  constructor(node: AStarWaypoint, cost: CostFunction | number)
  {
    this.node     = node;
    this._cost    = typeof cost === 'number' ? Number(cost) : -1;
    this._costFcn = typeof cost === 'function' ? cost : null;

    this.next = null;
    this.prev = null;
  }

  /**
   * Access the arc's numerical cost or cost function
   */
  public get cost(): CostFunction | number
  {
    return this._cost != -1 ? this._cost as number : this._costFcn as CostFunction;
  }

  /**
   * Assign the arc's numerical cost or cost function
   *
   * @param value Reference to cost function or numerical cost for arc (numerical value must be greater than or equal to zero)
   */
  public set cost(value: CostFunction | number)
  {
    this._cost    = typeof value === 'number' ? Number(value) : -1;
    this._costFcn = typeof value === 'function' ? value : null;
  }

  /**
   * Clear the arc or edge data
   */
  public clear()
  {
    this.node = null;
    this.next = null;
    this.prev = null;
  }

  /**
   * Access the end waypoint associated with this arc
   */
  public get waypoint(): AStarWaypoint | null
  {
    return this.node;
  }
}
