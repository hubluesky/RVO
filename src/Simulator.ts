/*
 * Simulator.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

import { Agent } from "./Agent";
import { KdTree } from "./KdTree";
import { Line } from "./Line";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { RVOWorker } from "./RVOWorker";
import { Vector2 } from "./Vector2";


/**
 * <summary>Defines the simulation.</summary>
 */
export class Simulator {
  /**
   * <summary>Defines a worker.</summary>
   */

  public agents_: Agent[];
  public obstacles_: Obstacle[];
  public kdTree_: KdTree;
  public timeStep_: number;

  private defaultAgent_: Agent;
  // private doneEvents_: ManualResetEvent[];
  private workers_: RVOWorker[] | null = null;
  private numWorkers_: number;
  private globalTime_: number;

  public static readonly Instance: Simulator = new Simulator();

  /**
   * <summary>Constructs and initializes a simulation.</summary>
   */
  constructor() {
    this.clear();
    this.setNumWorkers(1);
  }

  /**
   * <summary>Adds a new agent to the simulation.</summary>
   *
   * <returns>The number of the agent.</returns>
   *
   * <param name="position">The two-dimensional starting position of this
   * agent.</param>
   * <param name="neighborDist">The maximum distance (center point to
   * center point) to other agents this agent takes into account in the
   * navigation. The larger this number, the longer the running time of
   * the simulation. If the number is too low, the simulation will not be
   * safe. Must be non-negative.</param>
   * <param name="maxNeighbors">The maximum number of other agents this
   * agent takes into account in the navigation. The larger this number,
   * the longer the running time of the simulation. If the number is too
   * low, the simulation will not be safe.</param>
   * <param name="timeHorizon">The minimal amount of time for which this
   * agent's velocities that are computed by the simulation are safe with
   * respect to other agents. The larger this number, the sooner this
   * agent will respond to the presence of other agents, but the less
   * freedom this agent has in choosing its velocities. Must be positive.
   * </param>
   * <param name="timeHorizonObst">The minimal amount of time for which
   * this agent's velocities that are computed by the simulation are safe
   * with respect to obstacles. The larger this number, the sooner this
   * agent will respond to the presence of obstacles, but the less freedom
   * this agent has in choosing its velocities. Must be positive.</param>
   * <param name="radius">The radius of this agent. Must be non-negative.
   * </param>
   * <param name="maxSpeed">The maximum speed of this agent. Must be
   * non-negative.</param>
   * <param name="velocity">The initial two-dimensional linear velocity of
   * this agent.</param>
   */
  public addAgent(position: Vector2,
    neighborDist: number = this.defaultAgent_.neighborDist_,
    maxNeighbors: number = this.defaultAgent_.maxNeighbors_,
    timeHorizon: number = this.defaultAgent_.timeHorizon_,
    timeHorizonObst: number = this.defaultAgent_.timeHorizonObst_,
    radius: number = this.defaultAgent_.radius_,
    maxSpeed: number = this.defaultAgent_.maxSpeed_,
    velocity: Vector2 = this.defaultAgent_.velocity_
  ) {
    const agent = new Agent(this);
    agent.id_ = this.agents_.length;
    agent.maxNeighbors_ = maxNeighbors;
    agent.maxSpeed_ = maxSpeed;
    agent.neighborDist_ = neighborDist;
    agent.position_ = position;
    agent.radius_ = radius;
    agent.timeHorizon_ = timeHorizon;
    agent.timeHorizonObst_ = timeHorizonObst;
    agent.velocity_ = velocity;
    this.agents_.push(agent);

    return agent.id_;
  }

  /**
   * <summary>Adds a new obstacle to the simulation.</summary>
   *
   * <returns>The number of the first vertex of the obstacle, or -1 when
   * the number of vertices is less than two.</returns>
   *
   * <param name="vertices">List of the vertices of the polygonal obstacle
   * in counterclockwise order.</param>
   *
   * <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
   * the environment, the vertices should be listed in clockwise order.
   * </remarks>
   */
  public addObstacle(vertices: Vector2[]) {
    if (vertices.length < 2) {
      return -1;
    }

    let obstacleNo = this.obstacles_.length;

    for (let i = 0; i < vertices.length; ++i) {
      const obstacle = new Obstacle();
      obstacle.point_ = vertices[i];

      if (i != 0) {
        obstacle.previous_ = this.obstacles_[this.obstacles_.length - 1];
        obstacle.previous_.next_ = obstacle;
      }

      if (i == vertices.length - 1) {
        obstacle.next_ = this.obstacles_[obstacleNo];
        obstacle.next_.previous_ = obstacle;
      }

      obstacle.direction_ = RVOMath.normalize(Vector2.subtract(vertices[(i == vertices.length - 1 ? 0 : i + 1)], vertices[i]));

      if (vertices.length == 2) {
        obstacle.convex_ = true;
      }
      else {
        obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0.0);
      }

      obstacle.id_ = this.obstacles_.length;
      this.obstacles_.push(obstacle);
    }

    return obstacleNo;
  }

  /**
   * <summary>Clears the simulation.</summary>
   */
  public clear() {
    this.agents_ = [];
    this.defaultAgent_ = null;
    this.kdTree_ = new KdTree(this);
    this.obstacles_ = [];
    this.globalTime_ = 0;
    this.timeStep_ = 0.1;
    this.setNumWorkers(0);
  }

  /**
   * <summary>Performs a simulation step and updates the two-dimensional
   * position and two-dimensional velocity of each agent.</summary>
   *
   * <returns>The global time after the simulation step.</returns>
   */
  public doStep() {
    if (this.workers_ == null) {
      this.workers_ = new Array<RVOWorker>(this.numWorkers_);
      // this.doneEvents_ = new Array<ManualResetEvent>(this.workers_.length);

      for (let block = 0; block < this.workers_.length; ++block) {
        // this.doneEvents_[block] = new ManualResetEvent(false);
        this.workers_[block] = new RVOWorker(this, block * this.getNumAgents() / this.workers_.length, (block + 1) * this.getNumAgents() / this.workers_.length); //, this.doneEvents_[block]);
      }
    }

    this.kdTree_.buildAgentTree();

    for (let block = 0; block < this.workers_.length; ++block) {
      // this.doneEvents_[block].Reset();
      // ThreadPool.QueueUserWorkItem(this.workers_[block].step);
      this.workers_[block].step();
    }

    // WaitHandle.WaitAll(this.doneEvents_);

    for (let block = 0; block < this.workers_.length; ++block) {
      // this.doneEvents_[block].Reset();
      // ThreadPool.QueueUserWorkItem(this.workers_[block].update);
      this.workers_[block].update();
    }

    // WaitHandle.WaitAll(this.doneEvents_);

    this.globalTime_ += this.timeStep_;

    return this.globalTime_;
  }

  /**
   * <summary>Returns the specified agent neighbor of the specified agent.
   * </summary>
   *
   * <returns>The number of the neighboring agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose agent neighbor is
   * to be retrieved.</param>
   * <param name="neighborNo">The number of the agent neighbor to be
   * retrieved.</param>
   */
  public getAgentAgentNeighbor(agentNo: number, neighborNo: number) {
    return this.agents_[agentNo].agentNeighbors_[neighborNo][1].id_;
  }

  /**
   * <summary>Returns the maximum neighbor count of a specified agent.
   * </summary>
   *
   * <returns>The present maximum neighbor count of the agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose maximum neighbor
   * count is to be retrieved.</param>
   */
  public getAgentMaxNeighbors(agentNo: number) {
    return this.agents_[agentNo].maxNeighbors_;
  }

  /**
   * <summary>Returns the maximum speed of a specified agent.</summary>
   *
   * <returns>The present maximum speed of the agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose maximum speed is
   * to be retrieved.</param>
   */
  public getAgentMaxSpeed(agentNo: number) {
    return this.agents_[agentNo].maxSpeed_;
  }

  /**
   * <summary>Returns the maximum neighbor distance of a specified agent.
   * </summary>
   *
   * <returns>The present maximum neighbor distance of the agent.
   * </returns>
   *
   * <param name="agentNo">The number of the agent whose maximum neighbor
   * distance is to be retrieved.</param>
   */
  public getAgentNeighborDist(agentNo: number) {
    return this.agents_[agentNo].neighborDist_;
  }

  /**
   * <summary>Returns the count of agent neighbors taken into account to
   * compute the current velocity for the specified agent.</summary>
   *
   * <returns>The count of agent neighbors taken into account to compute
   * the current velocity for the specified agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose count of agent
   * neighbors is to be retrieved.</param>
   */
  public getAgentNumAgentNeighbors(agentNo: number) {
    return this.agents_[agentNo].agentNeighbors_.length;
  }

  /**
   * <summary>Returns the count of obstacle neighbors taken into account
   * to compute the current velocity for the specified agent.</summary>
   *
   * <returns>The count of obstacle neighbors taken into account to
   * compute the current velocity for the specified agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose count of obstacle
   * neighbors is to be retrieved.</param>
   */
  public getAgentNumObstacleNeighbors(agentNo: number) {
    return this.agents_[agentNo].obstacleNeighbors_.length;
  }

  /**
   * <summary>Returns the specified obstacle neighbor of the specified
   * agent.</summary>
   *
   * <returns>The number of the first vertex of the neighboring obstacle
   * edge.</returns>
   *
   * <param name="agentNo">The number of the agent whose obstacle neighbor
   * is to be retrieved.</param>
   * <param name="neighborNo">The number of the obstacle neighbor to be
   * retrieved.</param>
   */
  public getAgentObstacleNeighbor(agentNo: number, neighborNo: number) {
    return this.agents_[agentNo].obstacleNeighbors_[neighborNo][1].id_;
  }

  /**
   * <summary>Returns the ORCA constraints of the specified agent.
   * </summary>
   *
   * <returns>A list of lines representing the ORCA constraints.</returns>
   *
   * <param name="agentNo">The number of the agent whose ORCA constraints
   * are to be retrieved.</param>
   *
   * <remarks>The halfplane to the left of each line is the region of
   * permissible velocities with respect to that ORCA constraint.
   * </remarks>
   */
  public getAgentOrcaLines(agentNo: number): Line[] {
    return this.agents_[agentNo].orcaLines_;
  }

  /**
   * <summary>Returns the two-dimensional position of a specified agent.
   * </summary>
   *
   * <returns>The present two-dimensional position of the (center of the)
   * agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * position is to be retrieved.</param>
   */
  public getAgentPosition(agentNo: number): Vector2 {
    return this.agents_[agentNo].position_;
  }

  /**
   * <summary>Returns the two-dimensional preferred velocity of a
   * specified agent.</summary>
   *
   * <returns>The present two-dimensional preferred velocity of the agent.
   * </returns>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * preferred velocity is to be retrieved.</param>
   */
  public getAgentPrefVelocity(agentNo: number): Vector2 {
    return this.agents_[agentNo].prefVelocity_;
  }

  /**
   * <summary>Returns the radius of a specified agent.</summary>
   *
   * <returns>The present radius of the agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose radius is to be
   * retrieved.</param>
   */
  public getAgentRadius(agentNo: number) {
    return this.agents_[agentNo].radius_;
  }

  /**
   * <summary>Returns the time horizon of a specified agent.</summary>
   *
   * <returns>The present time horizon of the agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose time horizon is
   * to be retrieved.</param>
   */
  public getAgentTimeHorizon(agentNo: number) {
    return this.agents_[agentNo].timeHorizon_;
  }

  /**
   * <summary>Returns the time horizon with respect to obstacles of a
   * specified agent.</summary>
   *
   * <returns>The present time horizon with respect to obstacles of the
   * agent.</returns>
   *
   * <param name="agentNo">The number of the agent whose time horizon with
   * respect to obstacles is to be retrieved.</param>
   */
  public getAgentTimeHorizonObst(agentNo: number) {
    return this.agents_[agentNo].timeHorizonObst_;
  }

  /**
   * <summary>Returns the two-dimensional linear velocity of a specified
   * agent.</summary>
   *
   * <returns>The present two-dimensional linear velocity of the agent.
   * </returns>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * linear velocity is to be retrieved.</param>
   */
  public getAgentVelocity(agentNo: number): Vector2 {
    return this.agents_[agentNo].velocity_;
  }

  /**
   * <summary>Returns the global time of the simulation.</summary>
   *
   * <returns>The present global time of the simulation (zero initially).
   * </returns>
   */
  public getGlobalTime() {
    return this.globalTime_;
  }

  /**
   * <summary>Returns the count of agents in the simulation.</summary>
   *
   * <returns>The count of agents in the simulation.</returns>
   */
  public getNumAgents() {
    return this.agents_.length;
  }

  /**
   * <summary>Returns the count of obstacle vertices in the simulation.
   * </summary>
   *
   * <returns>The count of obstacle vertices in the simulation.</returns>
   */
  public getNumObstacleVertices() {
    return this.obstacles_.length;
  }

  /**
   * <summary>Returns the count of workers.</summary>
   *
   * <returns>The count of workers.</returns>
   */
  public GetNumWorkers() {
    return this.numWorkers_;
  }

  /**
   * <summary>Returns the two-dimensional position of a specified obstacle
   * vertex.</summary>
   *
   * <returns>The two-dimensional position of the specified obstacle
   * vertex.</returns>
   *
   * <param name="vertexNo">The number of the obstacle vertex to be
   * retrieved.</param>
   */
  public getObstacleVertex(vertexNo: number): Vector2 {
    return this.obstacles_[vertexNo].point_;
  }

  /**
   * <summary>Returns the number of the obstacle vertex succeeding the
   * specified obstacle vertex in its polygon.</summary>
   *
   * <returns>The number of the obstacle vertex succeeding the specified
   * obstacle vertex in its polygon.</returns>
   *
   * <param name="vertexNo">The number of the obstacle vertex whose
   * successor is to be retrieved.</param>
   */
  public getNextObstacleVertexNo(vertexNo: number) {
    return this.obstacles_[vertexNo].next_.id_;
  }

  /**
   * <summary>Returns the number of the obstacle vertex preceding the
   * specified obstacle vertex in its polygon.</summary>
   *
   * <returns>The number of the obstacle vertex preceding the specified
   * obstacle vertex in its polygon.</returns>
   *
   * <param name="vertexNo">The number of the obstacle vertex whose
   * predecessor is to be retrieved.</param>
   */
  public getPrevObstacleVertexNo(vertexNo: number) {
    return this.obstacles_[vertexNo].previous_.id_;
  }

  /**
   * <summary>Returns the time step of the simulation.</summary>
   *
   * <returns>The present time step of the simulation.</returns>
   */
  public getTimeStep() {
    return this.timeStep_;
  }

  /**
   * <summary>Processes the obstacles that have been added so that they
   * are accounted for in the simulation.</summary>
   *
   * <remarks>Obstacles added to the simulation after this function has
   * been called are not accounted for in the simulation.</remarks>
   */
  public processObstacles() {
    this.kdTree_.buildObstacleTree();
  }

  /**
   * <summary>Performs a visibility query between the two specified points
   * with respect to the obstacles.</summary>
   *
   * <returns>A boolean specifying whether the two points are mutually
   * visible. Returns true when the obstacles have not been processed.
   * </returns>
   *
   * <param name="point1">The first point of the query.</param>
   * <param name="point2">The second point of the query.</param>
   * <param name="radius">The minimal distance between the line connecting
   * the two points and the obstacles in order for the points to be
   * mutually visible (optional). Must be non-negative.</param>
   */
  public queryVisibility(point1: Vector2, point2: Vector2, radius: number) {
    return this.kdTree_.queryVisibility(point1, point2, radius);
  }

  /**
   * <summary>Sets the default properties for any new agent that is added.
   * </summary>
   *
   * <param name="neighborDist">The default maximum distance (center point
   * to center point) to other agents a new agent takes into account in
   * the navigation. The larger this number, the longer he running time of
   * the simulation. If the number is too low, the simulation will not be
   * safe. Must be non-negative.</param>
   * <param name="maxNeighbors">The default maximum number of other agents
   * a new agent takes into account in the navigation. The larger this
   * number, the longer the running time of the simulation. If the number
   * is too low, the simulation will not be safe.</param>
   * <param name="timeHorizon">The default minimal amount of time for
   * which a new agent's velocities that are computed by the simulation
   * are safe with respect to other agents. The larger this number, the
   * sooner an agent will respond to the presence of other agents, but the
   * less freedom the agent has in choosing its velocities. Must be
   * positive.</param>
   * <param name="timeHorizonObst">The default minimal amount of time for
   * which a new agent's velocities that are computed by the simulation
   * are safe with respect to obstacles. The larger this number, the
   * sooner an agent will respond to the presence of obstacles, but the
   * less freedom the agent has in choosing its velocities. Must be
   * positive.</param>
   * <param name="radius">The default radius of a new agent. Must be
   * non-negative.</param>
   * <param name="maxSpeed">The default maximum speed of a new agent. Must
   * be non-negative.</param>
   * <param name="velocity">The default initial two-dimensional linear
   * velocity of a new agent.</param>
   */
  public setAgentDefaults(neighborDist: number, maxNeighbors: number, timeHorizon: number, timeHorizonObst: number, radius: number, maxSpeed: number, velocity: Vector2) {
    if (this.defaultAgent_ == null) {
      this.defaultAgent_ = new Agent(this);
    }

    this.defaultAgent_.maxNeighbors_ = maxNeighbors;
    this.defaultAgent_.maxSpeed_ = maxSpeed;
    this.defaultAgent_.neighborDist_ = neighborDist;
    this.defaultAgent_.radius_ = radius;
    this.defaultAgent_.timeHorizon_ = timeHorizon;
    this.defaultAgent_.timeHorizonObst_ = timeHorizonObst;
    this.defaultAgent_.velocity_ = velocity;
  }

  /**
   * <summary>Sets the maximum neighbor count of a specified agent.
   * </summary>
   *
   * <param name="agentNo">The number of the agent whose maximum neighbor
   * count is to be modified.</param>
   * <param name="maxNeighbors">The replacement maximum neighbor count.
   * </param>
   */
  public setAgentMaxNeighbors(agentNo: number, maxNeighbors: number) {
    this.agents_[agentNo].maxNeighbors_ = maxNeighbors;
  }

  /**
   * <summary>Sets the maximum speed of a specified agent.</summary>
   *
   * <param name="agentNo">The number of the agent whose maximum speed is
   * to be modified.</param>
   * <param name="maxSpeed">The replacement maximum speed. Must be
   * non-negative.</param>
   */
  public setAgentMaxSpeed(agentNo: number, maxSpeed: number) {
    this.agents_[agentNo].maxSpeed_ = maxSpeed;
  }

  /**
   * <summary>Sets the maximum neighbor distance of a specified agent.
   * </summary>
   *
   * <param name="agentNo">The number of the agent whose maximum neighbor
   * distance is to be modified.</param>
   * <param name="neighborDist">The replacement maximum neighbor distance.
   * Must be non-negative.</param>
   */
  public setAgentNeighborDist(agentNo: number, neighborDist: number) {
    this.agents_[agentNo].neighborDist_ = neighborDist;
  }

  /**
   * <summary>Sets the two-dimensional position of a specified agent.
   * </summary>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * position is to be modified.</param>
   * <param name="position">The replacement of the two-dimensional
   * position.</param>
   */
  public setAgentPosition(agentNo: number, position: Vector2) {
    this.agents_[agentNo].position_ = position;
  }

  /**
   * <summary>Sets the two-dimensional preferred velocity of a specified
   * agent.</summary>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * preferred velocity is to be modified.</param>
   * <param name="prefVelocity">The replacement of the two-dimensional
   * preferred velocity.</param>
   */
  public setAgentPrefVelocity(agentNo: number, prefVelocity: Vector2) {
    this.agents_[agentNo].prefVelocity_ = prefVelocity;
  }

  /**
   * <summary>Sets the radius of a specified agent.</summary>
   *
   * <param name="agentNo">The number of the agent whose radius is to be
   * modified.</param>
   * <param name="radius">The replacement radius. Must be non-negative.
   * </param>
   */
  public setAgentRadius(agentNo: number, radius: number) {
    this.agents_[agentNo].radius_ = radius;
  }

  /**
   * <summary>Sets the time horizon of a specified agent with respect to
   * other agents.</summary>
   *
   * <param name="agentNo">The number of the agent whose time horizon is
   * to be modified.</param>
   * <param name="timeHorizon">The replacement time horizon with respect
   * to other agents. Must be positive.</param>
   */
  public setAgentTimeHorizon(agentNo: number, timeHorizon: number) {
    this.agents_[agentNo].timeHorizon_ = timeHorizon;
  }

  /**
   * <summary>Sets the time horizon of a specified agent with respect to
   * obstacles.</summary>
   *
   * <param name="agentNo">The number of the agent whose time horizon with
   * respect to obstacles is to be modified.</param>
   * <param name="timeHorizonObst">The replacement time horizon with
   * respect to obstacles. Must be positive.</param>
   */
  public setAgentTimeHorizonObst(agentNo: number, timeHorizonObst: number) {
    this.agents_[agentNo].timeHorizonObst_ = timeHorizonObst;
  }

  /**
   * <summary>Sets the two-dimensional linear velocity of a specified
   * agent.</summary>
   *
   * <param name="agentNo">The number of the agent whose two-dimensional
   * linear velocity is to be modified.</param>
   * <param name="velocity">The replacement two-dimensional linear
   * velocity.</param>
   */
  public setAgentVelocity(agentNo: number, velocity: Vector2) {
    this.agents_[agentNo].velocity_ = velocity;
  }

  /**
   * <summary>Sets the global time of the simulation.</summary>
   *
   * <param name="globalTime">The global time of the simulation.</param>
   */
  public setGlobalTime(globalTime: number) {
    this.globalTime_ = globalTime;
  }

  /**
   * <summary>Sets the number of workers.</summary>
   *
   * <param name="numWorkers">The number of workers.</param>
   */
  public setNumWorkers(numWorkers: number) {
    this.numWorkers_ = numWorkers;

    // if (this.numWorkers_ <= 0) {
    //   ThreadPool.GetMinThreads(out this.numWorkers_, out _);
    // }
    this.workers_ = null;
  }

  /**
   * <summary>Sets the time step of the simulation.</summary>
   *
   * <param name="timeStep">The time step of the simulation. Must be
   * positive.</param>
   */
  public setTimeStep(timeStep: number) {
    this.timeStep_ = timeStep;
  }

}
