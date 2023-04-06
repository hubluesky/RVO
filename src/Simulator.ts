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
 * Defines the simulation.
 */
export class Simulator {
  public agents_: Agent[] = [];
  public obstacles_: Obstacle[] = [];
  public kdTree_: KdTree;
  public timeStep_: number;

  private defaultAgent_: Agent;
  // private doneEvents_: ManualResetEvent[];
  private workers_: RVOWorker[] | null = null;
  private numWorkers_: number;
  private globalTime_: number;
  public agentCount: number = 0;
  public obstacleCount: number = 0;

  /**
   * Constructs and initializes a simulation.
   */
  constructor() {
    this.clear();
  }

  public forEachAgent(callback: (agentNo: number) => void) {
    for (const agentNo in this.agents_) {
      callback(agentNo as any);
      // callback(this.agents_[agentNo].id_);
    }
  }

  /**
   * Adds a new agent to the simulation.
   * @param position The two-dimensional starting position of this agent.
   * @param neighborDist The maximum distance (center point to
   * center point) to other agents this agent takes into account in the
   * navigation. The larger this number, the longer the running time of
   * the simulation. If the number is too low, the simulation will not be
   * safe. Must be non-negative.
   * @param maxNeighbors The maximum number of other agents this
   * agent takes into account in the navigation. The larger this number,
   * the longer the running time of the simulation. If the number is too
   * low, the simulation will not be safe.
   * @param timeHorizon The minimal amount of time for which this
   * agent's velocities that are computed by the simulation are safe with
   * respect to other agents. The larger this number, the sooner this
   * agent will respond to the presence of other agents, but the less
   * freedom this agent has in choosing its velocities. Must be positive.
   * @param timeHorizonObst The minimal amount of time for which
   * this agent's velocities that are computed by the simulation are safe
   * with respect to obstacles. The larger this number, the sooner this
   * agent will respond to the presence of obstacles, but the less freedom
   * this agent has in choosing its velocities. Must be positive.
   * @param radius The radius of this agent. Must be non-negative.
   * @param maxSpeed The maximum speed of this agent. Must be non-negative.
   * @returns The number of the agent.
   */
  public addAgent(position: Vector2,
    neighborDist: number = this.defaultAgent_.neighborDist_,
    maxNeighbors: number = this.defaultAgent_.maxNeighbors_,
    timeHorizon: number = this.defaultAgent_.timeHorizon_,
    timeHorizonObst: number = this.defaultAgent_.timeHorizonObst_,
    radius: number = this.defaultAgent_.radius_,
    maxSpeed: number = this.defaultAgent_.maxSpeed_,
  ) {
    const agent = new Agent(this);
    agent.id_ = this.agents_.length;
    agent.maxNeighbors_ = maxNeighbors;
    agent.maxSpeed_ = maxSpeed;
    agent.neighborDist_ = neighborDist;
    agent.position_.set(position);
    agent.radius_ = radius;
    agent.timeHorizon_ = timeHorizon;
    agent.timeHorizonObst_ = timeHorizonObst;
    this.agents_.push(agent);
    this.agentCount++;
    return agent.id_;
  }

  public delAgent(agentNo: number): void {
    const agent = this.agents_[agentNo];
    if (agent == null) return;
    this.kdTree_.delAgent(agent);
    delete this.agents_[agentNo];
    this.agentCount--;
  }

  /**
   * Adds a new obstacle to the simulation.
   * To add a "negative" obstacle, e.g. a bounding polygon around
   * the environment, the vertices should be listed in clockwise order.
   * @param vertices List of the vertices of the polygonal obstacle in counterclockwise order.
   * @returns The number of the first vertex of the obstacle, or -1 when the number of vertices is less than two.
   */
  public addObstacle(vertices: Vector2[]): number {
    if (vertices.length < 2)
      return -1;

    let obstacleNo = this.obstacleCount;

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

    this.obstacleCount++;
    return obstacleNo;
  }

  public delObstacle(obstacleNo: number): void {
    for (let i = this.obstacles_.length - 1; i >= 0; i--) {
      const obstacle = this.obstacles_[i];
      if (obstacle.id_ == obstacleNo)
        this.obstacles_.splice(i, 1);
    }
    this.obstacleCount--;
    this.kdTree_.buildObstacleTree();
  }

  /**
   * Clears the simulation.
   */
  public clear() {
    this.agents_.length = 0;
    this.defaultAgent_ = null;
    this.kdTree_ = new KdTree(this);
    this.obstacles_.length = 0;
    this.globalTime_ = 0;
    this.timeStep_ = 0.1;
    // this.setNumWorkers(1);
  }

  /**
   * Performs a simulation step and updates the two-dimensional
   * position and two-dimensional velocity of each agent.
   * @returns The global time after the simulation step.
   */
  public doStep() {
    // if (this.workers_ == null) {
    //   this.workers_ = new Array<RVOWorker>(this.numWorkers_);
    //   // this.doneEvents_ = new Array<ManualResetEvent>(this.workers_.length);

    //   for (let block = 0; block < this.workers_.length; ++block) {
    //     // this.doneEvents_[block] = new ManualResetEvent(false);
    //     this.workers_[block] = new RVOWorker(this, block * this.getNumAgents() / this.workers_.length, (block + 1) * this.getNumAgents() / this.workers_.length); //, this.doneEvents_[block]);
    //   }
    // }

    this.kdTree_.buildAgentTree();

    for (const agentNo in this.agents_) {
      const agent = this.agents_[agentNo];
      if (agent.isFreeze) continue;
      agent.computeNeighbors();
      agent.computeNewVelocity();
    }

    for (const agentNo in this.agents_) {
      const agent = this.agents_[agentNo];
      if (agent.isFreeze) continue;
      agent.update();
    }
    // for (let block = 0; block < this.workers_.length; ++block) {
    //   // this.doneEvents_[block].Reset();
    //   // ThreadPool.QueueUserWorkItem(this.workers_[block].step);
    //   this.workers_[block].step();
    // }

    // WaitHandle.WaitAll(this.doneEvents_);

    // for (let block = 0; block < this.workers_.length; ++block) {
    // this.doneEvents_[block].Reset();
    // ThreadPool.QueueUserWorkItem(this.workers_[block].update);
    //   this.workers_[block].update();
    // }

    // WaitHandle.WaitAll(this.doneEvents_);

    this.globalTime_ += this.timeStep_;

    return this.globalTime_;
  }

  /**
   * Returns the specified agent neighbor of the specified agent.
   * @param agentNo The number of the agent whose agent neighbor is to be retrieved.
   * @param neighborNo The number of the agent neighbor to be retrieved.
   * @returns The number of the neighboring agent.
   */
  public getAgentAgentNeighbor(agentNo: number, neighborNo: number): number {
    return this.agents_[agentNo].agentNeighbors_[neighborNo][1].id_;
  }

  /**
   * Returns the maximum neighbor count of a specified agent.
   * @param agentNo The number of the agent whose maximum neighbor count is to be retrieved.
   * @returns The present maximum neighbor count of the agent.
   */
  public getAgentMaxNeighbors(agentNo: number): number {
    return this.agents_[agentNo].maxNeighbors_;
  }

  /**
   * Returns the maximum speed of a specified agent.
   * @param agentNo The number of the agent whose maximum speed is to be retrieved.
   * @returns The present maximum speed of the agent.
   */
  public getAgentMaxSpeed(agentNo: number): number {
    return this.agents_[agentNo].maxSpeed_;
  }

  /**
   * Returns the maximum neighbor distance of a specified agent.
   * @param agentNo The number of the agent whose maximum neighbor distance is to be retrieved.
   * @returns The present maximum neighbor distance of the agent.
   */
  public getAgentNeighborDist(agentNo: number): number {
    return this.agents_[agentNo].neighborDist_;
  }

  /**
   * Returns the count of agent neighbors taken into account to
   * compute the current velocity for the specified agent.
   * @param agentNo The number of the agent whose count of agent neighbors is to be retrieved.
   * @returns The count of agent neighbors taken into account to compute
   * the current velocity for the specified agent.
   */
  public getAgentNumAgentNeighbors(agentNo: number): number {
    return this.agents_[agentNo].agentNeighbors_.length;
  }

  /**
   * Returns the count of obstacle neighbors taken into account
   * to compute the current velocity for the specified agent.
   * @param agentNo The number of the agent whose count of obstacle neighbors is to be retrieved.
   * @returns The count of obstacle neighbors taken into account to
   * compute the current velocity for the specified agent.
   */
  public getAgentNumObstacleNeighbors(agentNo: number): number {
    return this.agents_[agentNo].obstacleNeighbors_.length;
  }

  /**
   * Returns the specified obstacle neighbor of the specified
   * agent.
   * @param agentNo The number of the agent whose obstacle neighbor is to be retrieved.
   * @param neighborNo The number of the obstacle neighbor to be retrieved.
   * @returns The number of the first vertex of the neighboring obstacle
   * edge.
   */
  public getAgentObstacleNeighbor(agentNo: number, neighborNo: number): number {
    return this.agents_[agentNo].obstacleNeighbors_[neighborNo][1].id_;
  }

  /**
   * Returns the ORCA constraints of the specified agent.
   * The halfplane to the left of each line is the region of
   * permissible velocities with respect to that ORCA constraint.
   * @param agentNo The number of the agent whose ORCA constraints
   * are to be retrieved.
   * @returns A list of lines representing the ORCA constraints.
   */
  public getAgentOrcaLines(agentNo: number): readonly Line[] {
    return this.agents_[agentNo].orcaLines_;
  }

  /**
   * Returns the two-dimensional position of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional position is to be retrieved.
   * @returns The present two-dimensional position of the (center of the) agent.
   */
  public getAgentPosition(agentNo: number): Vector2 {
    return this.agents_[agentNo].position_;
  }

  /**
   * Returns the two-dimensional preferred velocity of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be retrieved.
   * @returns The present two-dimensional preferred velocity of the agent.
   */
  public getAgentPrefVelocity(agentNo: number): Vector2 {
    return this.agents_[agentNo].prefVelocity_;
  }

  /**
   * Returns the radius of a specified agent.
   * @param agentNo The number of the agent whose radius is to be retrieved.
   * @returns The present radius of the agent.
   */
  public getAgentRadius(agentNo: number): number {
    return this.agents_[agentNo].radius_;
  }

  /**
   * Returns the time horizon of a specified agent.
   * @param agentNo The number of the agent whose time horizon is to be retrieved.
   * @returns The present time horizon of the agent.
   */
  public getAgentTimeHorizon(agentNo: number): number {
    return this.agents_[agentNo].timeHorizon_;
  }

  /**
   * Returns the time horizon with respect to obstacles of a specified agent.
   * @param agentNo The number of the agent whose time horizon with respect to obstacles is to be retrieved.
   * @returns The present time horizon with respect to obstacles of the agent.
   */
  public getAgentTimeHorizonObst(agentNo: number): number {
    return this.agents_[agentNo].timeHorizonObst_;
  }

  /**
   * Returns the two-dimensional linear velocity of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional linear velocity is to be retrieved.
   * @returns The present two-dimensional linear velocity of the agent.
   */
  public getAgentVelocity(agentNo: number): Vector2 {
    return this.agents_[agentNo].velocity_;
  }

  /**
   * Returns the global time of the simulation.
   * @returns The present global time of the simulation (zero initially).
   */
  public getGlobalTime(): number {
    return this.globalTime_;
  }

  /**
   * Returns the count of obstacle vertices in the simulation.
   * @returns The count of obstacle vertices in the simulation.
   */
  public getNumObstacleVertices(): number {
    return this.obstacles_.length;
  }

  /**
   * Returns the count of workers.
   * @returns The count of workers.
   */
  public getNumWorkers(): number {
    return this.numWorkers_;
  }

  /**
   * Returns the two-dimensional position of a specified obstacle vertex.
   * @param vertexNo The number of the obstacle vertex to be retrieved.
   * @returns The two-dimensional position of the specified obstacle vertex.
   */
  public getObstacleVertex(vertexNo: number): Vector2 {
    return this.obstacles_[vertexNo].point_;
  }

  /**
   * Returns the number of the obstacle vertex succeeding the specified obstacle vertex in its polygon.
   * @param vertexNo The number of the obstacle vertex whose successor is to be retrieved.
   * @returns The number of the obstacle vertex succeeding the specified obstacle vertex in its polygon.
   */
  public getNextObstacleVertexNo(vertexNo: number): number {
    return this.obstacles_[vertexNo].next_.id_;
  }

  /**
   * Returns the number of the obstacle vertex preceding the specified obstacle vertex in its polygon.
   * @param vertexNo The number of the obstacle vertex whose predecessor is to be retrieved.
   * @returns The number of the obstacle vertex preceding the specified obstacle vertex in its polygon.
   */
  public getPrevObstacleVertexNo(vertexNo: number): number {
    return this.obstacles_[vertexNo].previous_.id_;
  }

  /**
   * Returns the time step of the simulation.
   * @returns The present time step of the simulation.
   */
  public getTimeStep(): number {
    return this.timeStep_;
  }

  /**
   * Processes the obstacles that have been added so that they are accounted for in the simulation.
   * Obstacles added to the simulation after this function has been called are not accounted for in the simulation.
   */
  public processObstacles(): void {
    this.kdTree_.buildObstacleTree();
  }

  /**
   * Performs a visibility query between the two specified points with respect to the obstacles.
   * @param point1 The first point of the query.
   * @param point2 The second point of the query.
   * @param radius The minimal distance between the line connecting
   * the two points and the obstacles in order for the points to be
   * mutually visible (optional). Must be non-negative.
   * @returns A boolean specifying whether the two points are mutually
   * visible. Returns true when the obstacles have not been processed.
   */
  public queryVisibility(point1: Vector2, point2: Vector2, radius: number): boolean {
    return this.kdTree_.queryVisibility(point1, point2, radius);
  }

  /**
   * Sets the default properties for any new agent that is added.
   * @param neighborDist The default maximum distance (center point
   * to center point) to other agents a new agent takes into account in
   * the navigation. The larger this number, the longer he running time of
   * the simulation. If the number is too low, the simulation will not be
   * safe. Must be non-negative.
   * @param maxNeighbors The default maximum number of other agents
   * a new agent takes into account in the navigation. The larger this
   * number, the longer the running time of the simulation. If the number
   * is too low, the simulation will not be safe.
   * @param timeHorizon The default minimal amount of time for
   * which a new agent's velocities that are computed by the simulation
   * are safe with respect to other agents. The larger this number, the
   * sooner an agent will respond to the presence of other agents, but the
   * less freedom the agent has in choosing its velocities. Must be
   * positive.
   * @param timeHorizonObst The default minimal amount of time for
   * which a new agent's velocities that are computed by the simulation
   * are safe with respect to obstacles. The larger this number, the
   * sooner an agent will respond to the presence of obstacles, but the
   * less freedom the agent has in choosing its velocities. Must be
   * positive.
   * @param radius The default radius of a new agent. Must be non-negative.
   * @param maxSpeed The default maximum speed of a new agent. Must be non-negative.
   * @param velocity The default initial two-dimensional linear velocity of a new agent.
   */
  public setAgentDefaults(neighborDist: number, maxNeighbors: number, timeHorizon: number, timeHorizonObst: number, radius: number, maxSpeed: number): void {
    if (this.defaultAgent_ == null)
      this.defaultAgent_ = new Agent(this);

    this.defaultAgent_.maxNeighbors_ = maxNeighbors;
    this.defaultAgent_.maxSpeed_ = maxSpeed;
    this.defaultAgent_.neighborDist_ = neighborDist;
    this.defaultAgent_.radius_ = radius;
    this.defaultAgent_.timeHorizon_ = timeHorizon;
    this.defaultAgent_.timeHorizonObst_ = timeHorizonObst;
  }

  public freezeAgent(agentNo: number): void {
    this.agents_[agentNo].isFreeze = true;
    this.agents_[agentNo].velocity_.reset();
  }

  public unfreezeAgent(agentNo: number): void {
    this.agents_[agentNo].isFreeze = false;
  }

  /**
   * Sets the maximum neighbor count of a specified agent.
   * @param agentNo The number of the agent whose maximum neighbor count is to be modified.
   * @param maxNeighbors The replacement maximum neighbor count.
   */
  public setAgentMaxNeighbors(agentNo: number, maxNeighbors: number): void {
    this.agents_[agentNo].maxNeighbors_ = maxNeighbors;
  }

  /**
   * Sets the maximum speed of a specified agent.
   * @param agentNo The number of the agent whose maximum speed is to be modified.
   * @param maxSpeed The replacement maximum speed. Must be non-negative.
   */
  public setAgentMaxSpeed(agentNo: number, maxSpeed: number): void {
    this.agents_[agentNo].maxSpeed_ = maxSpeed;
  }

  /**
   * Sets the maximum neighbor distance of a specified agent.
   * @param agentNo The number of the agent whose maximum neighbor distance is to be modified.
   * @param neighborDist The replacement maximum neighbor distance. Must be non-negative.
   */
  public setAgentNeighborDist(agentNo: number, neighborDist: number): void {
    this.agents_[agentNo].neighborDist_ = neighborDist;
  }

  /**
   * Sets the two-dimensional position of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional position is to be modified.
   * @param position The replacement of the two-dimensional position.
   */
  public setAgentPosition(agentNo: number, position: Vector2): void {
    this.agents_[agentNo].position_.set(position);
  }

  /**
   * Sets the two-dimensional preferred velocity of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be modified.
   * @param prefVelocity The replacement of the two-dimensional preferred velocity.
   */
  public setAgentPrefVelocity(agentNo: number, prefVelocity: Vector2): void {
    this.agents_[agentNo].prefVelocity_.set(prefVelocity);
  }

  /**
   * Sets the radius of a specified agent.
   * @param agentNo The number of the agent whose radius is to be modified.
   * @param radius The replacement radius. Must be non-negative.
   */
  public setAgentRadius(agentNo: number, radius: number): void {
    this.agents_[agentNo].radius_ = radius;
  }

  /**
   * Sets the time horizon of a specified agent with respect to other agents.
   * @param agentNo The number of the agent whose time horizon is to be modified.
   * @param timeHorizon The replacement time horizon with respect to other agents. Must be positive.
   */
  public setAgentTimeHorizon(agentNo: number, timeHorizon: number): void {
    this.agents_[agentNo].timeHorizon_ = timeHorizon;
  }

  /**
   * Sets the time horizon of a specified agent with respect to obstacles.
   * @param agentNo The number of the agent whose time horizon with respect to obstacles is to be modified.
   * @param timeHorizonObst The replacement time horizon with respect to obstacles. Must be positive.
   */
  public setAgentTimeHorizonObst(agentNo: number, timeHorizonObst: number): void {
    this.agents_[agentNo].timeHorizonObst_ = timeHorizonObst;
  }

  /**
   * Sets the two-dimensional linear velocity of a specified agent.
   * @param agentNo The number of the agent whose two-dimensional linear velocity is to be modified.
   * @param velocity The replacement two-dimensional linear velocity.
   */
  public setAgentVelocity(agentNo: number, velocity: Vector2): void {
    this.agents_[agentNo].velocity_.set(velocity);
  }

  /**
   * Sets the global time of the simulation.
   * @param globalTime The global time of the simulation.
   */
  public setGlobalTime(globalTime: number): void {
    this.globalTime_ = globalTime;
  }

  /**
   * Sets the number of workers.
   * @param numWorkers The number of workers.
   */
  public setNumWorkers(numWorkers: number): void {
    this.numWorkers_ = numWorkers;

    // if (this.numWorkers_ <= 0) {
    //   ThreadPool.GetMinThreads(out this.numWorkers_, out _);
    // }
    this.workers_ = null;
  }

  /**
   * Sets the time step of the simulation.
   * @param timeStep The time step of the simulation. Must be positive.
   */
  public setTimeStep(timeStep: number): void {
    this.timeStep_ = timeStep;
  }
}
