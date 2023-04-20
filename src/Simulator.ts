import { Agent } from "./Agent";
import { KdTree } from "./KdTree";
import { Line } from "./Line";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Vector2 } from "./Vector2";

/*
 * Copyright 2008 University of North Carolina at Chapel Hill
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

/**
 * Defines the simulation.
 */
export class Simulator {
    private readonly agents: Array<Agent> = [];
    readonly obstacles: Array<Obstacle> = [];
    kdTree: KdTree;
    timeStep: number;

    private globalTime: number;
    public agentCount: number = 0;
    public obstacleCount: number = 0;
    /**
     * Constructs and initializes a simulation.
     */
    public constructor() {
        this.clear();
    }

    public forEachAgent(callback: (agentNo: number) => void) {
        for (const agentNo in this.agents) {
            callback(agentNo as any);
            // callback(this.agents_[agentNo].id_);
        }
    }

    /**
     * Adds a new agent to the simulation.
     * @param position The two-dimensional starting position of this agent.
     * @param radius The radius of this agent. Must be non-negative.
     * @param maxSpeed The maximum speed of this agent. Must be non-negative.
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
     * @param maxNeighbors The default maximum number of other agents
     * a new agent takes into account in the navigation. The larger this
     * number, the longer the running time of the simulation. If the number
     * is too low, the simulation will not be safe.
     * @returns The number of the agent.
     */
    addAgent(position: Vector2, radius: number, maxSpeed: number, timeHorizon: number = 1, timeHorizonObst: number = 1, maxNeighbors: number = 10): number {
        let agent = new Agent(this, this.agents.length);
        agent.position.set(position);
        agent.radius = radius;
        agent.maxSpeed = maxSpeed;
        agent.timeHorizon = timeHorizon;
        agent.timeHorizonObst = timeHorizonObst;
        agent.maxNeighbors = maxNeighbors;
        this.agents.push(agent);
        this.agentCount++;
        return agent.id;
    }

    delAgent(agentNo: number): void {
        const agent = this.agents[agentNo];
        if (agent == null) return;
        this.kdTree.delAgent(agent);
        delete this.agents[agentNo];
        this.agentCount--;
    }
    /**
     * Adds a new obstacle to the simulation.
     * To add a "negative" obstacle, e.g. a bounding polygon around
     * the environment, the vertices should be listed in clockwise order.
     * @param vertices List of the vertices of the polygonal obstacle in counterclockwise order.
     * @returns The number of the first vertex of the obstacle, or -1 when the number of vertices is less than two.
     */
    addObstacle(vertices: readonly Vector2[]): number {
        console.assert(vertices.length >= 2);
        if (vertices.length < 2)
            return -1;

        let obstacleNo = this.obstacles.length;

        for (let i = 0; i < vertices.length; ++i) {
            let obstacle = new Obstacle(obstacleNo);
            obstacle.point = vertices[i];

            if (i > 0) {
                obstacle.previous = this.obstacles[this.obstacles.length - 1];
                obstacle.previous.next = obstacle;
            }

            if (i == vertices.length - 1) {
                obstacle.next = this.obstacles[obstacleNo];
                obstacle.next.previous = obstacle;
            }

            Vector2.subtract(vertices[(i == vertices.length - 1 ? 0 : i + 1)], vertices[i], obstacle.direction).normalize();

            if (vertices.length == 2) {
                obstacle.convex = true;
            } else {
                obstacle.convex = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            this.obstacles.push(obstacle);
        }

        return obstacleNo;
    }

    public delObstacle(obstacleNo: number): boolean {
        let result: boolean = false;
        for (let i = this.obstacles.length - 1; i >= 0; i--) {
            const obstacle = this.obstacles[i];
            if (obstacle.id == obstacleNo) {
                result = true;
                this.obstacles.splice(i, 1);
                // if (obstacle.previous_) obstacle.previous_.next_ = null;
                // if (obstacle.next_) obstacle.next_.previous_ = null;
            }
        }
        if (result)
            this.obstacleCount--;
        return result;
    }
    /**
     * Clears the simulation.
     */
    clear(): void {
        this.agents.length = 0;
        this.kdTree = new KdTree(this);
        this.obstacles.length = 0;
        this.globalTime = 0;
        this.timeStep = 0.1;
    }
    /**
     * Performs a simulation step and updates the two-dimensional
     * position and two-dimensional velocity of each agent.
     * @returns The global time after the simulation step.
     */
    doStep(): number {
        this.kdTree.buildAgentTree();

        this.forEachAgent((agentNo) => {
            const agent = this.agents[agentNo];
            if (agent.isFreeze) return;
            agent.computeNeighbors();
            agent.computeNewVelocity();
        });

        this.forEachAgent((agentNo) => {
            const agent = this.agents[agentNo];
            if (agent.isFreeze) return;
            agent.update();
        });

        this.globalTime += this.timeStep;
        return this.globalTime;
    }

    public getAgent(agentNo: number): Agent {
        return this.agents[agentNo];
    }

    /**
     * Returns the specified agent neighbor of the specified agent.
     * @param agentNo The number of the agent whose agent neighbor is to be retrieved.
     * @param neighborNo The number of the agent neighbor to be retrieved.
     * @returns The number of the neighboring agent.
     */
    public getAgentAgentNeighbor(agentNo: number, neighborNo: number): number {
        return this.agents[agentNo].agentNeighbors[neighborNo].value.id;
    }

    /**
     * Returns the maximum neighbor count of a specified agent.
     * @param agentNo The number of the agent whose maximum neighbor count is to be retrieved.
     * @returns The present maximum neighbor count of the agent.
     */
    public getAgentMaxNeighbors(agentNo: number): number {
        return this.agents[agentNo].maxNeighbors;
    }

    /**
     * Returns the maximum speed of a specified agent.
     * @param agentNo The number of the agent whose maximum speed is to be retrieved.
     * @returns The present maximum speed of the agent.
     */
    public getAgentMaxSpeed(agentNo: number): number {
        return this.agents[agentNo].maxSpeed;
    }

    /**
     * Returns the count of agent neighbors taken into account to
     * compute the current velocity for the specified agent.
     * @param agentNo The number of the agent whose count of agent neighbors is to be retrieved.
     * @returns The count of agent neighbors taken into account to compute
     * the current velocity for the specified agent.
     */
    public getAgentNumAgentNeighbors(agentNo: number): number {
        return this.agents[agentNo].agentNeighbors.length;
    }

    /**
     * Returns the count of obstacle neighbors taken into account
     * to compute the current velocity for the specified agent.
     * @param agentNo The number of the agent whose count of obstacle neighbors is to be retrieved.
     * @returns The count of obstacle neighbors taken into account to
     * compute the current velocity for the specified agent.
     */
    public getAgentNumObstacleNeighbors(agentNo: number): number {
        return this.agents[agentNo].obstacleNeighbors.length;
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
        return this.agents[agentNo].obstacleNeighbors[neighborNo].value.id;
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
        return this.agents[agentNo].orcaLines;
    }

    /**
     * Returns the two-dimensional position of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional position is to be retrieved.
     * @returns The present two-dimensional position of the (center of the) agent.
     */
    public getAgentPosition(agentNo: number): Vector2 {
        return this.agents[agentNo].position;
    }

    /**
     * Returns the two-dimensional preferred velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be retrieved.
     * @returns The present two-dimensional preferred velocity of the agent.
     */
    public getAgentPrefVelocity(agentNo: number): Vector2 {
        return this.agents[agentNo].prefVelocity;
    }

    /**
     * Returns the radius of a specified agent.
     * @param agentNo The number of the agent whose radius is to be retrieved.
     * @returns The present radius of the agent.
     */
    public getAgentRadius(agentNo: number): number {
        return this.agents[agentNo].radius;
    }

    /**
     * Returns the time horizon of a specified agent.
     * @param agentNo The number of the agent whose time horizon is to be retrieved.
     * @returns The present time horizon of the agent.
     */
    public getAgentTimeHorizon(agentNo: number): number {
        return this.agents[agentNo].timeHorizon;
    }

    /**
     * Returns the time horizon with respect to obstacles of a specified agent.
     * @param agentNo The number of the agent whose time horizon with respect to obstacles is to be retrieved.
     * @returns The present time horizon with respect to obstacles of the agent.
     */
    public getAgentTimeHorizonObst(agentNo: number): number {
        return this.agents[agentNo].timeHorizonObst;
    }

    /**
     * Returns the two-dimensional linear velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional linear velocity is to be retrieved.
     * @returns The present two-dimensional linear velocity of the agent.
     */
    public getAgentVelocity(agentNo: number): Vector2 {
        return this.agents[agentNo].velocity;
    }

    /**
     * Returns the global time of the simulation.
     * @returns The present global time of the simulation (zero initially).
     */
    public getGlobalTime(): number {
        return this.globalTime;
    }

    /**
     * Returns the count of obstacle vertices in the simulation.
     * @returns The count of obstacle vertices in the simulation.
     */
    public getNumObstacleVertices(): number {
        return this.obstacles.length;
    }

    /**
     * Returns the two-dimensional position of a specified obstacle vertex.
     * @param vertexNo The number of the obstacle vertex to be retrieved.
     * @returns The two-dimensional position of the specified obstacle vertex.
     */
    public getObstacleVertex(vertexNo: number): Vector2 {
        return this.obstacles[vertexNo].point;
    }

    /**
     * Returns the number of the obstacle vertex succeeding the specified obstacle vertex in its polygon.
     * @param vertexNo The number of the obstacle vertex whose successor is to be retrieved.
     * @returns The number of the obstacle vertex succeeding the specified obstacle vertex in its polygon.
     */
    public getNextObstacleVertexNo(vertexNo: number): number {
        return this.obstacles[vertexNo].next.id;
    }

    /**
     * Returns the number of the obstacle vertex preceding the specified obstacle vertex in its polygon.
     * @param vertexNo The number of the obstacle vertex whose predecessor is to be retrieved.
     * @returns The number of the obstacle vertex preceding the specified obstacle vertex in its polygon.
     */
    public getPrevObstacleVertexNo(vertexNo: number): number {
        return this.obstacles[vertexNo].previous.id;
    }

    /**
     * Returns the time step of the simulation.
     * @returns The present time step of the simulation.
     */
    public getTimeStep(): number {
        return this.timeStep;
    }
    /**
     * Processes the obstacles that have been added so that they are accounted for in the simulation.
     * Obstacles added to the simulation after this function has been called are not accounted for in the simulation.
     */
    processObstacles(): void {
        this.kdTree.buildObstacleTree();
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
    queryVisibility(point1: Vector2, point2: Vector2, radius: number): boolean {
        return this.kdTree.queryVisibility(point1, point2, radius);
    }

    queryNearAgent(point: Vector2, radius: number, filter: (target: Agent) => boolean = () => false): number {
        if (this.agentCount == 0) return -1;
        return this.kdTree.queryNearAgent(point, radius, filter);
    }

    public freezeAgent(agentNo: number): void {
        this.agents[agentNo].isFreeze = true;
        this.agents[agentNo].velocity.reset();
    }

    public unfreezeAgent(agentNo: number): void {
        this.agents[agentNo].isFreeze = false;
    }

    public setAgentUserData<T>(agentNo: number, userData: T): void {
        this.agents[agentNo].userData = userData;
    }

    public getAgentUserData<T>(agentNo: number): T {
        return this.agents[agentNo].userData;
    }

    /**
     * Sets the maximum neighbor count of a specified agent.
     * @param agentNo The number of the agent whose maximum neighbor count is to be modified.
     * @param maxNeighbors The replacement maximum neighbor count.
     */
    public setAgentMaxNeighbors(agentNo: number, maxNeighbors: number): void {
        this.agents[agentNo].maxNeighbors = maxNeighbors;
    }

    /**
     * Sets the maximum speed of a specified agent.
     * @param agentNo The number of the agent whose maximum speed is to be modified.
     * @param maxSpeed The replacement maximum speed. Must be non-negative.
     */
    public setAgentMaxSpeed(agentNo: number, maxSpeed: number): void {
        this.agents[agentNo].maxSpeed = maxSpeed;
    }

    /**
     * Sets the two-dimensional position of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional position is to be modified.
     * @param position The replacement of the two-dimensional position.
     */
    public setAgentPosition(agentNo: number, position: Vector2): void {
        this.agents[agentNo].position.set(position);
    }

    /**
     * Sets the two-dimensional preferred velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be modified.
     * @param prefVelocity The replacement of the two-dimensional preferred velocity.
     */
    public setAgentPrefVelocity(agentNo: number, prefVelocity: Vector2): void {
        this.agents[agentNo].prefVelocity.set(prefVelocity);
    }

    /**
     * Sets the radius of a specified agent.
     * @param agentNo The number of the agent whose radius is to be modified.
     * @param radius The replacement radius. Must be non-negative.
     */
    public setAgentRadius(agentNo: number, radius: number): void {
        this.agents[agentNo].radius = radius;
    }

    /**
     * Sets the time horizon of a specified agent with respect to other agents.
     * @param agentNo The number of the agent whose time horizon is to be modified.
     * @param timeHorizon The replacement time horizon with respect to other agents. Must be positive.
     */
    public setAgentTimeHorizon(agentNo: number, timeHorizon: number): void {
        this.agents[agentNo].timeHorizon = timeHorizon;
    }

    /**
     * Sets the time horizon of a specified agent with respect to obstacles.
     * @param agentNo The number of the agent whose time horizon with respect to obstacles is to be modified.
     * @param timeHorizonObst The replacement time horizon with respect to obstacles. Must be positive.
     */
    public setAgentTimeHorizonObst(agentNo: number, timeHorizonObst: number): void {
        this.agents[agentNo].timeHorizonObst = timeHorizonObst;
    }

    /**
     * Sets the two-dimensional linear velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional linear velocity is to be modified.
     * @param velocity The replacement two-dimensional linear velocity.
     */
    public setAgentVelocity(agentNo: number, velocity: Vector2): void {
        this.agents[agentNo].velocity.set(velocity);
    }

    /**
     * Sets the global time of the simulation.
     * @param globalTime The global time of the simulation.
     */
    public setGlobalTime(globalTime: number): void {
        this.globalTime = globalTime;
    }

    /**
     * Sets the time step of the simulation.
     * @param timeStep The time step of the simulation. Must be positive.
     */
    public setTimeStep(timeStep: number): void {
        this.timeStep = timeStep;
    }
}