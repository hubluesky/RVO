import { Agent } from "./Agent";
import { AgentKdTree, ObstacleKdTree } from "./KdTree";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { IVector2, Vector2 } from "./Vector2";

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
    private readonly obstacles: Array<Obstacle> = [];
    private obstrcleTree: ObstacleKdTree;
    private agentTree: AgentKdTree;

    private _agentCount: number = 0;
    public get agentCount(): number { return this._agentCount; }
    private _obstacleCount: number = 0;
    public get obstacleCount(): number { return this._obstacleCount; }

    public readonly layerMasks: number[] = [];

    /**
     * Constructs and initializes a simulation.
     */
    public constructor() {
        this.clear();
    }

    /**
     * 角色碰撞层矩阵，角色的碰撞层（即Layer）不能为0
     * @param matrix 碰撞层矩阵
     */
    public initLayerMatrix(matrix: readonly number[][]): void {
        this.layerMasks.length = matrix.length;
        for (let i = 0; i < matrix.length; i++) {
            let mask = 0;
            for (let j = 0; j < matrix[i].length; j++)
                mask |= 1 << matrix[i][j];
            this.layerMasks[i] = mask;
        }
    }

    public checkLayerMask(layer: number, otherLayer: number): boolean {
        return (this.layerMasks[layer] & (1 << otherLayer)) != 0;
    }

    public forEachAgent(callback: (agentNo: number) => void) {
        for (const agentNo in this.agents) {
            callback(agentNo as any);
        }
    }

    /**
     * Adds a new agent to the simulation.
     * @param layer collision layer of the agent
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
    public addAgent(layer: number, position: IVector2, radius: number, maxSpeed: number, timeHorizon: number = 1, timeHorizonObst: number = 1, maxNeighbors: number = 10): number {
        console.assert(this.layerMasks[layer] != null, "layer " + layer + " is not defined");
        const agent = new Agent(this, this.agents.length, layer);
        agent.position.set(position);
        agent.radius = radius;
        agent.maxSpeed = maxSpeed;
        agent.timeHorizon = timeHorizon;
        agent.timeHorizonObst = timeHorizonObst;
        agent.maxNeighbors = maxNeighbors;
        this.agents.push(agent);
        this._agentCount++;
        return agent.id;
    }

    public delAgent(agentNo: number): void {
        const agent = this.agents[agentNo];
        if (agent == null) return;
        this.agentTree.delAgent(agent);
        delete this.agents[agentNo];
        this._agentCount--;
    }
    /**
     * Adds a new obstacle to the simulation.
     * To add a "negative" obstacle, e.g. a bounding polygon around
     * the environment, the vertices should be listed in clockwise order.
     * @param layer collision layer of the obstacle
     * @param vertices List of the vertices of the polygonal obstacle in counterclockwise order.
     * @returns The number of the first vertex of the obstacle, or -1 when the number of vertices is less than two.
     */
    public addObstacle(layer: number, vertices: readonly IVector2[]): number {
        console.assert(this.layerMasks[layer] != null, "layer " + layer + " is not defined");
        console.assert(vertices.length >= 2);
        if (vertices.length < 2)
            return -1;

        const obstacleNo = this.obstacles.length;

        for (let i = 0; i < vertices.length; ++i) {
            const obstacle = new Obstacle(obstacleNo, layer);
            obstacle.point = vertices[i];

            if (i > 0) {
                obstacle.previous = this.obstacles[this.obstacles.length - 1];
                obstacle.previous.next = obstacle;
            }

            if (i == vertices.length - 1) {
                obstacle.next = this.obstacles[obstacleNo];
                obstacle.next.previous = obstacle;
            }

            Vector2.subtract(vertices[(i == vertices.length - 1 ? 0 : i + 1)], vertices[i], obstacle.direction as Vector2).normalize();

            if (vertices.length == 2) {
                obstacle.convex = true;
            } else {
                obstacle.convex = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            this.obstacles.push(obstacle);
        }

        this._obstacleCount++;
        return obstacleNo;
    }

    /**
     * Creates vertices for a circular obstacle.
     * 
     * This static method generates an array of `Vector2` objects that form a circular obstacle. 
     * The circle is approximated by a polygon with the specified number of segments. Each vertex 
     * is calculated based on the center point, radius, and angle determined by the segment count.
     * 
     * @param center - The center point of the circular obstacle as an `IVector2`.
     * @param radius - The radius of the circular obstacle.
     * @param segments - The number of segments (sides) used to approximate the circle.
     * @returns An array of `Vector2` objects representing the vertices of the circular obstacle.
     */
    public static createCircleObstacleVertices(center: IVector2, radius: number, segments: number): Vector2[] {
        const vertices: Vector2[] = [];
        for (let i = 0; i < segments; i++) {
            const angle = (i * 2 * Math.PI) / segments;
            vertices.push(new Vector2(
                center.x + radius * Math.cos(angle),
                center.y + radius * Math.sin(angle)
            ));
        }
        return vertices;
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
            this._obstacleCount--;
        return result;
    }
    /**
     * Clears the simulation.
     */
    public clear(): void {
        this.agents.length = 0;
        this.agentTree = new AgentKdTree(this);
        this.obstrcleTree = new ObstacleKdTree();
        this.obstacles.length = 0;
    }
    /**
     * Performs a simulation step and updates the two-dimensional
     * position and two-dimensional velocity of each agent.
     * @param timeStep The time step of the simulation. Must be positive.
     */
    public doStep(timeStep: number): void {
        this.agentTree.buildAgentTree();

        this.forEachAgent((agentNo) => {
            const agent = this.agents[agentNo];
            if (agent.isFreeze) return;
            agent.computeNeighbors(this.agentTree, this.obstrcleTree);
            agent.computeNewVelocity(timeStep);
        });

        this.forEachAgent((agentNo) => {
            const agent = this.agents[agentNo];
            if (agent.isFreeze) return;
            agent.update(timeStep);
        });
    }

    public getAgent(agentNo: number): Agent {
        return this.agents[agentNo];
    }

    public getAgentLayer(agentNo: number): number {
        return this.agents[agentNo].layer;
    }

    // /**
    //  * Returns the specified agent neighbor of the specified agent.
    //  * @param agentNo The number of the agent whose agent neighbor is to be retrieved.
    //  * @param neighborNo The number of the agent neighbor to be retrieved.
    //  * @returns The number of the neighboring agent.
    //  */
    // public getAgentAgentNeighbor(agentNo: number, neighborNo: number): number {
    //     return this.agents[agentNo].agentNeighbors[neighborNo].value.id;
    // }

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

    // /**
    //  * Returns the count of agent neighbors taken into account to
    //  * compute the current velocity for the specified agent.
    //  * @param agentNo The number of the agent whose count of agent neighbors is to be retrieved.
    //  * @returns The count of agent neighbors taken into account to compute
    //  * the current velocity for the specified agent.
    //  */
    // public getAgentNumAgentNeighbors(agentNo: number): number {
    //     return this.agents[agentNo].agentNeighbors.length;
    // }

    // /**
    //  * Returns the count of obstacle neighbors taken into account
    //  * to compute the current velocity for the specified agent.
    //  * @param agentNo The number of the agent whose count of obstacle neighbors is to be retrieved.
    //  * @returns The count of obstacle neighbors taken into account to
    //  * compute the current velocity for the specified agent.
    //  */
    // public getAgentNumObstacleNeighbors(agentNo: number): number {
    //     return this.agents[agentNo].obstacleNeighbors.length;
    // }

    // /**
    //  * Returns the specified obstacle neighbor of the specified
    //  * agent.
    //  * @param agentNo The number of the agent whose obstacle neighbor is to be retrieved.
    //  * @param neighborNo The number of the obstacle neighbor to be retrieved.
    //  * @returns The number of the first vertex of the neighboring obstacle
    //  * edge.
    //  */
    // public getAgentObstacleNeighbor(agentNo: number, neighborNo: number): number {
    //     return this.agents[agentNo].obstacleNeighbors[neighborNo].value.id;
    // }

    // /**
    //  * Returns the ORCA constraints of the specified agent.
    //  * The halfplane to the left of each line is the region of
    //  * permissible velocities with respect to that ORCA constraint.
    //  * @param agentNo The number of the agent whose ORCA constraints
    //  * are to be retrieved.
    //  * @returns A list of lines representing the ORCA constraints.
    //  */
    // public getAgentOrcaLines(agentNo: number): readonly Line[] {
    //     return this.agents[agentNo].orcaLines;
    // }

    /**
     * Returns the two-dimensional position of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional position is to be retrieved.
     * @returns The present two-dimensional position of the (center of the) agent.
     */
    public getAgentPosition(agentNo: number): IVector2 {
        return this.agents[agentNo].position;
    }

    /**
     * Returns the two-dimensional preferred velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be retrieved.
     * @returns The present two-dimensional preferred velocity of the agent.
     */
    public getAgentPrefVelocity(agentNo: number): IVector2 {
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
    public getAgentVelocity(agentNo: number): IVector2 {
        return this.agents[agentNo].velocity;
    }

    /**
     * Returns the count of obstacle vertices in the simulation.
     * @returns The count of obstacle vertices in the simulation.
     */
    public getNumObstacleVertices(): number {
        return this.obstacles.length;
    }

    public getObstacleLayer(vertexNo: number): number {
        return this.obstacles[vertexNo].layer;
    }

    public setObstacleLayer(vertexNo: number, layer: number) {
        this.obstacles[vertexNo].layer = layer;
    }

    /**
     * Returns the two-dimensional position of a specified obstacle vertex.
     * @param vertexNo The number of the obstacle vertex to be retrieved.
     * @returns The two-dimensional position of the specified obstacle vertex.
     */
    public getObstacleVertex(vertexNo: number): IVector2 {
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
     * Processes the obstacles that have been added so that they are accounted for in the simulation.
     * Obstacles added to the simulation after this function has been called are not accounted for in the simulation.
     */
    public processObstacles(): void {
        this.obstrcleTree.buildObstacleTree(this.obstacles);
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
    public queryVisibility(point1: IVector2, point2: IVector2, radius: number): boolean {
        return this.obstrcleTree.queryVisibility(point1, point2, radius);
    }

    public queryNearAgent(point: IVector2, radius: number, callback: (angntId: number) => void): void {
        if (this.agentCount == 0) return;
        return this.agentTree.queryNearAgent(point, radius, callback);
    }

    public setAgentAvoidenceWeight(agentNo: number, weight: number): void {
        this.agents[agentNo].avoidenceWeight = weight;
    }

    public getAgentAvoidenceWeight(agentNo: number): number {
        return this.agents[agentNo].avoidenceWeight;
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

    public setAgentLayer(agentNo: number, layer: number): void {
        this.agents[agentNo].layer = layer;
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
    public setAgentPosition(agentNo: number, position: IVector2): void {
        this.agents[agentNo].position.set(position);
    }

    /**
     * Sets the two-dimensional preferred velocity of a specified agent.
     * @param agentNo The number of the agent whose two-dimensional preferred velocity is to be modified.
     * @param prefVelocity The replacement of the two-dimensional preferred velocity.
     */
    public setAgentPrefVelocity(agentNo: number, prefVelocity: IVector2): void {
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
    public setAgentVelocity(agentNo: number, velocity: IVector2): void {
        this.agents[agentNo].velocity.set(velocity);
    }
}