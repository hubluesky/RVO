import { Agent } from "./Agent";
import { KdTree } from "./KdTree";
import { Line } from "./Line";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Vector2 } from "./Vector2";

type int = number;

class Worker {
    end_: int;
    start_: int;

    public constructor(public readonly simulator: Simulator, start: int, end: int) {
        this.start_ = start;
        this.end_ = end;
    }

    config(start: int, end: int): void {
        this.start_ = start;
        this.end_ = end;
    }

    step(): void {
        for (let index = this.start_; index < this.end_; ++index) {
            let agent = this.simulator.agents_[index];
            agent.computeNeighbors();
            agent.computeNewVelocity();
        }
    }

    update(): void {
        for (let index = this.start_; index < this.end_; ++index) {
            this.simulator.agents_[index].update();
        }
    }
}

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
export class Simulator {
    agentNo2indexDict_: Map<int, int>; // get
    index2agentNoDict_: Map<int, int>;
    agents_: Array<Agent>;
    obstacles_: Array<Obstacle>;
    kdTree_: KdTree;
    timeStep_: number;

    static s_totalID = 0;
    private static instance_ = new Simulator();
    public static get Instance() { return Simulator.instance_; }

    private defaultAgent_: Agent;
    private workers_: Worker[];
    private numWorkers_: int;
    private workerAgentCount_: int;
    private globalTime_: number;

    public constructor() {
        this.clear();
    }

    delAgent(agentNo: int): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].needDelete_ = true;
    }

    updateDeleteAgent(): void {
        let isDelete = false;
        for (let i = this.agents_.length - 1; i >= 0; i--) {
            if (this.agents_[i].needDelete_) {
                this.agents_.splice(i, 1);
                isDelete = true;
            }
        }
        if (isDelete)
            this.onDelAgent();
    }

    addAgent(position: Vector2): int {
        if (this.defaultAgent_ == null) {
            return -1;
        }

        let agent = new Agent(this);
        agent.id_ = Simulator.s_totalID;
        Simulator.s_totalID++;
        agent.maxNeighbors_ = this.defaultAgent_.maxNeighbors_;
        agent.maxSpeed_ = this.defaultAgent_.maxSpeed_;
        agent.neighborDist_ = this.defaultAgent_.neighborDist_;
        agent.position_ = position.clone();
        agent.radius_ = this.defaultAgent_.radius_;
        agent.timeHorizon_ = this.defaultAgent_.timeHorizon_;
        agent.timeHorizonObst_ = this.defaultAgent_.timeHorizonObst_;
        agent.velocity_ = this.defaultAgent_.velocity_.clone();
        agent.prefVelocity_ = new Vector2();
        this.agents_.push(agent);
        this.onAddAgent();
        return agent.id_;
    }

    onDelAgent(): void {
        this.agentNo2indexDict_.clear();
        this.index2agentNoDict_.clear();

        for (let i = 0; i < this.agents_.length; i++) {
            let agentNo = this.agents_[i].id_;
            this.agentNo2indexDict_.set(agentNo, i);
            this.index2agentNoDict_.set(i, agentNo);
        }
    }

    onAddAgent(): void {
        if (this.agents_.length == 0)
            return;

        let index = this.agents_.length - 1;
        let agentNo = this.agents_[index].id_;
        this.agentNo2indexDict_.set(agentNo, index);
        this.index2agentNoDict_.set(index, agentNo);
    }

    addAgent2(position: Vector2, neighborDist: number, maxNeighbors: int, timeHorizon: number, timeHorizonObst: number, radius: number, maxSpeed: number, velocity: Vector2) {
        let agent = new Agent(this);
        agent.id_ = Simulator.s_totalID;
        Simulator.s_totalID++;
        agent.neighborDist_ = neighborDist;
        agent.maxNeighbors_ = maxNeighbors;
        agent.timeHorizon_ = timeHorizon;
        agent.timeHorizonObst_ = timeHorizonObst;
        agent.radius_ = radius;
        agent.maxSpeed_ = maxSpeed;
        agent.position_ = position.clone();
        agent.velocity_ = velocity.clone();
        agent.prefVelocity_ = new Vector2();
        this.agents_.push(agent);
        this.onAddAgent();
        return agent.id_;
    }

    addObstacle(vertices: Vector2[]): number {
        if (vertices.length < 2) {
            return -1;
        }

        let obstacleNo = this.obstacles_.length;

        for (let i = 0; i < vertices.length; ++i) {
            let obstacle = new Obstacle();
            obstacle.point_ = vertices[i];

            if (i != 0) {
                obstacle.previous_ = this.obstacles_[this.obstacles_.length - 1];
                obstacle.previous_.next_ = obstacle;
            }

            if (i == vertices.length - 1) {
                obstacle.next_ = this.obstacles_[obstacleNo];
                obstacle.next_.previous_ = obstacle;
            }

            let vt = new Vector2();
            obstacle.direction_ = RVOMath.normalize(Vector2.subtract(vertices[(i == vertices.length - 1 ? 0 : i + 1)], vertices[i], vt), vt);

            if (vertices.length == 2) {
                obstacle.convex_ = true;
            } else {
                obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            obstacle.id_ = this.obstacles_.length;
            this.obstacles_.push(obstacle);
        }

        return obstacleNo;
    }

    clear(): void {
        this.agents_ = new Array<Agent>();
        this.agentNo2indexDict_ = new Map<int, int>();
        this.index2agentNoDict_ = new Map<int, int>();
        this.defaultAgent_ = null;
        this.kdTree_ = new KdTree(this);
        this.obstacles_ = new Array<Obstacle>();
        this.globalTime_ = 0;
        this.timeStep_ = 0.1;

        this.setNumWorkers(10);
    }

    doStep(): number {
        this.updateDeleteAgent();

        let numAgents = this.getNumAgents();
        let workersLength = this.numWorkers_;
        if (this.workers_ == null) {
            this.workers_ = new Array<Worker>(workersLength);
            this.workerAgentCount_ = numAgents;

            for (let block = 0; block < workersLength; ++block) {
                this.workers_[block] = new Worker(this, Math.trunc(block * numAgents / workersLength), Math.trunc((block + 1) * numAgents / workersLength));
            }
        }

        if (this.workerAgentCount_ != numAgents) {
            this.workerAgentCount_ = numAgents;
            for (let block = 0; block < workersLength; ++block) {
                this.workers_[block].config(Math.trunc(block * numAgents / workersLength), Math.trunc((block + 1) * numAgents / workersLength));
            }
        }

        this.kdTree_.buildAgentTree();

        for (let block = 0; block < workersLength; ++block) {
            this.workers_[block].step();
        }

        for (let block = 0; block < workersLength; ++block) {
            this.workers_[block].update();
        }

        this.globalTime_ += this.timeStep_;
        return this.globalTime_;
    }

    getAgentAgentNeighbor(agentNo: int, neighborNo: int): int {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].agentNeighbors_[neighborNo].value.id_;
    }

    getAgentMaxNeighbors(agentNo: int): int {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].maxNeighbors_;
    }

    getAgentMaxSpeed(agentNo: int): number {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].maxSpeed_;
    }

    getAgentNeighborDist(agentNo: int): number {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].neighborDist_;
    }

    getAgentNumAgentNeighbors(agentNo: int): int {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].agentNeighbors_.length;
    }

    getAgentNumObstacleNeighbors(agentNo: int): int {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].obstacleNeighbors_.length;
    }

    getAgentObstacleNeighbor(agentNo: int, neighborNo: int): int {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].obstacleNeighbors_[neighborNo].value.id_;
    }

    getAgentOrcaLines(agentNo: int): Line[] {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].orcaLines_;
    }

    getAgentPosition(agentNo: int): Vector2 {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].position_;
    }

    getAgentPrefVelocity(agentNo: int): Vector2 {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].prefVelocity_;
    }

    getAgentRadius(agentNo: int): number {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].radius_;
    }

    getAgentTimeHorizon(agentNo: int): number {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].timeHorizon_;
    }

    getAgentTimeHorizonObst(agentNo: int): number {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].timeHorizonObst_;
    }

    getAgentVelocity(agentNo: int): Vector2 {
        let index = this.agentNo2indexDict_.get(agentNo);
        return this.agents_[index].velocity_;
    }

    getGlobalTime(): number {
        return this.globalTime_;
    }

    getNumAgents(): number {
        return this.agents_.length;
    }

    getNumObstacleVertices(): int {
        return this.obstacles_.length;
    }

    getNumWorkers(): int {
        return this.numWorkers_;
    }

    getObstacleVertex(vertexNo: int): Vector2 {
        return this.obstacles_[vertexNo].point_;
    }

    getNextObstacleVertexNo(vertexNo: int): int {
        return this.obstacles_[vertexNo].next_.id_;
    }

    getPrevObstacleVertexNo(vertexNo: int): int {
        return this.obstacles_[vertexNo].previous_.id_;
    }

    getTimeStep(): number {
        return this.timeStep_;
    }

    processObstacles(): void {
        this.kdTree_.buildObstacleTree();
    }

    queryVisibility(point1: Vector2, point2: Vector2, radius: number): boolean {
        return this.kdTree_.queryVisibility(point1, point2, radius);
    }

    queryNearAgent(point: Vector2, radius: number): int {
        if (this.getNumAgents() == 0)
            return -1;
        return this.kdTree_.queryNearAgent(point, radius);
    }

    setAgentDefaults(neighborDist: number, maxNeighbors: int, timeHorizon: number, timeHorizonObst: number, radius: number, maxSpeed: number, velocity: Vector2 = new Vector2(0.0, 0.0)) {
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

    setAgentMaxNeighbors(agentNo: int, maxNeighbors: int): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].maxNeighbors_ = maxNeighbors;
    }

    setAgentMaxSpeed(agentNo: int, maxSpeed: number): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].maxSpeed_ = maxSpeed;
    }

    setAgentNeighborDist(agentNo: int, neighborDist: number): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].neighborDist_ = neighborDist;
    }

    setAgentPosition(agentNo: int, position: Vector2): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].position_ = position;
    }

    setAgentPrefVelocity(agentNo: int, prefVelocity: Vector2): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].prefVelocity_.copy(prefVelocity);
    }

    setAgentRadius(agentNo: int, radius: number): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].radius_ = radius;
    }

    setAgentTimeHorizon(agentNo: int, timeHorizon: number) {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].timeHorizon_ = timeHorizon;
    }

    setAgentTimeHorizonObst(agentNo: int, timeHorizonObst: number): void {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].timeHorizonObst_ = timeHorizonObst;
    }

    setAgentVelocity(agentNo: int, velocity: Vector2) {
        let index = this.agentNo2indexDict_.get(agentNo);
        this.agents_[index].velocity_ = velocity;
    }

    setGlobalTime(globalTime: number): void {
        this.globalTime_ = globalTime;
    }

    setNumWorkers(numWorkers: number) {
        this.numWorkers_ = numWorkers;
        this.workers_ = null;
        this.workerAgentCount_ = 0;
    }

    setTimeStep(timeStep: number): void {
        this.timeStep_ = timeStep;
    }
}