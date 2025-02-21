import { Agent } from "./Agent";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";
import { IVector2, Vector2 } from "./Vector2";

const __vecTemp1 = new Vector2();
const __vecTemp2 = new Vector2();

/**
 * Defines a node of an agent k-D tree.
 */
class AgentTreeNode {
    begin: number;
    end: number;
    left: number;
    right: number;
    maxX: number;
    maxY: number;
    minX: number;
    minY: number;
}

/**
 * Defines a pair of scalar values.
 */
class FloatPair {
    /**
     * Constructs and initializes a pair of scalar values.
     * @param a The first scalar value.
     * @param b The second scalar value.
     */
    public constructor(private a: number, private b: number) { }

    /**
     * Returns true if the first pair of scalar values is less than the second pair of scalar values.
     * @param pair1 The first pair of scalar values.
     * @param pair2 The second pair of scalar values.
     * @returns True if the first pair of scalar values is less than the second pair of scalar values.
     */
    public static lessThan(pair1: FloatPair, pair2: FloatPair): boolean {
        return pair1.a < pair2.a || !(pair2.a < pair1.a) && pair1.b < pair2.b;
    }

    /**
     * Returns true if the first pair of scalar values is less than or equal to the second pair of scalar values.
     * @param pair1 The first pair of scalar values.
     * @param pair2 The second pair of scalar values.
     * @returns True if the first pair of scalar values is less than or equal to the second pair of scalar values.
     */
    public static lessThanEq(pair1: FloatPair, pair2: FloatPair): boolean {
        return (pair1.a == pair2.a && pair1.b == pair2.b) || FloatPair.lessThan(pair1, pair2);
    }

    /**
     * Returns true if the first pair of scalar values is greater than the second pair of scalar values.
     * @param pair1 The first pair of scalar values.
     * @param pair2 The second pair of scalar values.
     * @returns True if the first pair of scalar values is greater than the second pair of scalar values.
     */
    public static greaterThan(pair1: FloatPair, pair2: FloatPair): boolean {
        return !FloatPair.lessThanEq(pair1, pair2);
    }

    /**
     * Returns true if the first pair of scalar values is greater than or equal to the second pair of scalar values.
     * @param pair1 The first pair of scalar values.
     * @param pair2 The second pair of scalar values.
     * @returns True if the first pair of scalar values is greater than or equal to the second pair of scalar values.
     */
    public static greaterThanEq(pair1: FloatPair, pair2: FloatPair): boolean {
        return !FloatPair.lessThan(pair1, pair2);
    }
}

/**
 * Defines a node of an obstacle k-D tree.
 */
class ObstacleTreeNode {
    obstacle: Obstacle;
    left: ObstacleTreeNode;
    right: ObstacleTreeNode;
};

export class ObstacleKdTree {
    private obstacleTree: ObstacleTreeNode;

    /**
     * Builds an obstacle k-D tree.
     */
    buildObstacleTree(obstacles: Obstacle[]) {
        // this.obstacleTree = new ObstacleTreeNode();
        this.obstacleTree = this.buildObstacleTreeRecursive(Array.from(obstacles), obstacles);
    }

    /**
     * Recursive method for building an obstacle k-D tree.
     * @param obstacles A list of obstacles.
     * @returns An obstacle k-D tree node.
     */
    buildObstacleTreeRecursive(obstacles: readonly Obstacle[], sourceObstacles: Obstacle[]): ObstacleTreeNode {
        if (obstacles.length == 0)
            return null;

        const node = new ObstacleTreeNode();

        let optimalSplit: number = 0;
        let minLeft: number = obstacles.length;
        let minRight: number = obstacles.length;

        for (let i = 0; i < obstacles.length; ++i) {
            let leftSize: number = 0;
            let rightSize: number = 0;

            const obstacleI1 = obstacles[i];
            const obstacleI2 = obstacleI1.next;

            /* Compute optimal split node. */
            for (let j = 0; j < obstacles.length; ++j) {
                if (i == j)
                    continue;

                const obstacleJ1 = obstacles[j];
                const obstacleJ2 = obstacleJ1.next;

                const j1LeftOfI: number = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                const j2LeftOfI: number = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                    ++leftSize;
                } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                    ++rightSize;
                } else {
                    ++leftSize;
                    ++rightSize;
                }

                if (FloatPair.greaterThanEq(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
                    new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight)))) {
                    break;
                }
            }

            if (FloatPair.lessThan(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
                new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight)))) {
                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }

        {
            /* Build split node. */
            const leftObstacles = new Array<Obstacle>(minLeft);
            const rightObstacles = new Array<Obstacle>(minRight);

            let leftCounter: number = 0;
            let rightCounter: number = 0;
            const i: number = optimalSplit;

            const obstacleI1 = obstacles[i];
            const obstacleI2 = obstacleI1.next;

            for (let j = 0; j < obstacles.length; ++j) {
                if (i == j)
                    continue;

                const obstacleJ1 = obstacles[j];
                const obstacleJ2 = obstacleJ1.next;

                const j1LeftOfI: number = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                const j2LeftOfI: number = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                    leftObstacles[leftCounter++] = obstacles[j];
                } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                    rightObstacles[rightCounter++] = obstacles[j];
                } else {
                    /* Split obstacle j. */
                    const i2si1 = Vector2.subtract(obstacleI2.point, obstacleI1.point, __vecTemp1);
                    const t: number = Vector2.cross(i2si1, Vector2.subtract(obstacleJ1.point, obstacleI1.point, __vecTemp2)) / Vector2.cross(i2si1, Vector2.subtract(obstacleJ1.point, obstacleJ2.point, __vecTemp2));

                    const splitPoint: Vector2 = Vector2.subtract(obstacleJ2.point, obstacleJ1.point, __vecTemp1).multiply(t).add(obstacleJ1.point);

                    const newObstacle = new Obstacle(obstacleJ1.id, obstacleJ1.layer);
                    newObstacle.point = splitPoint;
                    newObstacle.previous = obstacleJ1;
                    newObstacle.next = obstacleJ2;
                    newObstacle.convex = true;
                    newObstacle.direction = obstacleJ1.direction;

                    sourceObstacles.push(newObstacle);

                    obstacleJ1.next = newObstacle;
                    obstacleJ2.previous = newObstacle;

                    if (j1LeftOfI > 0.0) {
                        leftObstacles[leftCounter++] = obstacleJ1;
                        rightObstacles[rightCounter++] = newObstacle;
                    } else {
                        rightObstacles[rightCounter++] = obstacleJ1;
                        leftObstacles[leftCounter++] = newObstacle;
                    }
                }
            }

            node.obstacle = obstacleI1;
            node.left = this.buildObstacleTreeRecursive(leftObstacles, sourceObstacles);
            node.right = this.buildObstacleTreeRecursive(rightObstacles, sourceObstacles);

            return node;
        }
    }

    /**
     * Computes the obstacle neighbors of the specified agent.
     * @param agent The agent for which obstacle neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     */
    computeObstacleNeighbors(agent: Agent, rangeSq: number): void {
        this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree);
    }
    /**
     * Queries the visibility between two points within a specified radius.
     * @param q1 The first point between which visibility is to be tested.
     * @param q2 The second point between which visibility is to be tested.
     * @param radius The radius within which visibility is to be tested.
     * @returns True if q1 and q2 are mutually visible within the radius; false otherwise.
     */
    queryVisibility(q1: IVector2, q2: IVector2, radius: number): boolean {
        return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree);
    }


    /**
     * Recursive method for computing the obstacle neighbors of the specified agent.
     * @param agent The agent for which obstacle neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     * @param node The current obstacle k-D node.
     */
    queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode): void {
        if (node == null) return;
        const obstacle1 = node.obstacle;
        const obstacle2 = obstacle1.next;

        const agentLeftOfLine = RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position);

        this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.left : node.right);

        const distSqLine = RVOMath.sqr(agentLeftOfLine) / Vector2.subtract(obstacle2.point, obstacle1.point, __vecTemp1).lengthSq();

        if (distSqLine < rangeSq) {
            if (agentLeftOfLine < 0.0) {
                /*
                 * Try obstacle at this node only if agent is on right side of
                 * obstacle (and can see obstacle).
                 */
                agent.insertObstacleNeighbor(node.obstacle, rangeSq);
            }

            /* Try other side of line. */
            this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.right : node.left);
        }
    }

    /**
     * Recursive method for querying the visibility between two points within a specified radius.
     * @param q1 The first point between which visibility is to be tested.
     * @param q2 The second point between which visibility is to be tested.
     * @param radius The radius within which visibility is to be tested.
     * @param node The current obstacle k-D node.
     * @returns True if q1 and q2 are mutually visible within the radius; false otherwise.
     */
    queryVisibilityRecursive(q1: IVector2, q2: IVector2, radius: number, node: ObstacleTreeNode): boolean {
        if (node == null) return true;

        const obstacle1 = node.obstacle;
        const obstacle2 = obstacle1.next;

        const q1LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q1);
        const q2LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q2);
        const invLengthI = 1.0 / Vector2.subtract(obstacle2.point, obstacle1.point, __vecTemp1).lengthSq();

        if (q1LeftOfI >= 0.0 && q2LeftOfI >= 0.0) {
            return this.queryVisibilityRecursive(q1, q2, radius, node.left) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
                && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.right));
        }

        if (q1LeftOfI <= 0.0 && q2LeftOfI <= 0.0) {
            return this.queryVisibilityRecursive(q1, q2, radius, node.right) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
                && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.left));
        }

        if (q1LeftOfI >= 0.0 && q2LeftOfI <= 0.0) {
            /* One can see through obstacle from left to right. */
            return this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right);
        }

        const point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point);
        const point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point);
        const invLengthQ = 1.0 / Vector2.subtract(q2, q1, __vecTemp1).lengthSq();

        return point1LeftOfQ * point2LeftOfQ >= 0.0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && this.queryVisibilityRecursive(q1, q2, radius, node.left) && this.queryVisibilityRecursive(q1, q2, radius, node.right);
    }
}

/**
 * Defines k-D trees for agents and static obstacles in the simulation.
 */
export class AgentKdTree {
    /**
     * The maximum size of an agent k-D tree leaf.
     */
    private static readonly MAX_LEAF_SIZE = 10;
    private readonly agents: Agent[] = [];
    private agentTree: AgentTreeNode[];

    public constructor(public readonly simulator: Simulator) { }
    /**
     * Builds an agent k-D tree.
     */
    public buildAgentTree(): void {
        const agentsLength = this.simulator.agentCount;
        if (this.agents.length != agentsLength) {
            this.agents.length = 0;

            this.simulator.forEachAgent((agentNo) => {
                this.agents.push(this.simulator.getAgent(agentNo));
            });

            this.agentTree = new Array<AgentTreeNode>(2 * this.agents.length);

            for (let i = 0; i < this.agentTree.length; ++i) {
                this.agentTree[i] = new AgentTreeNode();
            }
        }

        if (this.agents.length > 0) {
            this.buildAgentTreeRecursive(0, this.agents.length, 0);
        }
    }

    public delAgent(agent: Agent): void {
        const index = this.agents.indexOf(agent);
        if (index == -1) return;
        this.agents.splice(index, 1);
    }

    /**
     * Recursive method for building an agent k-D tree.
     * @param begin The beginning agent k-D tree node node index.
     * @param end The ending agent k-D tree node index.
     * @param node The current agent k-D tree node index.
     */
    private buildAgentTreeRecursive(begin: number, end: number, node: number): void {
        const treeNode = this.agentTree[node];
        treeNode.begin = begin;
        treeNode.end = end;
        treeNode.minX = treeNode.maxX = this.agents[begin].position.x;
        treeNode.minY = treeNode.maxY = this.agents[begin].position.y;

        for (let i = begin + 1; i < end; ++i) {
            treeNode.maxX = Math.max(treeNode.maxX, this.agents[i].position.x);
            treeNode.minX = Math.min(treeNode.minX, this.agents[i].position.x);
            treeNode.maxY = Math.max(treeNode.maxY, this.agents[i].position.y);
            treeNode.minY = Math.min(treeNode.minY, this.agents[i].position.y);
        }

        if (end - begin > AgentKdTree.MAX_LEAF_SIZE) {
            /* No leaf node. */
            const isVertical: boolean = treeNode.maxX - treeNode.minX > treeNode.maxY - treeNode.minY;
            const splitValue: number = 0.5 * (isVertical ? treeNode.maxX + treeNode.minX : treeNode.maxY + treeNode.minY);

            let left: number = begin;
            let right: number = end;

            while (left < right) {
                while (left < right && (isVertical ? this.agents[left].position.x : this.agents[left].position.y) < splitValue) {
                    ++left;
                }

                while (right > left && (isVertical ? this.agents[right - 1].position.x : this.agents[right - 1].position.y) >= splitValue) {
                    --right;
                }

                if (left < right) {
                    const tempAgent = this.agents[left];
                    this.agents[left] = this.agents[right - 1];
                    this.agents[right - 1] = tempAgent;
                    ++left;
                    --right;
                }
            }

            let leftSize: number = left - begin;

            if (leftSize == 0) {
                ++leftSize;
                ++left;
                ++right;
            }

            treeNode.left = node + 1;
            treeNode.right = node + 2 * leftSize;

            this.buildAgentTreeRecursive(begin, left, treeNode.left);
            this.buildAgentTreeRecursive(left, end, treeNode.right);
        }
    }

    /**
     * Computes the agent neighbors of the specified agent.
     * @param agent The agent for which agent neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     */
    public computeAgentNeighbors(agent: Agent, rangeSq: number): number {
        return this.queryAgentTreeRecursive3(agent, rangeSq, 0);
    }

    /**
     * Recursive method for computing the agent neighbors of the specified agent.
     * @param agent The agent for which agent neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     * @param node The current agent k-D tree node index.
     */
    private queryAgentTreeRecursive3(agent: Agent, rangeSq: number, node: number): number {
        const treeNode = this.agentTree[node];
        if (treeNode.end - treeNode.begin <= AgentKdTree.MAX_LEAF_SIZE) {
            for (let i = treeNode.begin; i < treeNode.end; ++i) {
                rangeSq = agent.insertAgentNeighbor(this.agents[i], rangeSq);
            }
        } else {
            const treeNodeLeft = this.agentTree[treeNode.left];
            const treeNodeRight = this.agentTree[treeNode.right];
            const distSqLeft = RVOMath.sqr(Math.max(0.0, treeNodeLeft.minX - agent.position.x))
                + RVOMath.sqr(Math.max(0.0, agent.position.x - treeNodeLeft.maxX))
                + RVOMath.sqr(Math.max(0.0, treeNodeLeft.minY - agent.position.y))
                + RVOMath.sqr(Math.max(0.0, agent.position.y - treeNodeLeft.maxY));
            const distSqRight = RVOMath.sqr(Math.max(0.0, treeNodeRight.minX - agent.position.x))
                + RVOMath.sqr(Math.max(0.0, agent.position.x - treeNodeRight.maxX))
                + RVOMath.sqr(Math.max(0.0, treeNodeRight.minY - agent.position.y))
                + RVOMath.sqr(Math.max(0.0, agent.position.y - treeNodeRight.maxY));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.left);

                    if (distSqRight < rangeSq) {
                        rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.right);
                    }
                }
            } else {
                if (distSqRight < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.right);

                    if (distSqLeft < rangeSq)
                        rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.left);
                }
            }

        }
        return rangeSq;
    }

    public queryNearAgent(point: IVector2, radius: number, callback: (angntId: number) => void): void {
        return this.queryAgentTreeRecursive4(point, radius * radius, 0, callback);
    }

    private queryAgentTreeRecursive4(position: IVector2, rangeSq: number, node: number, callback: (angntId: number) => void): void {
        const treeNode = this.agentTree[node];
        if (treeNode.end - treeNode.begin <= AgentKdTree.MAX_LEAF_SIZE) {
            for (let i = treeNode.begin; i < treeNode.end; ++i) {
                const target = this.agents[i];
                if (target == null) continue;
                const distSq = Vector2.distanceSq(position, target.position);
                if (distSq < rangeSq)
                    callback(target.id);
            }
        } else {
            const treeNodeLeft = this.agentTree[treeNode.left];
            const treeNodeRight = this.agentTree[treeNode.right];
            const distSqLeft: number = RVOMath.sqr(Math.max(0.0, treeNodeLeft.minX - position.x))
                + RVOMath.sqr(Math.max(0.0, position.x - treeNodeLeft.maxX))
                + RVOMath.sqr(Math.max(0.0, treeNodeLeft.minY - position.y))
                + RVOMath.sqr(Math.max(0.0, position.y - treeNodeLeft.maxY));
            const distSqRight: number = RVOMath.sqr(Math.max(0.0, treeNodeRight.minX - position.x))
                + RVOMath.sqr(Math.max(0.0, position.x - treeNodeRight.maxX))
                + RVOMath.sqr(Math.max(0.0, treeNodeRight.minY - position.y))
                + RVOMath.sqr(Math.max(0.0, position.y - treeNodeRight.maxY));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq) {
                    this.queryAgentTreeRecursive4(position, rangeSq, treeNode.left, callback);

                    if (distSqRight < rangeSq)
                        this.queryAgentTreeRecursive4(position, rangeSq, treeNode.right, callback);
                }
            } else {
                if (distSqRight < rangeSq) {
                    this.queryAgentTreeRecursive4(position, rangeSq, treeNode.right, callback);

                    if (distSqLeft < rangeSq)
                        this.queryAgentTreeRecursive4(position, rangeSq, treeNode.left, callback);
                }
            }
        }
    }
}