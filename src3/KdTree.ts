import { Agent } from "./Agent";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";
import { Vector2 } from "./Vector2";

const __vecTemp1 = new Vector2();
type int = number;

/**
 * Defines a node of an agent k-D tree.
 */
class AgentTreeNode {
    begin_: int;
    end_: int;
    left_: int;
    right_: int;
    maxX_: number;
    maxY_: number;
    minX_: number;
    minY_: number;
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
    obstacle_: Obstacle;
    left_: ObstacleTreeNode;
    right_: ObstacleTreeNode;
};

/**
 * Defines k-D trees for agents and static obstacles in the simulation.
 */
export class KdTree {
    /**
     * The maximum size of an agent k-D tree leaf.
     */
    private static readonly MAX_LEAF_SIZE = 10;
    private readonly agents_: Agent[] = [];
    private agentTree_: AgentTreeNode[];
    private obstacleTree_: ObstacleTreeNode;

    public constructor(public readonly simulator: Simulator) { }
    /**
     * Builds an agent k-D tree.
     */
    buildAgentTree() {
        let agentsLength = this.simulator.agents_.length;
        if (this.agents_.length != agentsLength) {
            this.agents_.length = 0;

            this.simulator.forEachAgent((agentNo) => {
                this.agents_.push(this.simulator.agents_[agentNo]);
            });

            this.agentTree_ = new Array<AgentTreeNode>(2 * this.agents_.length);

            for (let i = 0; i < this.agentTree_.length; ++i) {
                this.agentTree_[i] = new AgentTreeNode();
            }
        }

        if (this.agents_.length > 0) {
            this.buildAgentTreeRecursive(0, this.agents_.length, 0);
        }
    }

    /**
     * Builds an obstacle k-D tree.
     */
    buildObstacleTree() {
        this.obstacleTree_ = new ObstacleTreeNode();

        let obstacles = new Array<Obstacle>(this.simulator.obstacles_.length);

        for (let i = 0; i < this.simulator.obstacles_.length; ++i) {
            obstacles[i] = this.simulator.obstacles_[i];
        }

        this.obstacleTree_ = this.buildObstacleTreeRecursive(obstacles);
    }

    public delAgent(agent: Agent): void {
        const index = this.agents_.indexOf(agent);
        console.assert(index != -1);
        this.agents_.splice(index, 1);
        this.agentTree_.length = this.agents_.length * 2;
    }
    /**
     * Computes the agent neighbors of the specified agent.
     * @param agent The agent for which agent neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     */
    computeAgentNeighbors(agent: Agent, rangeSq: number): number {
        return this.queryAgentTreeRecursive3(agent, rangeSq, 0);
    }
    /**
     * Computes the obstacle neighbors of the specified agent.
     * @param agent The agent for which obstacle neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     */
    computeObstacleNeighbors(agent: Agent, rangeSq: number): void {
        this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree_);
    }
    /**
     * Queries the visibility between two points within a specified radius.
     * @param q1 The first point between which visibility is to be tested.
     * @param q2 The second point between which visibility is to be tested.
     * @param radius The radius within which visibility is to be tested.
     * @returns True if q1 and q2 are mutually visible within the radius; false otherwise.
     */
    queryVisibility(q1: Vector2, q2: Vector2, radius: number): boolean {
        return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree_);
    }

    queryNearAgent(point: Vector2, radius: number): int {
        let result = this.queryAgentTreeRecursive4(point, Number.MAX_VALUE, -1, 0);
        if (result.rangeSq < radius * radius)
            return result.agentNo;
        return -1;
    }
    /**
     * Recursive method for building an agent k-D tree.
     * @param begin The beginning agent k-D tree node node index.
     * @param end The ending agent k-D tree node index.
     * @param node The current agent k-D tree node index.
     */
    buildAgentTreeRecursive(begin: int, end: int, node: int): void {
        let treeNode = this.agentTree_[node];
        treeNode.begin_ = begin;
        treeNode.end_ = end;
        treeNode.minX_ = treeNode.maxX_ = this.agents_[begin].position_.x;
        treeNode.minY_ = treeNode.maxY_ = this.agents_[begin].position_.y;

        for (let i = begin + 1; i < end; ++i) {
            treeNode.maxX_ = Math.max(treeNode.maxX_, this.agents_[i].position_.x);
            treeNode.minX_ = Math.min(treeNode.minX_, this.agents_[i].position_.x);
            treeNode.maxY_ = Math.max(treeNode.maxY_, this.agents_[i].position_.y);
            treeNode.minY_ = Math.min(treeNode.minY_, this.agents_[i].position_.y);
        }

        if (end - begin > KdTree.MAX_LEAF_SIZE) {
            /* No leaf node. */
            let isVertical: boolean = treeNode.maxX_ - treeNode.minX_ > treeNode.maxY_ - treeNode.minY_;
            let splitValue: number = 0.5 * (isVertical ? treeNode.maxX_ + treeNode.minX_ : treeNode.maxY_ + treeNode.minY_);

            let left: int = begin;
            let right: int = end;

            while (left < right) {
                while (left < right && (isVertical ? this.agents_[left].position_.x : this.agents_[left].position_.y) < splitValue) {
                    ++left;
                }

                while (right > left && (isVertical ? this.agents_[right - 1].position_.x : this.agents_[right - 1].position_.y) >= splitValue) {
                    --right;
                }

                if (left < right) {
                    let tempAgent = this.agents_[left];
                    this.agents_[left] = this.agents_[right - 1];
                    this.agents_[right - 1] = tempAgent;
                    ++left;
                    --right;
                }
            }

            let leftSize: int = left - begin;

            if (leftSize == 0) {
                ++leftSize;
                ++left;
                ++right;
            }

            treeNode.left_ = node + 1;
            treeNode.right_ = node + 2 * leftSize;

            this.buildAgentTreeRecursive(begin, left, treeNode.left_);
            this.buildAgentTreeRecursive(left, end, treeNode.right_);
        }
    }
    /**
     * Recursive method for building an obstacle k-D tree.
     * @param obstacles A list of obstacles.
     * @returns An obstacle k-D tree node.
     */
    buildObstacleTreeRecursive(obstacles: Obstacle[]): ObstacleTreeNode {
        if (obstacles.length == 0)
            return null;

        let node = new ObstacleTreeNode();

        let optimalSplit: int = 0;
        let minLeft: int = obstacles.length;
        let minRight: int = obstacles.length;

        for (let i = 0; i < obstacles.length; ++i) {
            let leftSize: int = 0;
            let rightSize: int = 0;

            let obstacleI1 = obstacles[i];
            let obstacleI2 = obstacleI1.next_;

            /* Compute optimal split node. */
            for (let j = 0; j < obstacles.length; ++j) {
                if (i == j)
                    continue;

                let obstacleJ1 = obstacles[j];
                let obstacleJ2 = obstacleJ1.next_;

                let j1LeftOfI: number = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                let j2LeftOfI: number = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

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
            let leftObstacles = new Array<Obstacle>(minLeft);
            let rightObstacles = new Array<Obstacle>(minRight);

            let leftCounter: int = 0;
            let rightCounter: int = 0;
            let i: int = optimalSplit;

            let obstacleI1 = obstacles[i];
            let obstacleI2 = obstacleI1.next_;

            for (let j = 0; j < obstacles.length; ++j) {
                if (i == j)
                    continue;

                let obstacleJ1 = obstacles[j];
                let obstacleJ2 = obstacleJ1.next_;

                let j1LeftOfI: number = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                let j2LeftOfI: number = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                    leftObstacles[leftCounter++] = obstacles[j];
                } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                    rightObstacles[rightCounter++] = obstacles[j];
                } else {
                    /* Split obstacle j. */
                    let i2si1 = Vector2.subtract(obstacleI2.point_, obstacleI1.point_);
                    let j1si1 = Vector2.subtract(obstacleJ1.point_, obstacleI1.point_);
                    let j1sj2 = Vector2.subtract(obstacleJ1.point_, obstacleJ2.point_);
                    let t: number = RVOMath.det(i2si1, j1si1) / RVOMath.det(i2si1, j1sj2);

                    let j2sj1 = Vector2.subtract(obstacleJ2.point_, obstacleJ1.point_);
                    j2sj1 = Vector2.multiply(j2sj1, t, j2sj1);
                    let splitPoint: Vector2 = Vector2.add(obstacleJ1.point_, j2sj1, j2sj1);

                    let newObstacle = new Obstacle();
                    newObstacle.point_ = splitPoint;
                    newObstacle.previous_ = obstacleJ1;
                    newObstacle.next_ = obstacleJ2;
                    newObstacle.convex_ = true;
                    newObstacle.direction_ = obstacleJ1.direction_.clone();

                    newObstacle.id_ = this.simulator.obstacles_.length;

                    this.simulator.obstacles_.push(newObstacle);

                    obstacleJ1.next_ = newObstacle;
                    obstacleJ2.previous_ = newObstacle;

                    if (j1LeftOfI > 0.0) {
                        leftObstacles[leftCounter++] = obstacleJ1;
                        rightObstacles[rightCounter++] = newObstacle;
                    } else {
                        rightObstacles[rightCounter++] = obstacleJ1;
                        leftObstacles[leftCounter++] = newObstacle;
                    }
                }
            }

            node.obstacle_ = obstacleI1;
            node.left_ = this.buildObstacleTreeRecursive(leftObstacles);
            node.right_ = this.buildObstacleTreeRecursive(rightObstacles);

            return node;
        }
    }
    queryAgentTreeRecursive4(position: Vector2, rangeSq: number, agentNo: int, node: int): { rangeSq: number, agentNo: int } {
        let result = { rangeSq: rangeSq, agentNo: agentNo };
        let treeNode = this.agentTree_[node];
        if (treeNode.end_ - treeNode.begin_ <= KdTree.MAX_LEAF_SIZE) {
            for (let i = treeNode.begin_; i < treeNode.end_; ++i) {
                let distSq = Vector2.subtract(position, this.agents_[i].position_, __vecTemp1).lengthSq();
                if (distSq < result.rangeSq) {
                    result.rangeSq = distSq;
                    result.agentNo = this.agents_[i].id_;
                }
            }
        } else {
            let treeNodeLeft = this.agentTree_[treeNode.left_];
            let treeNodeRight = this.agentTree_[treeNode.right_];
            let distSqLeft: number = RVOMath.sqr(Math.max(0.0, treeNodeLeft.minX_ - position.x))
                + RVOMath.sqr(Math.max(0.0, position.x - treeNodeLeft.maxX_))
                + RVOMath.sqr(Math.max(0.0, treeNodeLeft.minY_ - position.y))
                + RVOMath.sqr(Math.max(0.0, position.y - treeNodeLeft.maxY_));
            let distSqRight: number = RVOMath.sqr(Math.max(0.0, treeNodeRight.minX_ - position.x))
                + RVOMath.sqr(Math.max(0.0, position.x - treeNodeRight.maxX_))
                + RVOMath.sqr(Math.max(0.0, treeNodeRight.minY_ - position.y))
                + RVOMath.sqr(Math.max(0.0, position.y - treeNodeRight.maxY_));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < result.rangeSq) {
                    result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.left_);

                    if (distSqRight < result.rangeSq)
                        result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.right_);
                }
            } else {
                if (distSqRight < result.rangeSq) {
                    result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.right_);

                    if (distSqLeft < result.rangeSq)
                        result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.left_);
                }
            }

        }
        return result;
    }
    /**
     * Recursive method for computing the agent neighbors of the specified agent.
     * @param agent The agent for which agent neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     * @param node The current agent k-D tree node index.
     */
    queryAgentTreeRecursive3(agent: Agent, rangeSq: number, node: int): number {
        let treeNode = this.agentTree_[node];
        if (treeNode.end_ - treeNode.begin_ <= KdTree.MAX_LEAF_SIZE) {
            for (let i = treeNode.begin_; i < treeNode.end_; ++i) {
                rangeSq = agent.insertAgentNeighbor(this.agents_[i], rangeSq);
            }
        } else {
            let treeNodeLeft = this.agentTree_[treeNode.left_];
            let treeNodeRight = this.agentTree_[treeNode.right_];
            let distSqLeft = RVOMath.sqr(Math.max(0.0, treeNodeLeft.minX_ - agent.position_.x))
                + RVOMath.sqr(Math.max(0.0, agent.position_.x - treeNodeLeft.maxX_))
                + RVOMath.sqr(Math.max(0.0, treeNodeLeft.minY_ - agent.position_.y))
                + RVOMath.sqr(Math.max(0.0, agent.position_.y - treeNodeLeft.maxY_));
            let distSqRight = RVOMath.sqr(Math.max(0.0, treeNodeRight.minX_ - agent.position_.x))
                + RVOMath.sqr(Math.max(0.0, agent.position_.x - treeNodeRight.maxX_))
                + RVOMath.sqr(Math.max(0.0, treeNodeRight.minY_ - agent.position_.y))
                + RVOMath.sqr(Math.max(0.0, agent.position_.y - treeNodeRight.maxY_));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.left_);

                    if (distSqRight < rangeSq) {
                        rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.right_);
                    }
                }
            } else {
                if (distSqRight < rangeSq) {
                    rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.right_);

                    if (distSqLeft < rangeSq)
                        rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.left_);
                }
            }

        }
        return rangeSq;
    }
    /**
     * Recursive method for computing the obstacle neighbors of the specified agent.
     * @param agent The agent for which obstacle neighbors are to be computed.
     * @param rangeSq The squared range around the agent.
     * @param node The current obstacle k-D node.
     */
    queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode): void {
        if (node == null) return;
        let obstacle1 = node.obstacle_;
        let obstacle2 = obstacle1.next_;

        let agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

        this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.left_ : node.right_);

        let distSqLine = RVOMath.sqr(agentLeftOfLine) / Vector2.subtract(obstacle2.point_, obstacle1.point_, __vecTemp1).lengthSq();

        if (distSqLine < rangeSq) {
            if (agentLeftOfLine < 0.0) {
                /*
                 * Try obstacle at this node only if agent is on right side of
                 * obstacle (and can see obstacle).
                 */
                agent.insertObstacleNeighbor(node.obstacle_, rangeSq);
            }

            /* Try other side of line. */
            this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.right_ : node.left_);
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
    queryVisibilityRecursive(q1: Vector2, q2: Vector2, radius: number, node: ObstacleTreeNode): boolean {
        if (node == null) return true;

        let obstacle1 = node.obstacle_;
        let obstacle2 = obstacle1.next_;

        let q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
        let q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
        let invLengthI = 1.0 / Vector2.subtract(obstacle2.point_, obstacle1.point_, __vecTemp1).lengthSq();

        if (q1LeftOfI >= 0.0 && q2LeftOfI >= 0.0) {
            return this.queryVisibilityRecursive(q1, q2, radius, node.left_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
                && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.right_));
        }

        if (q1LeftOfI <= 0.0 && q2LeftOfI <= 0.0) {
            return this.queryVisibilityRecursive(q1, q2, radius, node.right_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
                && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.left_));
        }

        if (q1LeftOfI >= 0.0 && q2LeftOfI <= 0.0) {
            /* One can see through obstacle from left to right. */
            return this.queryVisibilityRecursive(q1, q2, radius, node.left_) && this.queryVisibilityRecursive(q1, q2, radius, node.right_);
        }

        let point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
        let point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
        let invLengthQ = 1.0 / Vector2.subtract(q2, q1, __vecTemp1).lengthSq();

        return point1LeftOfQ * point2LeftOfQ >= 0.0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && this.queryVisibilityRecursive(q1, q2, radius, node.left_) && this.queryVisibilityRecursive(q1, q2, radius, node.right_);
    }
}