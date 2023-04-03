import { Agent } from "./Agent";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";
import { Vector2 } from "./Vector2";

type int = number;

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

class FloatPair {
    public constructor(private a: number, private b: number) { }

    public static Less(pair1: FloatPair, pair2: FloatPair): boolean {
        return pair1.a < pair2.a || !(pair2.a < pair1.a) && pair1.b < pair2.b;
    }

    public static LessEqual(pair1: FloatPair, pair2: FloatPair): boolean {
        return (pair1.a == pair2.a && pair1.b == pair2.b) || FloatPair.Less(pair1, pair2);
    }

    public static Greater(pair1: FloatPair, pair2: FloatPair): boolean {
        return !FloatPair.LessEqual(pair1, pair2);
    }

    public static GreaterEqual(pair1: FloatPair, pair2: FloatPair): boolean {
        return !FloatPair.Less(pair1, pair2);
    }
}

class ObstacleTreeNode {
    obstacle_: Obstacle;
    left_: ObstacleTreeNode;
    right_: ObstacleTreeNode;
};


export class KdTree {
    private static readonly MAX_LEAF_SIZE = 10;
    private agents_: Agent[];
    private agentTree_: AgentTreeNode[];
    private obstacleTree_: ObstacleTreeNode;

    public constructor(public readonly simulator: Simulator) { }

    buildAgentTree() {
        let agentsLength = this.simulator.agents_.length;
        if (this.agents_ == null || this.agents_.length != agentsLength) {
            this.agents_ = new Array<Agent>(agentsLength);

            for (let i = 0; i < agentsLength; ++i) {
                this.agents_[i] = this.simulator.agents_[i];
            }

            this.agentTree_ = new Array<AgentTreeNode>(2 * this.agents_.length);

            for (let i = 0; i < this.agentTree_.length; ++i) {
                this.agentTree_[i] = new AgentTreeNode();
            }
        }

        if (this.agents_.length != 0) {
            this.buildAgentTreeRecursive(0, this.agents_.length, 0);
        }
    }

    buildObstacleTree() {
        this.obstacleTree_ = new ObstacleTreeNode();

        let obstacles = new Array<Obstacle>(this.simulator.obstacles_.length);

        for (let i = 0; i < this.simulator.obstacles_.length; ++i) {
            obstacles[i] = this.simulator.obstacles_[i];
        }

        this.obstacleTree_ = this.buildObstacleTreeRecursive(obstacles);
    }

    computeAgentNeighbors(agent: Agent, rangeSq: number): number {
        return this.queryAgentTreeRecursive3(agent, rangeSq, 0);
    }

    computeObstacleNeighbors(agent: Agent, rangeSq: number): void {
        this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree_);
    }

    queryVisibility(q1: Vector2, q2: Vector2, radius: number): boolean {
        return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree_);
    }

    queryNearAgent(point: Vector2, radius: number): int {
        let result = this.queryAgentTreeRecursive4(point, Number.MAX_VALUE, -1, 0);
        if (result.rangeSq < radius * radius)
            return result.agentNo;
        return -1;
    }

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

                if (FloatPair.GreaterEqual(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
                    new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight)))) {
                    break;
                }
            }

            if (FloatPair.Less(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
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
            let vt = new Vector2();
            for (let i = treeNode.begin_; i < treeNode.end_; ++i) {
                let distSq = RVOMath.absSq(Vector2.subtract(position, this.agents_[i].position_, vt));
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

                    if (distSqRight < result.rangeSq) {
                        result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.right_);
                    }
                }
            } else {
                if (distSqRight < result.rangeSq) {
                    result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.right_);

                    if (distSqLeft < result.rangeSq) {
                        result = this.queryAgentTreeRecursive4(position, result.rangeSq, result.agentNo, treeNode.left_);
                    }
                }
            }

        }
        return result;
    }

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

                    if (distSqLeft < rangeSq) {
                        rangeSq = this.queryAgentTreeRecursive3(agent, rangeSq, treeNode.left_);
                    }
                }
            }

        }
        return rangeSq;
    }

    queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode): void {
        if (node == null) return;
        let obstacle1 = node.obstacle_;
        let obstacle2 = obstacle1.next_;

        let agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

        this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.left_ : node.right_);

        let distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(Vector2.subtract(obstacle2.point_, obstacle1.point_));

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

    queryVisibilityRecursive(q1: Vector2, q2: Vector2, radius: number, node: ObstacleTreeNode): boolean {
        if (node == null) return true;

        let vt = new Vector2();
        let obstacle1 = node.obstacle_;
        let obstacle2 = obstacle1.next_;

        let q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
        let q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
        let invLengthI = 1.0 / RVOMath.absSq(Vector2.subtract(obstacle2.point_, obstacle1.point_, vt));

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
        let invLengthQ = 1.0 / RVOMath.absSq(Vector2.subtract(q2, q1, vt));

        return point1LeftOfQ * point2LeftOfQ >= 0.0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
            && this.queryVisibilityRecursive(q1, q2, radius, node.left_) && this.queryVisibilityRecursive(q1, q2, radius, node.right_);
    }
}