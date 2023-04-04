/*
 * KdTree.cs
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

import { Vector2 } from "./Vector2";
import { Obstacle } from "./Obstacle";
import { Agent } from "./Agent";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";

/**
 * Defines a node of an agent k-D tree.
 */
class AgentTreeNode {
  public begin_: number;
  public end_: number;
  public left_: number;
  public right_: number;
  public maxX_: number;
  public maxY_: number;
  public minX_: number;
  public minY_: number;
}


/**
 * Defines a pair of scalar values.
 */
class FloatPair {
  /**
   * <summary>Constructs and initializes a pair of scalar
   * values.</summary>
   *
   * <param name="a">The first scalar value.</returns>
   * <param name="b">The second scalar value.</returns>
   */
  constructor(public a_: number, public b_: number) { }

  /**
   * <summary>Returns true if the first pair of scalar values is less
   * than the second pair of scalar values.</summary>
   *
   * <returns>True if the first pair of scalar values is less than the
   * second pair of scalar values.</returns>
   *
   * <param name="pair1">The first pair of scalar values.</param>
   * <param name="pair2">The second pair of scalar values.</param>
   */
  public static lessThan(pair1: FloatPair, pair2: FloatPair) {
    return pair1.a_ < pair2.a_ || !(pair2.a_ < pair1.a_) && pair1.b_ < pair2.b_;
  }

  /**
   * <summary>Returns true if the first pair of scalar values is less
   * than or equal to the second pair of scalar values.</summary>
   *
   * <returns>True if the first pair of scalar values is less than or
   * equal to the second pair of scalar values.</returns>
   *
   * <param name="pair1">The first pair of scalar values.</param>
   * <param name="pair2">The second pair of scalar values.</param>
   */
  public static lessThanEq(pair1: FloatPair, pair2: FloatPair) {
    return (pair1.a_ == pair2.a_ && pair1.b_ == pair2.b_) || FloatPair.lessThan(pair1, pair2);
  }

  /**
   * <summary>Returns true if the first pair of scalar values is
   * greater than the second pair of scalar values.</summary>
   *
   * <returns>True if the first pair of scalar values is greater than
   * the second pair of scalar values.</returns>
   *
   * <param name="pair1">The first pair of scalar values.</param>
   * <param name="pair2">The second pair of scalar values.</param>
   */
  public static greaterThan(pair1: FloatPair, pair2: FloatPair) {
    return !FloatPair.lessThanEq(pair1, pair2);
  }

  /**
   * <summary>Returns true if the first pair of scalar values is
   * greater than or equal to the second pair of scalar values.
   * </summary>
   *
   * <returns>True if the first pair of scalar values is greater than
   * or equal to the second pair of scalar values.</returns>
   *
   * <param name="pair1">The first pair of scalar values.</param>
   * <param name="pair2">The second pair of scalar values.</param>
   */
  public static greaterThanEq(pair1: FloatPair, pair2: FloatPair) {
    return !FloatPair.lessThan(pair1, pair2);
  }
}

/**
 * Defines a node of an obstacle k-D tree.
 */
class ObstacleTreeNode {
  public obstacle_: Obstacle;
  public left_: ObstacleTreeNode;
  public right_: ObstacleTreeNode;
};

/**
 * The maximum size of an agent k-D tree leaf.
 */
const MAX_LEAF_SIZE = 10;

/**
 * Defines k-D trees for agents and static obstacles in the simulation.
 */
export class KdTree {
  private agents_: Agent[] = [];
  private agentTree_: AgentTreeNode[] = [];
  private obstacleTree_: ObstacleTreeNode;

  public constructor(public readonly simulator: Simulator) { }

  /**
   * Builds an agent k-D tree.
   */
  public buildAgentTree() {
    if (this.agents_.length != this.simulator.agentCount) {
      this.simulator.forEachAgent((agentNo) => {
        this.agents_.push(this.simulator.agents_[agentNo]);
      });

      this.agentTree_ = new Array<AgentTreeNode>(2 * this.agents_.length);

      for (let i = 0; i < this.agentTree_.length; ++i) {
        this.agentTree_[i] = new AgentTreeNode();
      }
    }

    if (this.agents_.length > 0)
      this.buildAgentTreeRecursive(0, this.agents_.length, 0);
  }

  public delAgent(agent: Agent): void {
    const index = this.agents_.indexOf(agent);
    console.assert(index != -1);
    this.agents_.splice(index, 1);
    this.agentTree_.length = this.agents_.length * 2;
  }

  /**
   * Builds an obstacle k-D tree.
   */
  public buildObstacleTree() {
    this.obstacleTree_ = new ObstacleTreeNode();

    const obstacles: Obstacle[] = [];

    for (let i = 0; i < this.simulator.obstacles_.length; ++i) {
      obstacles.push(this.simulator.obstacles_[i]);
    }

    this.obstacleTree_ = this.buildObstacleTreeRecursive(obstacles);
  }

  /**
   * Computes the agent neighbors of the specified agent.
   * @param agent The agent for which agent neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   */
  public computeAgentNeighbors(agent: Agent, rangeSq: number): number {
    return this.queryAgentTreeRecursive(agent, rangeSq, 0);
  }

  /**
   * Computes the obstacle neighbors of the specified agent.
   * @param agent The agent for which obstacle neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   */
  public computeObstacleNeighbors(agent: Agent, rangeSq: number) {
    this.queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree_);
  }

  /**
   * Queries the visibility between two points within a specified radius.
   * @param q1 The first point between which visibility is to be tested.
   * @param q2 The second point between which visibility is to be tested.
   * @param radius The radius within which visibility is to be tested.
   * @returns True if q1 and q2 are mutually visible within the radius; false otherwise.
   */
  public queryVisibility(q1: Vector2, q2: Vector2, radius: number) {
    return this.queryVisibilityRecursive(q1, q2, radius, this.obstacleTree_);
  }

  /**
   * Recursive method for building an agent k-D tree.
   * @param begin The beginning agent k-D tree node node index.
   * @param end The ending agent k-D tree node index.
   * @param node The current agent k-D tree node index.
   */
  private buildAgentTreeRecursive(begin: number, end: number, node: number) {
    this.agentTree_[node].begin_ = begin;
    this.agentTree_[node].end_ = end;
    this.agentTree_[node].minX_ = this.agentTree_[node].maxX_ = this.agents_[begin].position_.x;
    this.agentTree_[node].minY_ = this.agentTree_[node].maxY_ = this.agents_[begin].position_.y;

    for (let i = begin + 1; i < end; ++i) {
      this.agentTree_[node].maxX_ = Math.max(this.agentTree_[node].maxX_, this.agents_[i].position_.x);
      this.agentTree_[node].minX_ = Math.min(this.agentTree_[node].minX_, this.agents_[i].position_.x);
      this.agentTree_[node].maxY_ = Math.max(this.agentTree_[node].maxY_, this.agents_[i].position_.y);
      this.agentTree_[node].minY_ = Math.min(this.agentTree_[node].minY_, this.agents_[i].position_.y);
    }

    if (end - begin > MAX_LEAF_SIZE) {
      /* No leaf node. */
      const isVertical = this.agentTree_[node].maxX_ - this.agentTree_[node].minX_ > this.agentTree_[node].maxY_ - this.agentTree_[node].minY_;
      const splitValue = 0.5 * (isVertical ? this.agentTree_[node].maxX_ + this.agentTree_[node].minX_ : this.agentTree_[node].maxY_ + this.agentTree_[node].minY_);

      let left = begin;
      let right = end;

      while (left < right) {
        while (left < right && (isVertical ? this.agents_[left].position_.x : this.agents_[left].position_.y) < splitValue) {
          ++left;
        }

        while (right > left && (isVertical ? this.agents_[right - 1].position_.x : this.agents_[right - 1].position_.y) >= splitValue) {
          --right;
        }

        if (left < right) {
          const tempAgent = this.agents_[left];
          this.agents_[left] = this.agents_[right - 1];
          this.agents_[right - 1] = tempAgent;
          ++left;
          --right;
        }
      }

      let leftSize = left - begin;

      if (leftSize == 0) {
        ++leftSize;
        ++left;
        ++right;
      }

      this.agentTree_[node].left_ = node + 1;
      this.agentTree_[node].right_ = node + 2 * leftSize;

      this.buildAgentTreeRecursive(begin, left, this.agentTree_[node].left_);
      this.buildAgentTreeRecursive(left, end, this.agentTree_[node].right_);
    }
  }

  /**
   * Recursive method for building an obstacle k-D tree.
   * @param obstacles A list of obstacles.
   * @returns An obstacle k-D tree node.
   */
  private buildObstacleTreeRecursive(obstacles: Obstacle[]): ObstacleTreeNode {
    if (obstacles.length === 0) {
      return null;
    }

    const node = new ObstacleTreeNode();

    let optimalSplit = 0;
    let minLeft = obstacles.length;
    let minRight = obstacles.length;

    for (let i = 0; i < obstacles.length; ++i) {
      let leftSize = 0;
      let rightSize = 0;

      const obstacleI1 = obstacles[i];
      const obstacleI2 = obstacleI1.next_;

      /* Compute optimal split node. */
      for (let j = 0; j < obstacles.length; ++j) {
        if (i == j) {
          continue;
        }

        const obstacleJ1 = obstacles[j];
        const obstacleJ2 = obstacleJ1.next_;

        const j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
        const j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

        if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
          ++leftSize;
        }
        else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
          ++rightSize;
        }
        else {
          ++leftSize;
          ++rightSize;
        }

        if (
          FloatPair.greaterThanEq(
            new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
            new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight))
          )
        ) {
          break;
        }
      }

      if (
        FloatPair.lessThan(
          new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)),
          new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight))
        )
      ) {
        minLeft = leftSize;
        minRight = rightSize;
        optimalSplit = i;
      }
    }

    {
      /* Build split node. */
      const leftObstacles: Obstacle[] = [];

      for (let n = 0; n < minLeft; ++n) {
        leftObstacles.push(null);
      }

      const rightObstacles: Obstacle[] = [];

      for (let n = 0; n < minRight; ++n) {
        rightObstacles.push(null);
      }

      let leftCounter = 0;
      let rightCounter = 0;
      let i = optimalSplit;

      const obstacleI1 = obstacles[i];
      const obstacleI2 = obstacleI1.next_;

      for (let j = 0; j < obstacles.length; ++j) {
        if (i == j) {
          continue;
        }

        const obstacleJ1 = obstacles[j];
        const obstacleJ2 = obstacleJ1.next_;

        const j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
        const j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

        if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
          leftObstacles[leftCounter++] = obstacles[j];
        }
        else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
          rightObstacles[rightCounter++] = obstacles[j];
        }
        else {
          /* Split obstacle j. */
          const t: number = RVOMath.det(
            Vector2.subtract(obstacleI2.point_, obstacleI1.point_),
            Vector2.subtract(obstacleJ1.point_, obstacleI1.point_)
          ) / RVOMath.det(
            Vector2.subtract(obstacleI2.point_, obstacleI1.point_),
            Vector2.subtract(obstacleJ1.point_, obstacleJ2.point_)
          );

          const splitPoint = Vector2.add(
            obstacleJ1.point_,
            Vector2.multiply(Vector2.subtract(obstacleJ2.point_, obstacleJ1.point_), t)
          );

          const newObstacle = new Obstacle();
          newObstacle.point_ = splitPoint;
          newObstacle.previous_ = obstacleJ1;
          newObstacle.next_ = obstacleJ2;
          newObstacle.convex_ = true;
          newObstacle.direction_ = obstacleJ1.direction_;

          newObstacle.id_ = obstacleJ1.id_;

          this.simulator.obstacles_.push(newObstacle);

          obstacleJ1.next_ = newObstacle;
          obstacleJ2.previous_ = newObstacle;

          if (j1LeftOfI > 0) {
            leftObstacles[leftCounter++] = obstacleJ1;
            rightObstacles[rightCounter++] = newObstacle;
          }
          else {
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

  /**
   * Recursive method for computing the agent neighbors of the specified agent.
   * @param agent The agent for which agent neighbors are to be computed.
   * @param rangeSq The squared range around the agent.
   * @param node The current agent k-D tree node index.
   */
  private queryAgentTreeRecursive(agent: Agent, rangeSq: number, node: number): number {
    if (this.agentTree_[node].end_ - this.agentTree_[node].begin_ <= MAX_LEAF_SIZE) {
      for (let i = this.agentTree_[node].begin_; i < this.agentTree_[node].end_; ++i) {
        rangeSq = agent.insertAgentNeighbor(this.agents_[i], rangeSq);
      }
    }
    else {
      const distSqLeft = RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].left_].minX_ - agent.position_.x)) + RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree_[this.agentTree_[node].left_].maxX_)) + RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].left_].minY_ - agent.position_.y)) + RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree_[this.agentTree_[node].left_].maxY_));
      const distSqRight = RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].right_].minX_ - agent.position_.x)) + RVOMath.sqr(Math.max(0, agent.position_.x - this.agentTree_[this.agentTree_[node].right_].maxX_)) + RVOMath.sqr(Math.max(0, this.agentTree_[this.agentTree_[node].right_].minY_ - agent.position_.y)) + RVOMath.sqr(Math.max(0, agent.position_.y - this.agentTree_[this.agentTree_[node].right_].maxY_));

      if (distSqLeft < distSqRight) {
        if (distSqLeft < rangeSq) {
          rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree_[node].left_);

          if (distSqRight < rangeSq) {
            rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree_[node].right_);
          }
        }
      }
      else {
        if (distSqRight < rangeSq) {
          rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree_[node].right_);

          if (distSqLeft < rangeSq) {
            rangeSq = this.queryAgentTreeRecursive(agent, rangeSq, this.agentTree_[node].left_);
          }
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
  private queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node?: ObstacleTreeNode) {
    if (node != null) {
      const obstacle1 = node.obstacle_;
      const obstacle2 = obstacle1.next_;

      const agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

      this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0 ? node.left_ : node.right_);

      const distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(Vector2.subtract(obstacle2.point_, obstacle1.point_));

      if (distSqLine < rangeSq) {
        if (agentLeftOfLine < 0) {
          /*
           * Try obstacle at this node only if agent is on right side of
           * obstacle (and can see obstacle).
           */
          agent.insertObstacleNeighbor(node.obstacle_, rangeSq);
        }

        /* Try other side of line. */
        this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0 ? node.right_ : node.left_);
      }
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
  private queryVisibilityRecursive(q1: Vector2, q2: Vector2, radius: number, node: ObstacleTreeNode): boolean {
    if (node == null) {
      return true;
    }

    const obstacle1 = node.obstacle_;
    const obstacle2 = obstacle1.next_;

    const q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
    const q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
    const invLengthI = 1 / RVOMath.absSq(Vector2.subtract(obstacle2.point_, obstacle1.point_));

    if (q1LeftOfI >= 0 && q2LeftOfI >= 0) {
      return this.queryVisibilityRecursive(q1, q2, radius, node.left_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.right_));
    }

    if (q1LeftOfI <= 0 && q2LeftOfI <= 0) {
      return this.queryVisibilityRecursive(q1, q2, radius, node.right_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.left_));
    }

    if (q1LeftOfI >= 0 && q2LeftOfI <= 0) {
      /* One can see through obstacle from left to right. */
      return this.queryVisibilityRecursive(q1, q2, radius, node.left_) && this.queryVisibilityRecursive(q1, q2, radius, node.right_);
    }

    const point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
    const point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
    const invLengthQ = 1.0 / RVOMath.absSq(Vector2.subtract(q2, q1));

    return point1LeftOfQ * point2LeftOfQ >= 0 && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && this.queryVisibilityRecursive(q1, q2, radius, node.left_) && this.queryVisibilityRecursive(q1, q2, radius, node.right_);
  }
}
