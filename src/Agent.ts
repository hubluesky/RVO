import { AgentKdTree, ObstacleKdTree } from "./KdTree";
import { Line } from "./Line";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";
import { Vector2 } from "./Vector2";

const __vecTemp1 = new Vector2();
const __vecTemp2 = new Vector2();
const __vecTemp3 = new Vector2();
const __vecTemp4 = new Vector2();
const __vecTemp5 = new Vector2();
const __vecTemp6 = new Vector2();

class KeyValuePair<TKey, TValue> {
    public constructor(readonly key: TKey, readonly value: TValue) { }
}

/**
 * Defines an agent in the simulation.
 */
export class Agent {
    private readonly agentNeighbors = new Array<KeyValuePair<number, Agent>>();
    private readonly obstacleNeighbors = new Array<KeyValuePair<number, Obstacle>>();
    private readonly orcaLines = new Array<Line>();
    readonly position: Vector2 = new Vector2();
    readonly prefVelocity: Vector2 = new Vector2();
    readonly velocity: Vector2 = new Vector2();
    radius: number = 0;
    maxSpeed: number = 0;
    timeHorizon: number = 0;
    timeHorizonObst: number = 0;
    maxNeighbors: number = 0;
    avoidenceWeight: number = 0.5;
    isFreeze: boolean = false;
    userData: any;

    private newVelocity: Vector2 = new Vector2();

    public constructor(public readonly simulator: Simulator, public readonly id: number, public layer: number) { }

    /**
     * Computes the neighbors of this agent.
     */
    public computeNeighbors(kdTree: AgentKdTree, obstrcleTree: ObstacleKdTree): void {
        this.obstacleNeighbors.length = 0;
        let rangeSq = RVOMath.sqr(this.timeHorizonObst * this.maxSpeed + this.radius);
        obstrcleTree.computeObstacleNeighbors(this, rangeSq);

        this.agentNeighbors.length = 0;
        if (this.maxNeighbors > 0) {
            rangeSq = RVOMath.sqr(this.timeHorizon * this.maxSpeed + this.radius);
            kdTree.computeAgentNeighbors(this, rangeSq);
        }
    }

    private getAvoidenceWeight(otherAvoidenceWeight: number): number {
        return this.avoidenceWeight / (this.avoidenceWeight + otherAvoidenceWeight);
    }

    /**
     * Computes the new velocity of this agent.
     */
    public computeNewVelocity(timeStep: number): void {
        this.orcaLines.length = 0;

        const invTimeHorizonObst = 1.0 / this.timeHorizonObst;

        /* Create obstacle ORCA lines. */
        for (let i = 0; i < this.obstacleNeighbors.length; ++i) {

            let obstacle1 = this.obstacleNeighbors[i].value;
            let obstacle2 = obstacle1.next;

            const relativePosition1 = Vector2.subtract(obstacle1.point, this.position, __vecTemp1);
            const relativePosition2 = Vector2.subtract(obstacle2.point, this.position, __vecTemp2);

            /*
             * Check if velocity obstacle of obstacle is already taken care
             * of by previously constructed obstacle ORCA lines.
             */
            let alreadyCovered = false;

            for (let j = 0; j < this.orcaLines.length; ++j) {
                const orcaLine = this.orcaLines[j];
                if (Vector2.cross(Vector2.multiply(relativePosition1, invTimeHorizonObst, __vecTemp3).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVO_EPSILON &&
                    Vector2.cross(Vector2.multiply(relativePosition2, invTimeHorizonObst, __vecTemp4).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVO_EPSILON) {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
                continue;

            /* Not yet covered. Check for collisions. */
            const distSq1 = relativePosition1.lengthSq();
            const distSq2 = relativePosition2.lengthSq();

            const radiusSq = RVOMath.sqr(this.radius);

            const obstacleVector = Vector2.subtract(obstacle2.point, obstacle1.point, __vecTemp3);
            const rp1n = Vector2.negate(relativePosition1, __vecTemp4);
            const s = Vector2.dot(rp1n, obstacleVector) / obstacleVector.lengthSq();
            const distSqLine = rp1n.subtract(obstacleVector.multiply(s)).lengthSq();

            const line: Line = new Line();
            if (s < 0.0 && distSq1 <= radiusSq) {
                /* Collision with left vertex. Ignore if non-convex. */
                if (obstacle1.convex) {
                    line.point.reset();
                    line.direction.reset(-relativePosition1.y, relativePosition1.x).normalize();
                    this.orcaLines.push(line);
                }

                continue;
            } else if (s > 1.0 && distSq2 <= radiusSq) {
                /*
                 * Collision with right vertex. Ignore if non-convex or if
                 * it will be taken care of by neighboring obstacle.
                 */
                if (obstacle2.convex && Vector2.cross(relativePosition2, obstacle2.direction) >= 0.0) {
                    line.point.reset();
                    line.direction.reset(-relativePosition2.y, relativePosition2.x).normalize();
                    this.orcaLines.push(line);
                }

                continue;
            } else if (s >= 0.0 && s < 1.0 && distSqLine <= radiusSq) {
                /* Collision with obstacle segment. */
                line.point.reset();
                line.direction.set(obstacle1.direction).negate();
                this.orcaLines.push(line);
                continue;
            }

            /*
             * No collision. Compute legs. When obliquely viewed, both legs
             * can come from a single vertex. Legs extend cut-off line when
             * non-convex vertex.
             */

            const leftLegDirection: Vector2 = __vecTemp3, rightLegDirection: Vector2 = __vecTemp4;

            if (s < 0.0 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that left vertex
                 * defines velocity obstacle.
                 */
                if (!obstacle1.convex) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle2 = obstacle1;

                const leg1 = Math.sqrt(distSq1 - radiusSq);
                leftLegDirection.reset(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1);
                leftLegDirection.divide(distSq1);
                rightLegDirection.reset(relativePosition1.x * leg1 + relativePosition1.y * this.radius, -relativePosition1.x * this.radius + relativePosition1.y * leg1);
                rightLegDirection.divide(distSq1);
            } else if (s > 1.0 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that
                 * right vertex defines velocity obstacle.
                 */
                if (!obstacle2.convex) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle1 = obstacle2;

                const leg2 = Math.sqrt(distSq2 - radiusSq);
                leftLegDirection.reset(relativePosition2.x * leg2 - relativePosition2.y * this.radius, relativePosition2.x * this.radius + relativePosition2.y * leg2);
                leftLegDirection.divide(distSq2);
                rightLegDirection.reset(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2);
                rightLegDirection.divide(distSq2);
            } else {
                /* Usual situation. */
                if (obstacle1.convex) {
                    const leg1 = Math.sqrt(distSq1 - radiusSq);
                    leftLegDirection.reset(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1);
                    leftLegDirection.divide(distSq1);
                } else {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    leftLegDirection.set(obstacle1.direction).negate();
                }

                if (obstacle2.convex) {
                    const leg2 = Math.sqrt(distSq2 - radiusSq);
                    rightLegDirection.reset(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2);
                    rightLegDirection.divide(distSq2);
                } else {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    rightLegDirection.set(obstacle1.direction);
                }
            }

            /*
             * Legs can never point into neighboring edge when convex
             * vertex, take cutoff-line of neighboring edge instead. If
             * velocity projected on "foreign" leg, no constraint is added.
             */

            const leftNeighbor = obstacle1.previous;

            let isLeftLegForeign = false;
            let isRightLegForeign = false;

            const leftNeighborNegate = Vector2.negate(leftNeighbor.direction, __vecTemp1);
            if (obstacle1.convex && Vector2.cross(leftLegDirection, leftNeighborNegate) >= 0.0) {
                /* Left leg points into obstacle. */
                leftLegDirection.set(leftNeighborNegate);
                isLeftLegForeign = true;
            }

            if (obstacle2.convex && Vector2.cross(rightLegDirection, obstacle2.direction) <= 0.0) {
                /* Right leg points into obstacle. */
                rightLegDirection.set(obstacle2.direction);
                isRightLegForeign = true;
            }

            /* Compute cut-off centers. */
            const leftCutOff = Vector2.subtract(obstacle1.point, this.position, __vecTemp1).multiply(invTimeHorizonObst);
            const rightCutOff = Vector2.subtract(obstacle2.point, this.position, __vecTemp2).multiply(invTimeHorizonObst);
            const cutOffVector = Vector2.subtract(rightCutOff, leftCutOff, __vecTemp5);

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            const vslc = Vector2.subtract(this.velocity, leftCutOff, __vecTemp6);
            const t = obstacle1 == obstacle2 ? 0.5 : Vector2.dot(vslc, cutOffVector) / cutOffVector.lengthSq();
            const tLeft = Vector2.dot(vslc, leftLegDirection);
            const tRight = Vector2.dot(vslc, rightLegDirection);

            if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0)) {
                /* Project on left cut-off circle. */
                const unitW = vslc.normalize();

                line.direction.reset(unitW.y, -unitW.x);
                line.point = unitW.multiply(this.radius * invTimeHorizonObst).add(leftCutOff);
                this.orcaLines.push(line);

                continue;
            } else if (t > 1.0 && tRight < 0.0) {
                /* Project on right cut-off circle. */
                const vsrc = Vector2.subtract(this.velocity, rightCutOff, __vecTemp6);
                const unitW = vsrc.normalize();

                line.direction.reset(unitW.y, -unitW.x);
                line.point = unitW.multiply(this.radius * invTimeHorizonObst).add(rightCutOff);
                this.orcaLines.push(line);

                continue;
            }

            /*
             * Project on left leg, right leg, or cut-off line, whichever is
             * closest to velocity.
             */
            const distSqCutoff = (t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? Number.MAX_VALUE : Vector2.subtract(this.velocity, cutOffVector.multiply(t).add(leftCutOff), __vecTemp5).lengthSq();
            const distSqLeft = tLeft < 0.0 ? Number.MAX_VALUE : Vector2.subtract(this.velocity, Vector2.multiply(leftLegDirection, tLeft, __vecTemp5).add(leftCutOff), __vecTemp5).lengthSq();
            const distSqRight = tRight < 0.0 ? Number.MAX_VALUE : Vector2.subtract(this.velocity, Vector2.multiply(rightLegDirection, tRight, __vecTemp5).add(rightCutOff), __vecTemp5).lengthSq();

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
                /* Project on cut-off line. */
                line.direction.set(obstacle1.direction).negate();
                line.point.reset(-line.direction.y, line.direction.x).multiply(this.radius * invTimeHorizonObst).add(leftCutOff);
                this.orcaLines.push(line);
                continue;
            }

            if (distSqLeft <= distSqRight) {
                /* Project on left leg. */
                if (isLeftLegForeign)
                    continue;

                line.direction = leftLegDirection;
                line.point.reset(-line.direction.y, line.direction.x).multiply(this.radius * invTimeHorizonObst).add(leftCutOff);
                this.orcaLines.push(line);
                continue;
            }

            /* Project on right leg. */
            if (isRightLegForeign)
                continue;

            line.direction.set(rightLegDirection).negate();
            line.point.reset(-line.direction.y, line.direction.x).multiply(this.radius * invTimeHorizonObst).add(rightCutOff);
            this.orcaLines.push(line);
        }


        const numObstLines = this.orcaLines.length;
        const invTimeHorizon = 1.0 / this.timeHorizon;

        /* Create agent ORCA lines. */
        for (let i = 0; i < this.agentNeighbors.length; ++i) {
            const other = this.agentNeighbors[i].value;

            const relativePosition = Vector2.subtract(other.position, this.position, __vecTemp1);
            const relativeVelocity = Vector2.subtract(this.velocity, other.velocity, __vecTemp2);
            const distSq = relativePosition.lengthSq();
            const combinedRadius = this.radius + other.radius;
            const combinedRadiusSq = RVOMath.sqr(combinedRadius);

            const line: Line = new Line();
            let u: Vector2;

            if (distSq > combinedRadiusSq) {
                /* No collision. */
                const w: Vector2 = Vector2.subtract(relativeVelocity, Vector2.multiply(relativePosition, invTimeHorizon, __vecTemp3), __vecTemp3);

                /* Vector from cutoff center to relative velocity. */
                const wLengthSq = w.lengthSq();
                const dotProduct1 = Vector2.dot(w, relativePosition);

                if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                    /* Project on cut-off circle. */
                    if (wLengthSq == 0) continue;
                    const wLength = Math.sqrt(wLengthSq);
                    const unitW = w.divide(wLength);

                    line.direction.reset(unitW.y, -unitW.x);
                    u = Vector2.multiply(unitW, combinedRadius * invTimeHorizon - wLength, __vecTemp3);
                } else {
                    /* Project on legs. */
                    const leg = Math.sqrt(distSq - combinedRadiusSq);

                    if (Vector2.cross(relativePosition, w) > 0.0) {
                        /* Project on left leg. */
                        line.direction.reset(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg);

                    } else {
                        /* Project on right leg. */
                        line.direction.reset(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg).negate();
                    }

                    line.direction = Vector2.divide(line.direction, distSq, line.direction);

                    const dotProduct2 = Vector2.dot(relativeVelocity, line.direction);
                    u = Vector2.multiply(line.direction, dotProduct2, __vecTemp3).subtract(relativeVelocity);
                }
            } else {
                /* Collision. Project on cut-off circle of time timeStep. */
                const invTimeStep = 1.0 / timeStep;

                /* Vector from cutoff center to relative velocity. */
                const w = Vector2.subtract(relativeVelocity, Vector2.multiply(relativePosition, invTimeStep, __vecTemp3), __vecTemp3);
                const wLength = w.length();
                if (wLength == 0) continue;
                const unitW = w.divide(wLength);

                line.direction.reset(unitW.y, -unitW.x);
                u = Vector2.multiply(unitW, combinedRadius * invTimeStep - wLength, __vecTemp3);
            }

            // Vector2.add(this.velocity, u.multiply(0.5), line.point);
            const avoidenceWeight = other.isFreeze ? 1 : this.getAvoidenceWeight(other.avoidenceWeight);
            Vector2.add(this.velocity, u.multiply(avoidenceWeight), line.point);
            this.orcaLines.push(line);
        }

        const lineFail = this.linearProgram2(this.orcaLines, this.maxSpeed, this.prefVelocity, false);
        this.newVelocity.set(lineFail.result);
        if (lineFail.count < this.orcaLines.length) {
            this.newVelocity = this.linearProgram3(this.orcaLines, numObstLines, lineFail.count, this.maxSpeed, this.newVelocity);
        }
    }

    /**
     * Inserts an agent neighbor into the set of neighbors of this agent.
     * @param agent A pointer to the agent to be inserted.
     * @param rangeSq The squared range around this agent.
     */
    public insertAgentNeighbor(agent: Agent, rangeSq: number): number {
        if (this == agent) return rangeSq;
        if (!this.simulator.checkLayerMask(this.layer, agent.layer)) return rangeSq;

        const distSq = Vector2.distanceSq(this.position, agent.position);
        const radiusSq = RVOMath.sqr(this.radius + agent.radius);
        if (distSq - radiusSq >= rangeSq) return rangeSq;

        if (this.agentNeighbors.length < this.maxNeighbors) {
            this.agentNeighbors.push(new KeyValuePair(distSq, agent));
        }

        let i = this.agentNeighbors.length - 1;

        while (i != 0 && distSq < this.agentNeighbors[i - 1].key) {
            this.agentNeighbors[i] = this.agentNeighbors[i - 1];
            --i;
        }

        this.agentNeighbors[i] = new KeyValuePair(distSq, agent);

        if (this.agentNeighbors.length == this.maxNeighbors) {
            rangeSq = this.agentNeighbors[this.agentNeighbors.length - 1].key;
        }
        return rangeSq;
    }

    /**
     * Inserts a static obstacle neighbor into the set of neighbors of this agent.
     * @param obstacle The number of the static obstacle to be inserted.
     * @param rangeSq The squared range around this agent.
     */
    public insertObstacleNeighbor(obstacle: Obstacle, rangeSq: number): number {
        if (!this.simulator.checkLayerMask(this.layer, obstacle.layer)) return rangeSq;

        const nextObstacle = obstacle.next;

        const distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, this.position);

        if (distSq < rangeSq) {
            this.obstacleNeighbors.push(new KeyValuePair(distSq, obstacle));

            let i = this.obstacleNeighbors.length - 1;

            while (i != 0 && distSq < this.obstacleNeighbors[i - 1].key) {
                this.obstacleNeighbors[i] = this.obstacleNeighbors[i - 1];
                --i;
            }
            this.obstacleNeighbors[i] = new KeyValuePair(distSq, obstacle);
        }
        return rangeSq;
    }

    /**
     * Updates the two-dimensional position and two-dimensional velocity of this agent.
     * @param timeStep The time step of the simulation. Must be positive.
     */
    public update(timeStep: number) {
        this.velocity.set(this.newVelocity);
        this.position.add(Vector2.multiply(this.velocity, timeStep, __vecTemp1));
        this.prefVelocity.reset();
    }

    /**
     * Solves a one-dimensional linear program on a specified line
     * subject to linear constraints defined by lines and a circular
     * constraint.
     * @param lines Lines defining the linear constraints.
     * @param lineNo The specified line constraint.
     * @param radius The radius of the circular constraint.
     * @param optVelocity The optimization velocity.
     * @param directionOpt True if the direction should be optimized.
     * @param result A reference to the result of the linear program.
     * @returns velocity if successful.
     */
    private linearProgram1(lines: Array<Line>, lineNo: number, radius: number, optVelocity: Vector2, directionOpt: boolean): Vector2 {
        const lineNoDirection = lines[lineNo].direction, lineNoPoint = lines[lineNo].point;
        const dotProduct = Vector2.dot(lineNoPoint, lineNoDirection);
        const discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - lineNoPoint.lengthSq();

        /* Max speed circle fully invalidates line lineNo. */
        if (discriminant < 0.0)
            return null;

        const sqrtDiscriminant = Math.sqrt(discriminant);
        let tLeft = -dotProduct - sqrtDiscriminant;
        let tRight = -dotProduct + sqrtDiscriminant;

        for (let i = 0; i < lineNo; ++i) {
            const denominator = Vector2.cross(lineNoDirection, lines[i].direction);
            const numerator = Vector2.cross(lines[i].direction, Vector2.subtract(lineNoPoint, lines[i].point, __vecTemp1));

            if (Math.abs(denominator) <= RVOMath.RVO_EPSILON) {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0)
                    return null;

                continue;
            }

            const t = numerator / denominator;
            if (denominator >= 0.0) {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.min(tRight, t);
            } else {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.max(tLeft, t);
            }

            if (tLeft > tRight)
                return null;
        }

        let vector: Vector2;
        if (directionOpt) {
            /* Optimize direction. */
            if (Vector2.dot(optVelocity, lineNoDirection) > 0.0) {
                /* Take right extreme. */
                vector = Vector2.multiply(lineNoDirection, tRight, __vecTemp1).add(lineNoPoint);
            } else {
                /* Take left extreme. */
                vector = Vector2.multiply(lineNoDirection, tLeft, __vecTemp1).add(lineNoPoint);
            }
        } else {
            /* Optimize closest point. */
            let t = Vector2.dot(lineNoDirection, Vector2.subtract(optVelocity, lineNoPoint, __vecTemp1));
            // console.assert(tLeft <= 0 && tRight >= 0, tLeft, tRight, t);
            if (t < tLeft) {
                vector = Vector2.multiply(lineNoDirection, tLeft, __vecTemp1).add(lineNoPoint);
            } else if (t > tRight) {
                vector = Vector2.multiply(lineNoDirection, tRight, __vecTemp1).add(lineNoPoint);
            } else {
                // const tCenter = (tLeft + tRight) * 0.5;
                // const tMin = (tRight - tCenter) * 0.5;
                // if (t > tCenter - tMin && t < tCenter + tMin)
                //     t = Vector2.dot(optVelocity, lineNoDirection) > 0.0 ? tMin : -tMin;
                // t = t > 0 ? t * Math.abs(tRight) : t * Math.abs(tLeft);
                // 修复，当t为0时，会导致移动速度为0，角色卡死。
                if (t == 0) t = RVOMath.RVO_EPSILON;
                vector = Vector2.multiply(lineNoDirection, t, __vecTemp1).add(lineNoPoint);
            }
        }
        return vector;
    }

    /**
     * Solves a two-dimensional linear program subject to linear
     * constraints defined by lines and a circular constraint.
     * @param lines Lines defining the linear constraints.
     * @param radius The radius of the circular constraint.
     * @param optVelocity The optimization velocity.
     * @param directionOpt True if the direction should be optimized.
     * @returns The number of the line it fails on, and the number of lines if successful.
     * A reference to the result of the linear program.
     */
    private linearProgram2(lines: Array<Line>, radius: number, optVelocity: Vector2, directionOpt: boolean): { count: number, result: Vector2 } {
        let result: Vector2;
        if (directionOpt) {
            /*
             * Optimize direction. Note that the optimization velocity is of
             * unit length in this case.
             */
            result = Vector2.multiply(optVelocity, radius, __vecTemp5);
        } else if (optVelocity.lengthSq() > RVOMath.sqr(radius)) {
            /* Optimize closest point and outside circle. */
            result = Vector2.normalize(optVelocity, __vecTemp5).multiply(radius);
        } else {
            /* Optimize closest point and inside circle. */
            result = __vecTemp5.set(optVelocity);
        }

        for (let i = 0; i < lines.length; ++i) {
            const lpr = Vector2.subtract(lines[i].point, result, __vecTemp6);
            if (Vector2.cross(lines[i].direction, lpr) > 0.0) {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                // const tempResult = result;
                const tempResult = this.linearProgram1(lines, i, radius, optVelocity, directionOpt);
                if (tempResult == null)
                    return { count: i, result };
                result.set(tempResult);
            }
        }

        return { count: lines.length, result };
    }

    /**
     * Solves a two-dimensional linear program subject to linear
     * constraints defined by lines and a circular constraint.
     * @param lines Lines defining the linear constraints.
     * @param numObstLines Count of obstacle lines.
     * @param beginLine The line on which the 2-d linear program failed.
     * @param radius The radius of the circular constraint.
     * @param result A reference to the result of the linear program.
     */
    private linearProgram3(lines: Array<Line>, numObstLines: number, beginLine: number, radius: number, result: Vector2): Vector2 {
        let distance = 0.0;

        for (let i = beginLine; i < lines.length; ++i) {
            if (Vector2.cross(lines[i].direction, Vector2.subtract(lines[i].point, result, __vecTemp4)) > distance) {
                /* Result does not satisfy constraint of line i. */
                const projLines = new Array<Line>();
                for (let ii = 0; ii < numObstLines; ++ii) {
                    projLines.push(lines[ii]);
                }

                for (let j = numObstLines; j < i; ++j) {
                    const line: Line = new Line();

                    const determinant = Vector2.cross(lines[i].direction, lines[j].direction);

                    if (Math.abs(determinant) <= RVOMath.RVO_EPSILON) {
                        /* Line i and line j are parallel. */
                        if (Vector2.dot(lines[i].direction, lines[j].direction) > 0.0) {
                            /* Line i and line j point in the same direction. */
                            continue;
                        } else {
                            /* Line i and line j point in opposite direction. */
                            Vector2.add(lines[i].point, lines[j].point, line.point).multiply(0.5);
                        }
                    } else {
                        Vector2.add(lines[i].point, Vector2.multiply(lines[i].direction, Vector2.cross(lines[j].direction, Vector2.subtract(lines[i].point, lines[j].point, line.point)) / determinant, line.point), line.point);
                    }

                    Vector2.subtract(lines[j].direction, lines[i].direction, line.direction).normalize();
                    projLines.push(line);
                }

                const results = this.linearProgram2(projLines, radius, __vecTemp4.reset(-lines[i].direction.y, lines[i].direction.x), true);
                if (results.count >= projLines.length) {
                    /*
                     * This should in principle not happen. The result is by
                     * definition already in the feasible region of this
                     * linear program. If it fails, it is due to small
                     * floating point error, and the current result is kept.
                     */
                    result.set(results.result);
                }

                distance = Vector2.cross(lines[i].direction, Vector2.subtract(lines[i].point, result, __vecTemp4));
            }
        }
        return result;
    }
}