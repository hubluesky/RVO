import { Line } from "./Line";
import { Obstacle } from "./Obstacle";
import { RVOMath } from "./RVOMath";
import { Simulator } from "./Simulator";
import { Vector2 } from "./Vector2";

const __vecTemp1 = new Vector2();
const __vecTemp2 = new Vector2();
const __vecTemp3 = new Vector2();
const __vecTemp4 = new Vector2();
type int = number;

class KeyValuePair<TKey, TValue> {
    public constructor(readonly key: TKey, readonly value: TValue) { }
}

/**
 * Defines an agent in the simulation.
 */
export class Agent {
    readonly agentNeighbors_ = new Array<KeyValuePair<number, Agent>>();
    readonly obstacleNeighbors_ = new Array<KeyValuePair<number, Obstacle>>();
    readonly orcaLines_ = new Array<Line>();
    position_: Vector2 = new Vector2();
    prefVelocity_: Vector2 = new Vector2();
    velocity_: Vector2 = new Vector2();
    id_: int = 0;
    maxNeighbors_: int = 0;
    maxSpeed_: number = 0;
    neighborDist_: number = 0;
    radius_: number = 0;
    timeHorizon_: number = 0;
    timeHorizonObst_: number = 0;
    needDelete_: boolean = false;
    isFreeze: boolean = false;

    private newVelocity_: Vector2 = new Vector2();

    public constructor(public readonly simulator: Simulator) { }

    /**
     * Computes the neighbors of this agent.
     */
    computeNeighbors(): void {
        this.obstacleNeighbors_.length = 0;
        let rangeSq = RVOMath.sqr(this.timeHorizonObst_ * this.maxSpeed_ + this.radius_);
        this.simulator.kdTree_.computeObstacleNeighbors(this, rangeSq);

        this.agentNeighbors_.length = 0;

        if (this.maxNeighbors_ > 0) {
            rangeSq = RVOMath.sqr(this.neighborDist_);
            this.simulator.kdTree_.computeAgentNeighbors(this, rangeSq);
        }
    }

    /**
     * Computes the new velocity of this agent.
     */
    computeNewVelocity(): void {
        this.orcaLines_.length = 0;

        let invTimeHorizonObst = 1.0 / this.timeHorizonObst_;

        /* Create obstacle ORCA lines. */
        for (let i = 0; i < this.obstacleNeighbors_.length; ++i) {

            let obstacle1 = this.obstacleNeighbors_[i].value;
            let obstacle2 = obstacle1.next_;

            let relativePosition1 = Vector2.subtract(obstacle1.point_, this.position_, __vecTemp1);
            let relativePosition2 = Vector2.subtract(obstacle2.point_, this.position_, __vecTemp2);

            /*
             * Check if velocity obstacle of obstacle is already taken care
             * of by previously constructed obstacle ORCA lines.
             */
            let alreadyCovered = false;

            for (let j = 0; j < this.orcaLines_.length; ++j) {
                let orcaLine = this.orcaLines_[j];
                let rp1i = Vector2.multiply(relativePosition1, invTimeHorizonObst, __vecTemp3);
                let rp2i = Vector2.multiply(relativePosition2, invTimeHorizonObst, __vecTemp4);
                if (Vector2.cross(Vector2.subtract(rp1i, orcaLine.point, rp1i), orcaLine.direction) - invTimeHorizonObst * this.radius_
                    >= -RVOMath.RVO_EPSILON && Vector2.cross(Vector2.subtract(rp2i, orcaLine.point, rp2i), orcaLine.direction) - invTimeHorizonObst * this.radius_
                    >= -RVOMath.RVO_EPSILON) {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
                continue;

            /* Not yet covered. Check for collisions. */
            let distSq1 = relativePosition1.lengthSq();
            let distSq2 = relativePosition2.lengthSq();

            let radiusSq = RVOMath.sqr(this.radius_);

            let obstacleVector = Vector2.subtract(obstacle2.point_, obstacle1.point_, __vecTemp3);
            let rp1n = Vector2.negate(relativePosition1, __vecTemp4);
            let s = Vector2.dot(rp1n, obstacleVector) / obstacleVector.lengthSq();
            let distSqLine = Vector2.subtract(rp1n, Vector2.multiply(obstacleVector, s, obstacleVector), rp1n).lengthSq();

            let line: Line = new Line();
            if (s < 0.0 && distSq1 <= radiusSq) {
                /* Collision with left vertex. Ignore if non-convex. */
                if (obstacle1.convex_) {
                    line.point.reset();
                    line.direction.reset(-relativePosition1.y, relativePosition1.x).normalize();
                    this.orcaLines_.push(line);
                }

                continue;
            } else if (s > 1.0 && distSq2 <= radiusSq) {
                /*
                 * Collision with right vertex. Ignore if non-convex or if
                 * it will be taken care of by neighboring obstacle.
                 */
                if (obstacle2.convex_ && Vector2.cross(relativePosition2, obstacle2.direction_) >= 0.0) {
                    line.point.reset();
                    line.direction.reset(-relativePosition2.y, relativePosition2.x).normalize();
                    this.orcaLines_.push(line);
                }

                continue;
            } else if (s >= 0.0 && s < 1.0 && distSqLine <= radiusSq) {
                /* Collision with obstacle segment. */
                line.point.reset();
                line.direction.set(obstacle1.direction_).negate();
                this.orcaLines_.push(line);
                continue;
            }

            /*
             * No collision. Compute legs. When obliquely viewed, both legs
             * can come from a single vertex. Legs extend cut-off line when
             * non-convex vertex.
             */

            const leftLegDirection: Vector2 = new Vector2(), rightLegDirection: Vector2 = new Vector2();

            if (s < 0.0 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that left vertex
                 * defines velocity obstacle.
                 */
                if (!obstacle1.convex_) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle2 = obstacle1;

                let leg1 = Math.sqrt(distSq1 - radiusSq);
                leftLegDirection.reset(relativePosition1.x * leg1 - relativePosition1.y * this.radius_, relativePosition1.x * this.radius_ + relativePosition1.y * leg1);
                leftLegDirection.divide(distSq1);
                rightLegDirection.reset(relativePosition1.x * leg1 + relativePosition1.y * this.radius_, -relativePosition1.x * this.radius_ + relativePosition1.y * leg1);
                rightLegDirection.divide(distSq1);
            } else if (s > 1.0 && distSqLine <= radiusSq) {
                /*
                 * Obstacle viewed obliquely so that
                 * right vertex defines velocity obstacle.
                 */
                if (!obstacle2.convex_) {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle1 = obstacle2;

                let leg2 = Math.sqrt(distSq2 - radiusSq);
                leftLegDirection.reset(relativePosition2.x * leg2 - relativePosition2.y * this.radius_, relativePosition2.x * this.radius_ + relativePosition2.y * leg2);
                leftLegDirection.divide(distSq2);
                rightLegDirection.reset(relativePosition2.x * leg2 + relativePosition2.y * this.radius_, -relativePosition2.x * this.radius_ + relativePosition2.y * leg2);
                rightLegDirection.divide(distSq2);
            } else {
                /* Usual situation. */
                if (obstacle1.convex_) {
                    let leg1 = Math.sqrt(distSq1 - radiusSq);
                    leftLegDirection.reset(relativePosition1.x * leg1 - relativePosition1.y * this.radius_, relativePosition1.x * this.radius_ + relativePosition1.y * leg1);
                    leftLegDirection.divide(distSq1);
                } else {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    leftLegDirection.set(obstacle1.direction_).negate();
                }

                if (obstacle2.convex_) {
                    let leg2 = Math.sqrt(distSq2 - radiusSq);
                    rightLegDirection.reset(relativePosition2.x * leg2 + relativePosition2.y * this.radius_, -relativePosition2.x * this.radius_ + relativePosition2.y * leg2);
                    rightLegDirection.divide(distSq2);
                } else {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    rightLegDirection.set(obstacle1.direction_);
                }
            }

            /*
             * Legs can never point into neighboring edge when convex
             * vertex, take cutoff-line of neighboring edge instead. If
             * velocity projected on "foreign" leg, no constraint is added.
             */

            let leftNeighbor = obstacle1.previous_;

            let isLeftLegForeign = false;
            let isRightLegForeign = false;

            let leftNeighborNegate = Vector2.negate(leftNeighbor.direction_);
            if (obstacle1.convex_ && Vector2.cross(leftLegDirection, leftNeighborNegate) >= 0.0) {
                /* Left leg points into obstacle. */
                leftLegDirection.set(leftNeighborNegate);
                isLeftLegForeign = true;
            }

            if (obstacle2.convex_ && Vector2.cross(rightLegDirection, obstacle2.direction_) <= 0.0) {
                /* Right leg points into obstacle. */
                rightLegDirection.set(obstacle2.direction_);
                isRightLegForeign = true;
            }

            /* Compute cut-off centers. */
            let leftCutOff = Vector2.subtract(obstacle1.point_, this.position_);
            leftCutOff = Vector2.multiply(leftCutOff, invTimeHorizonObst, leftCutOff);
            let rightCutOff = Vector2.subtract(obstacle2.point_, this.position_);
            rightCutOff = Vector2.multiply(rightCutOff, invTimeHorizonObst, rightCutOff);
            let cutOffVector = Vector2.subtract(rightCutOff, leftCutOff);

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            let vslc = Vector2.subtract(this.velocity_, leftCutOff);
            let t = obstacle1 == obstacle2 ? 0.5 : Vector2.dot(vslc, cutOffVector) / cutOffVector.lengthSq();
            let tLeft = Vector2.dot(vslc, leftLegDirection);
            let tRight = Vector2.dot(vslc, rightLegDirection);

            if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0)) {
                /* Project on left cut-off circle. */
                let unitW = vslc.normalize();

                line.direction = new Vector2(unitW.y, -unitW.x);
                let vw = Vector2.multiply(unitW, this.radius_ * invTimeHorizonObst, unitW);
                line.point = Vector2.add(leftCutOff, vw);
                this.orcaLines_.push(line);

                continue;
            } else if (t > 1.0 && tRight < 0.0) {
                /* Project on right cut-off circle. */
                let vsrc = Vector2.subtract(this.velocity_, rightCutOff);
                let unitW = vsrc.normalize();

                line.direction = new Vector2(unitW.y, -unitW.x);
                let vw = Vector2.multiply(unitW, this.radius_ * invTimeHorizonObst, unitW);
                line.point = Vector2.add(rightCutOff, vw);
                this.orcaLines_.push(line);

                continue;
            }

            /*
             * Project on left leg, right leg, or cut-off line, whichever is
             * closest to velocity.
             */
            let vt10 = Vector2.multiply(cutOffVector, t);
            vt10 = Vector2.add(leftCutOff, vt10, vt10);
            vt10 = Vector2.subtract(this.velocity_, vt10, vt10);

            let distSqCutoff = (t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? Number.MAX_VALUE : vt10.lengthSq();

            let vt11 = Vector2.multiply(leftLegDirection, tLeft, vt10);
            vt11 = Vector2.add(leftCutOff, vt11, vt10);
            vt11 = Vector2.subtract(this.velocity_, vt11, vt10);

            let distSqLeft = tLeft < 0.0 ? Number.MAX_VALUE : vt11.lengthSq();

            let vt12 = Vector2.multiply(rightLegDirection, tRight, vt10);
            vt12 = Vector2.add(rightCutOff, vt12, vt10);
            vt12 = Vector2.subtract(this.velocity_, vt12, vt10);

            let distSqRight = tRight < 0.0 ? Number.MAX_VALUE : vt12.lengthSq();

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
                /* Project on cut-off line. */
                line.direction = Vector2.negate(obstacle1.direction_);
                let point = new Vector2(-line.direction.y, line.direction.x);
                point = Vector2.multiply(point, this.radius_ * invTimeHorizonObst, point);
                line.point = Vector2.add(leftCutOff, point, point);
                this.orcaLines_.push(line);

                continue;
            }

            if (distSqLeft <= distSqRight) {
                /* Project on left leg. */
                if (isLeftLegForeign)
                    continue;

                line.direction = leftLegDirection;
                let point = new Vector2(-line.direction.y, line.direction.x);
                point = Vector2.multiply(point, this.radius_ * invTimeHorizonObst, point);
                line.point = Vector2.add(leftCutOff, point, point);
                this.orcaLines_.push(line);

                continue;
            }

            /* Project on right leg. */
            if (isRightLegForeign)
                continue;

            line.direction = Vector2.negate(rightLegDirection);
            let point = new Vector2(-line.direction.y, line.direction.x);
            point = Vector2.multiply(point, this.radius_ * invTimeHorizonObst, point);
            line.point = Vector2.add(rightCutOff, point, point);
            this.orcaLines_.push(line);
        }


        let numObstLines = this.orcaLines_.length;

        let invTimeHorizon = 1.0 / this.timeHorizon_;

        /* Create agent ORCA lines. */
        for (let i = 0; i < this.agentNeighbors_.length; ++i) {
            let other = this.agentNeighbors_[i].value;

            let relativePosition = Vector2.subtract(other.position_, this.position_, __vecTemp1);
            let relativeVelocity = Vector2.subtract(this.velocity_, other.velocity_, __vecTemp2);
            let distSq = relativePosition.lengthSq();
            let combinedRadius = this.radius_ + other.radius_;
            let combinedRadiusSq = RVOMath.sqr(combinedRadius);

            let line: Line = new Line();
            let u: Vector2;

            if (distSq > combinedRadiusSq) {
                /* No collision. */
                let rpi = Vector2.multiply(relativePosition, invTimeHorizon, __vecTemp3);
                let w: Vector2 = Vector2.subtract(relativeVelocity, rpi, rpi);

                /* Vector from cutoff center to relative velocity. */
                let wLengthSq = w.lengthSq();
                let dotProduct1 = Vector2.dot(w, relativePosition);

                if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                    /* Project on cut-off circle. */
                    let wLength = Math.sqrt(wLengthSq);
                    let unitW = Vector2.divide(w, wLength);

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    u = Vector2.multiply(unitW, combinedRadius * invTimeHorizon - wLength);
                } else {
                    /* Project on legs. */
                    let leg = Math.sqrt(distSq - combinedRadiusSq);

                    if (Vector2.cross(relativePosition, w) > 0.0) {
                        /* Project on left leg. */
                        line.direction = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg);

                    } else {
                        /* Project on right leg. */
                        line.direction = new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg);
                        line.direction = Vector2.negate(line.direction);
                    }

                    line.direction = Vector2.divide(line.direction, distSq, line.direction);

                    let dotProduct2 = Vector2.dot(relativeVelocity, line.direction);
                    let ld = Vector2.multiply(line.direction, dotProduct2);
                    u = Vector2.subtract(ld, relativeVelocity, ld);
                }
            } else {
                /* Collision. Project on cut-off circle of time timeStep. */
                let invTimeStep = 1.0 / this.simulator.timeStep_;

                /* Vector from cutoff center to relative velocity. */
                let rpi = Vector2.multiply(relativePosition, invTimeStep, __vecTemp3);
                let w = Vector2.subtract(relativeVelocity, rpi, rpi);

                let wLength = w.length();
                let unitW = Vector2.divide(w, wLength, w);

                line.direction = new Vector2(unitW.y, -unitW.x);
                u = Vector2.multiply(unitW, combinedRadius * invTimeStep - wLength);
            }

            u = Vector2.multiply(u, 0.5);
            line.point = Vector2.add(this.velocity_, u, u);
            this.orcaLines_.push(line);
        }

        let lineFail = this.linearProgram2(this.orcaLines_, this.maxSpeed_, this.prefVelocity_, false);
        this.newVelocity_ = lineFail.result;
        if (lineFail.count < this.orcaLines_.length) {
            this.newVelocity_ = this.linearProgram3(this.orcaLines_, numObstLines, lineFail.count, this.maxSpeed_, this.newVelocity_);
        }
    }

    /**
     * Inserts an agent neighbor into the set of neighbors of this agent.
     * @param agent A pointer to the agent to be inserted.
     * @param rangeSq The squared range around this agent.
     */
    insertAgentNeighbor(agent: Agent, rangeSq: number): number {
        if (this != agent) {
            let distSq = Vector2.subtract(this.position_, agent.position_).lengthSq();

            if (distSq < rangeSq) {
                if (this.agentNeighbors_.length < this.maxNeighbors_) {
                    this.agentNeighbors_.push(new KeyValuePair(distSq, agent));
                }

                let i = this.agentNeighbors_.length - 1;

                while (i != 0 && distSq < this.agentNeighbors_[i - 1].key) {
                    this.agentNeighbors_[i] = this.agentNeighbors_[i - 1];
                    --i;
                }

                this.agentNeighbors_[i] = new KeyValuePair(distSq, agent);

                if (this.agentNeighbors_.length == this.maxNeighbors_) {
                    rangeSq = this.agentNeighbors_[this.agentNeighbors_.length - 1].key;
                }
            }
        }
        return rangeSq;
    }

    /**
     * Inserts a static obstacle neighbor into the set of neighbors of this agent.
     * @param obstacle The number of the static obstacle to be inserted.
     * @param rangeSq The squared range around this agent.
     */
    insertObstacleNeighbor(obstacle: Obstacle, rangeSq: number): number {
        let nextObstacle = obstacle.next_;

        let distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, this.position_);

        if (distSq < rangeSq) {
            this.obstacleNeighbors_.push(new KeyValuePair(distSq, obstacle));

            let i = this.obstacleNeighbors_.length - 1;

            while (i != 0 && distSq < this.obstacleNeighbors_[i - 1].key) {
                this.obstacleNeighbors_[i] = this.obstacleNeighbors_[i - 1];
                --i;
            }
            this.obstacleNeighbors_[i] = new KeyValuePair(distSq, obstacle);
        }
        return rangeSq;
    }

    private vpv = new Vector2();
    /**
     * Updates the two-dimensional position and two-dimensional velocity of this agent.
     */
    update() {
        this.velocity_ = this.newVelocity_.clone();
        let vt = Vector2.multiply(this.velocity_, this.simulator.timeStep_, this.vpv);
        this.position_ = Vector2.add(this.position_, vt, this.position_);
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
    linearProgram1(lines: Array<Line>, lineNo: int, radius: number, optVelocity: Vector2, directionOpt: boolean): Vector2 {
        let lineNoDirection = lines[lineNo].direction, lineNoPoint = lines[lineNo].point;
        let dotProduct = Vector2.dot(lineNoPoint, lineNoDirection);
        let discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - lineNoPoint.lengthSq();

        /* Max speed circle fully invalidates line lineNo. */
        if (discriminant < 0.0)
            return null;

        let sqrtDiscriminant = Math.sqrt(discriminant);
        let tLeft = -dotProduct - sqrtDiscriminant;
        let tRight = -dotProduct + sqrtDiscriminant;

        let vt = new Vector2();

        for (let i = 0; i < lineNo; ++i) {
            let denominator = Vector2.cross(lineNoDirection, lines[i].direction);
            let numerator = Vector2.cross(lines[i].direction, Vector2.subtract(lineNoPoint, lines[i].point, vt));

            if (Math.abs(denominator) <= RVOMath.RVO_EPSILON) {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0)
                    return null;

                continue;
            }

            let t = numerator / denominator;

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

        let vector: Vector2 = vt;
        if (directionOpt) {
            /* Optimize direction. */
            if (Vector2.dot(optVelocity, lineNoDirection) > 0.0) {
                /* Take right extreme. */
                vt = Vector2.multiply(lineNoDirection, tRight, vt);
                vector = Vector2.add(lineNoPoint, vt, vt);
            } else {
                /* Take left extreme. */
                vt = Vector2.multiply(lineNoDirection, tLeft, vt);
                vector = Vector2.add(lineNoPoint, vt, vt);
            }
        } else {
            /* Optimize closest point. */
            vt = Vector2.subtract(optVelocity, lineNoPoint, vt);
            let t = Vector2.dot(lineNoDirection, vt);

            if (t < tLeft) {
                vt = Vector2.multiply(lineNoDirection, tLeft, vt);
                vector = Vector2.add(lineNoPoint, vt, vt);
            } else if (t > tRight) {
                vt = Vector2.multiply(lineNoDirection, tRight, vt);
                vector = Vector2.add(lineNoPoint, vt, vt);
            } else {
                vt = Vector2.multiply(lineNoDirection, t, vt);
                vector = Vector2.add(lineNoPoint, vt, vt);
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
    linearProgram2(lines: Array<Line>, radius: number, optVelocity: Vector2, directionOpt: boolean): { count: int, result: Vector2 } {
        let result: Vector2;
        if (directionOpt) {
            /*
             * Optimize direction. Note that the optimization velocity is of
             * unit length in this case.
             */
            result = Vector2.multiply(optVelocity, radius);
        } else if (optVelocity.lengthSq() > RVOMath.sqr(radius)) {
            /* Optimize closest point and outside circle. */
            let n = Vector2.normalize(optVelocity);
            result = Vector2.multiply(n, radius, n);
        } else {
            /* Optimize closest point and inside circle. */
            result = optVelocity.clone();
        }

        let vt = new Vector2();
        for (let i = 0; i < lines.length; ++i) {
            let lpr = Vector2.subtract(lines[i].point, result, vt);
            if (Vector2.cross(lines[i].direction, lpr) > 0.0) {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                // let tempResult = result;
                let tempResult = this.linearProgram1(lines, i, radius, optVelocity, directionOpt);
                if (tempResult == null)
                    return { count: i, result };
                result = tempResult;
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
    linearProgram3(lines: Array<Line>, numObstLines: int, beginLine: int, radius: number, result: Vector2): Vector2 {
        let distance = 0.0;

        let vt = new Vector2();
        for (let i = beginLine; i < lines.length; ++i) {
            if (Vector2.cross(lines[i].direction, Vector2.subtract(lines[i].point, result, vt)) > distance) {
                /* Result does not satisfy constraint of line i. */
                let projLines = new Array<Line>();
                for (let ii = 0; ii < numObstLines; ++ii) {
                    projLines.push(lines[ii]);
                }

                for (let j = numObstLines; j < i; ++j) {
                    let line: Line = new Line();

                    let determinant = Vector2.cross(lines[i].direction, lines[j].direction);

                    if (Math.abs(determinant) <= RVOMath.RVO_EPSILON) {
                        /* Line i and line j are parallel. */
                        if (Vector2.dot(lines[i].direction, lines[j].direction) > 0.0) {
                            /* Line i and line j point in the same direction. */
                            continue;
                        } else {
                            /* Line i and line j point in opposite direction. */
                            let lialj = Vector2.add(lines[i].point, lines[j].point);
                            line.point = Vector2.multiply(lialj, 0.5);
                        }
                    } else {
                        let lislj = Vector2.subtract(lines[i].point, lines[j].point);
                        let lids = Vector2.multiply(lines[i].direction, Vector2.cross(lines[j].direction, lislj) / determinant, lislj);
                        line.point = Vector2.add(lines[i].point, lids, lislj);
                    }

                    let ljsli = Vector2.subtract(lines[j].direction, lines[i].direction);
                    line.direction.set(ljsli).normalize();
                    projLines.push(line);
                }

                let results = this.linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y, lines[i].direction.x), true);
                if (results.count >= projLines.length) {
                    /*
                     * This should in principle not happen. The result is by
                     * definition already in the feasible region of this
                     * linear program. If it fails, it is due to small
                     * floating point error, and the current result is kept.
                     */
                    result = results.result;
                }

                let lisr = Vector2.subtract(lines[i].point, result, vt);
                distance = Vector2.cross(lines[i].direction, lisr);
            }
        }
        return result;
    }
}