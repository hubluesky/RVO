import { Vector2 } from "./Vector2";

/**
 * Defines static obstacles in the simulation.
 */
export class Obstacle {
    next: Obstacle;
    previous: Obstacle;
    convex: boolean;

    public constructor(public readonly id: number, public layer: number) { }

    private _direction_: Vector2 = new Vector2();
    public get direction(): Vector2 { return this._direction_; }
    public set direction(value: Vector2) { this._direction_.set(value); }
    private _point_: Vector2 = new Vector2();
    public get point(): Vector2 { return this._point_; }
    public set point(value: Vector2) { this._point_.set(value); }
}