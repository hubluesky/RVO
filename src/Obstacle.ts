import { Vector2 } from "./Vector2";

/**
 * Defines static obstacles in the simulation.
 */
export class Obstacle {
    id_: number;
    next_: Obstacle;
    previous_: Obstacle;
    convex_: boolean;

    private _direction_: Vector2 = new Vector2();
    public get direction_(): Vector2 { return this._direction_; }
    public set direction_(value: Vector2) { this._direction_.set(value); }
    private _point_: Vector2 = new Vector2();
    public get point_(): Vector2 { return this._point_; }
    public set point_(value: Vector2) { this._point_.set(value); }
}