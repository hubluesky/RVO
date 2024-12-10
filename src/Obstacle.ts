import { IVector2, Vector2 } from "./Vector2";

/**
 * Defines static obstacles in the simulation.
 */
export class Obstacle {
    next: Obstacle;
    previous: Obstacle;
    convex: boolean;

    public constructor(public readonly id: number, public layer: number) { }

    private _direction_: Vector2 = new Vector2();
    public get direction(): IVector2 { return this._direction_; }
    public set direction(value: IVector2) { this._direction_.set(value); }
    private _point_: Vector2 = new Vector2();
    public get point(): IVector2 { return this._point_; }
    public set point(value: IVector2) { this._point_.set(value); }
}