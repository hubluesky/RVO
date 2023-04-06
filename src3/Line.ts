import { Vector2 } from "./Vector2";

/**
 * Defines a directed line.
 */
export class Line {
    public constructor(private _point: Vector2 = new Vector2, private _direction: Vector2 = new Vector2) { }
    public get direction(): Vector2 { return this._direction; }
    public set direction(value: Vector2) { this._direction.set(value); }
    public get point(): Vector2 { return this._point; }
    public set point(value: Vector2) { this._point.set(value); }
}