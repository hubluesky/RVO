import { Vector2 } from "./Vector2";

export class Obstacle {
    next_: Obstacle;
    previous_: Obstacle;
    direction_: Vector2;
    point_: Vector2;
    id_: number;
    convex_: boolean;
}