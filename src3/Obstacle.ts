import { Vector2 } from "./Vector2";

/**
 * Defines static obstacles in the simulation.
 */
export class Obstacle {
    next_: Obstacle;
    previous_: Obstacle;
    direction_: Vector2 = new Vector2();
    point_: Vector2 = new Vector2();
    id_: number;
    convex_: boolean;
}