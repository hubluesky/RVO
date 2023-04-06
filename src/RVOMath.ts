import { Vector2 } from "./Vector2";

const __vecTemp1 = new Vector2();
const __vecTemp2 = new Vector2();

/**
 * Contains functions and constants used in multiple classes.
 */
export class RVOMath {
    /**
     * A sufficiently small positive number.
     */
    public static readonly RVO_EPSILON = 0.00001;

    /**
     * Computes the square of a float.
     * @param scalar The float to be squared.
     * @returns The square of the float.
     */
    public static sqr(scalar: number): number {
        return scalar * scalar;
    }

    /**
     * Computes the squared distance from a line segment with the specified endpoints to a specified point.
     * @param vector1 The first endpoint of the line segment.
     * @param vector2 The second endpoint of the line segment.
     * @param vector3 The point to which the squared distance is to be calculated.
     * @returns The squared distance from the line segment to the point.
     */
    public static distSqPointLineSegment(vector1: Vector2, vector2: Vector2, vector3: Vector2): number {
        let vt1 = Vector2.subtract(vector3, vector1, __vecTemp1);
        let vt2 = Vector2.subtract(vector2, vector1, __vecTemp2);
        let r = Vector2.dot(vt1, vt2) / vt2.lengthSq();
        if (r < 0) return vt1.lengthSq();
        if (r > 1) return Vector2.subtract(vector3, vector2, __vecTemp1).lengthSq();
        vt2 = Vector2.multiply(vt2, r, __vecTemp2);
        vt2 = Vector2.add(vector1, vt2, __vecTemp2);
        return Vector2.subtract(vector3, vt2, __vecTemp2).lengthSq();
    }

    /**
     * Computes the signed distance from a line connecting the specified points to a specified point.
     * @param a The first point on the line.
     * @param b The second point on the line.
     * @param c The point to which the signed distance is to be calculated.
     * @returns Positive when the point c lies to the left of the line ab.
     */
    public static leftOf(a: Vector2, b: Vector2, c: Vector2): number {
        return Vector2.cross(Vector2.subtract(a, c, __vecTemp1), Vector2.subtract(b, a, __vecTemp2));
    }
}