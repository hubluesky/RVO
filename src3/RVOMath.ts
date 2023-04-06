import { Vector2 } from "./Vector2";

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
     * Computes the squared length of a specified two-dimensional vector.
     * @param vector The two-dimensional vector whose squared length is to be computed.
     * @returns The squared length of the two-dimensional vector.
     */
    public static absSq(vector: Vector2): number {
        return Vector2.dot(vector, vector);
    }

    /**
     * Computes the length of a specified two-dimensional vector.
     * @param vector The two-dimensional vector whose length is to be computed.
     * @returns The length of the two-dimensional vector.
     */
    public static abs(vector: Vector2): number {
        return Math.sqrt(RVOMath.absSq(vector));
    }

    /**
     * Computes the normalization of the specified two-dimensional vector.
     * @param vector The two-dimensional vector whose normalization is to be computed.
     * @returns The normalization of the two-dimensional vector.
     */
    public static normalize(vector: Vector2, out: Vector2 = new Vector2): Vector2 {
        let length = RVOMath.abs(vector);
        return Vector2.divide(vector, length, out);
    }

    /**
     * Computes the determinant of a two-dimensional square matrix with rows consisting of the specified two-dimensional vectors.
     * @param vector1 The top row of the two-dimensional square matrix.
     * @param vector2 The bottom row of the two-dimensional square matrix.
     * @returns The determinant of the two-dimensional square matrix.
     */
    public static det(vector1: Vector2, vector2: Vector2): number {
        return vector1.x * vector2.y - vector1.y * vector2.x;
    }

    /**
     * Computes the squared distance from a line segment with the specified endpoints to a specified point.
     * @param vector1 The first endpoint of the line segment.
     * @param vector2 The second endpoint of the line segment.
     * @param vector3 The point to which the squared distance is to be calculated.
     * @returns The squared distance from the line segment to the point.
     */
    public static distSqPointLineSegment(vector1: Vector2, vector2: Vector2, vector3: Vector2): number {
        let vt1 = Vector2.subtract(vector3, vector1);
        let vt2 = Vector2.subtract(vector2, vector1);
        let r = Vector2.dot(vt1, vt2) / RVOMath.absSq(vt2);
        if (r < 0) return RVOMath.absSq(vt1);
        if (r > 1) return RVOMath.absSq(Vector2.subtract(vector3, vector2));
        vt2 = Vector2.multiply(vt2, r, vt2);
        vt2 = Vector2.add(vector1, vt2, vt2);
        return RVOMath.absSq(Vector2.subtract(vector3, vt2, vt2));
    }

    /**
     * Computes the signed distance from a line connecting the specified points to a specified point.
     * @param a The first point on the line.
     * @param b The second point on the line.
     * @param c The point to which the signed distance is to be calculated.
     * @returns Positive when the point c lies to the left of the line ab.
     */
    public static leftOf(a: Vector2, b: Vector2, c: Vector2): number {
        return RVOMath.det(Vector2.subtract(a, c), Vector2.subtract(b, a));
    }
}