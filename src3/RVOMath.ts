import { Vector2 } from "./Vector2";

export class RVOMath {
    public static readonly RVO_EPSILON = 0.00001;

    public static sqr(scalar: number): number {
        return scalar * scalar;
    }

    public static sqrt(scalar: number): number {
        return Math.sqrt(scalar);
    }

    public static absSq(vector: Vector2): number {
        return Vector2.dot(vector, vector);
    }

    public static fAbs(scalar: number): number {
        return Math.abs(scalar);
    }

    public static abs(vector: Vector2): number {
        return RVOMath.sqrt(RVOMath.absSq(vector));
    }

    public static normalize(vector: Vector2, out: Vector2 = new Vector2): Vector2 {
        let length = RVOMath.abs(vector);
        return Vector2.divide(vector, length, out);
    }

    public static det(vector1: Vector2, vector2: Vector2): number {
        return vector1.x * vector2.y - vector1.y * vector2.x;
    }

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

    public static leftOf(a: Vector2, b: Vector2, c: Vector2): number {
        return RVOMath.det(Vector2.subtract(a, c), Vector2.subtract(b, a));
    }

}