/**
 * Defines a two-dimensional vector.
 */
export class Vector2 {
    /**
     * Constructs and initializes a two-dimensional vector from the
     * specified xy-coordinates.
     * @param x The x-coordinate of the two-dimensional vector.
     * @param y The y-coordinate of the two-dimensional vector.
     */
    public constructor(public x: number = 0, public y: number = 0) { }

    /**
     * Returns the string representation of this vector.
     * @returns The string representation of this vector.
     */
    public toString(): string {
        return "(" + this.x + "," + this.y + ")";
    }

    public dot(other: Vector2): number {
        return this.x * other.x + this.y * other.y;
    }

    public multiply(scalar: number): this {
        this.x = this.x * scalar;
        this.y = this.y * scalar;
        return this;
    }

    public divide(scalar: number): this {
        this.x = this.x / scalar;
        this.y = this.y / scalar;
        return this;
    }

    public add(other: Vector2): this {
        this.x = this.x + other.x;
        this.y = this.y + other.y;
        return this;
    }

    public subtract(other: Vector2): this {
        this.x = this.x - other.x;
        this.y = this.y - other.y;
        return this;
    }

    public reset(): this {
        this.x = 0;
        this.y = 0;
        return this;
    }

    public set(other: Vector2): Vector2 {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    public clone(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    /**
     * Computes the vector sum of the two specified two-dimensional
     * vectors.
     * @param vector1 The first two-dimensional vector.
     * @param vector2 The second two-dimensional vector.
     * @param out The out vector.
     * @returns The vector sum of the two specified two-dimensional vectors.
     */
    public static add(a: Vector2, b: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }

    /**
     * Computes the vector difference of the two specified two-dimensional vectors
     * @param vector1 The first two-dimensional vector.
     * @param vector2 The second two-dimensional vector.
     * @param out The out vector.
     * @returns The vector difference of the two specified two-dimensional vectors.
     */
    public static subtract(a: Vector2, b: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }

    /**
     * Computes the scalar multiplication of the specified
     * two-dimensional vector with the specified scalar value.
     * @param vector The scalar value.
     * @param scalar The two-dimensional vector.
     * @param out The out vector.
     * @returns The scalar multiplication of the specified two-dimensional
     * vector with the specified scalar value.
     */
    public static multiply(a: Vector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x * scalar;
        out.y = a.y * scalar;
        return out;
    }

    /**
     * Computes the scalar division of the specified
     * two-dimensional vector with the specified scalar value.
     * @param vector The two-dimensional vector.
     * @param scalar The scalar value.
     * @param out The out vector.
     * @returns The scalar division of the specified two-dimensional vector
     * with the specified scalar value.
     */
    public static divide(a: Vector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x / scalar;
        out.y = a.y / scalar;
        return out;
    }

    /**
     * Computes the dot product of the two specified
     * two-dimensional vectors.
     * @param vector1 The first two-dimensional vector.
     * @param vector2 The second two-dimensional vector.
     * @returns The dot product of the two specified two-dimensional
     * vectors.
     */
    public static dot(a: Vector2, b: Vector2): number {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * Computes the negation of the specified two-dimensional vector.
     * @param vector The two-dimensional vector.
     * @param out The out vector.
     * @returns The negation of the specified two-dimensional vector.
     */
    public static negate(a: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = -a.x;
        out.y = -a.y;
        return out;
    }
}