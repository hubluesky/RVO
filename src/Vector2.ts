export interface IVector2 { x: number, y: number }

/**
 * Defines a two-dimensional vector.
 */
export class Vector2 implements IVector2 {
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

    /**
     * Computes the squared length of a specified two-dimensional vector.
     * @returns The squared length of the two-dimensional vector.
     */
    public lengthSq(): number {
        return this.dot(this);
    }

    /**
     * Computes the length of a specified two-dimensional vector.
     * @returns The length of the two-dimensional vector.
     */
    public length(): number {
        return Math.sqrt(this.lengthSq());
    }

    /**
     * Computes the normalization of the specified two-dimensional vector.
     * @returns The normalization of the two-dimensional vector.
     */
    public normalize(): Vector2 {
        return this.multiply(1 / this.length());
    }

    public dot(other: IVector2): number {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Computes the determinant of a two-dimensional square matrix with rows consisting of the specified two-dimensional vectors.
     * @param other The bottom row of the two-dimensional square matrix.
     * @returns The determinant of the two-dimensional square matrix.
     */
    public cross(other: IVector2): number {
        return this.x * other.y - this.y * other.x;
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

    public add(other: IVector2): this {
        this.x = this.x + other.x;
        this.y = this.y + other.y;
        return this;
    }

    public subtract(other: IVector2): this {
        this.x = this.x - other.x;
        this.y = this.y - other.y;
        return this;
    }

    public reset(x: number = 0, y: number = 0): this {
        this.x = x;
        this.y = y;
        return this;
    }

    public set(other: IVector2): this {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    /**
     * Computes the negation of the specified two-dimensional vector.
     * @returns The negation of the specified two-dimensional vector.
     */
    public negate(): Vector2 {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }

    public clone(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    public static lengthSq(vector: IVector2): number {
        return Vector2.dot(vector, vector);
    }

    public static distanceSq(vector1: IVector2, vector2: IVector2): number {
        const x = vector1.x - vector2.x;
        const y = vector1.y - vector2.y;
        return x * x + y * y;
    }

    public static distance(vector1: IVector2, vector2: IVector2): number {
        return Math.sqrt(Vector2.distanceSq(vector1, vector2));
    }

    /**
     * Computes the vector sum of the two specified two-dimensional
     * vectors.
     * @param vector1 The first two-dimensional vector.
     * @param vector2 The second two-dimensional vector.
     * @param out The out vector.
     * @returns The vector sum of the two specified two-dimensional vectors.
     */
    public static add(a: IVector2, b: IVector2, out: Vector2 = new Vector2()): Vector2 {
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
    public static subtract(a: IVector2, b: IVector2, out: Vector2 = new Vector2()): Vector2 {
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
    public static multiply(a: IVector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
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
    public static divide(a: IVector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
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
    public static dot(a: IVector2, b: IVector2): number {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * Computes the determinant of a two-dimensional square matrix with rows consisting of the specified two-dimensional vectors.
     * @param vector1 The top row of the two-dimensional square matrix.
     * @param vector2 The bottom row of the two-dimensional square matrix.
     * @returns The determinant of the two-dimensional square matrix.
     */
    public static cross(vector1: IVector2, vector2: IVector2): number {
        return vector1.x * vector2.y - vector1.y * vector2.x;
    }

    /**
     * Computes the normalization of the specified two-dimensional vector.
     * @param vector The two-dimensional vector whose normalization is to be computed.
     * @returns The normalization of the two-dimensional vector.
     */
    public static normalize(vector: IVector2, out: Vector2 = new Vector2): Vector2 {
        return Vector2.divide(vector, Math.sqrt(Vector2.lengthSq(vector)), out);
    }

    /**
     * Computes the negation of the specified two-dimensional vector.
     * @param vector The two-dimensional vector.
     * @param out The out vector.
     * @returns The negation of the specified two-dimensional vector.
     */
    public static negate(a: IVector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = -a.x;
        out.y = -a.y;
        return out;
    }
}