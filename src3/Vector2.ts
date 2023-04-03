
export class Vector2 {
    public constructor(public x: number = 0, public y: number = 0) { }

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

    public copy(other: Vector2): Vector2 {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    public clone(): Vector2 {
        return new Vector2(this.x, this.y);
    }

    public static add(a: Vector2, b: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }

    public static subtract(a: Vector2, b: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }

    public static multiply(a: Vector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x * scalar;
        out.y = a.y * scalar;
        return out;
    }

    public static divide(a: Vector2, scalar: number, out: Vector2 = new Vector2()): Vector2 {
        out.x = a.x / scalar;
        out.y = a.y / scalar;
        return out;
    }

    public static dot(a: Vector2, b: Vector2): number {
        return a.x * b.x + a.y * b.y;
    }

    public static negate(a: Vector2, out: Vector2 = new Vector2()): Vector2 {
        out.x = -a.x;
        out.y = -a.y;
        return out;
    }
}