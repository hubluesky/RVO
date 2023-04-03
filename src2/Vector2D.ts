export default class Vector2D {

  static ZERO: Vector2D = new Vector2D();

  x = 0;
  y = 0;

  constructor(x: number = 0, y: number = 0) {
    this.x = x;
    this.y = y;
  }

  plus(vector: Vector2D): Vector2D {
    return new Vector2D(this.x + vector.x, this.y + vector.y);
  }

  //subtract
  minus(vector: Vector2D): Vector2D {
    return new Vector2D(this.x - vector.x, this.y - vector.y);
  }

  multiply(vector: Vector2D): number {
    return this.x * vector.x + this.y * vector.y;
  }

  scale(k: number): Vector2D {
    return new Vector2D(this.x * k, this.y * k);
  }

  normalize(): Vector2D {
    return this.scale(1 / this.abs());
  }

  absSq(): number {
    return this.multiply(this);
  }

  abs(): number {
    return Math.sqrt(this.absSq());
  }

  clone(): Vector2D {
    return new Vector2D(this.x, this.y);
  }

  public static sub(vector1: Vector2D, vector2: Vector2D, out: Vector2D = new Vector2D) {
    out.x = vector1.x - vector2.x;
    out.y = vector1.y - vector2.y;
    return out;
  }

  public dot(other: Vector2D): number {
    return this.x * other.x + this.y * other.y;
  }

  public mul(scalar: number): this {
    this.x = this.x * scalar;
    this.y = this.y * scalar;
    return this;
  }

  public div(scalar: number): this {
    this.x = this.x / scalar;
    this.y = this.y / scalar;
    return this;
  }

  public add(other: Vector2D): this {
    this.x = this.x + other.x;
    this.y = this.y + other.y;
    return this;
  }

  public sub(other: Vector2D): this {
    this.x = this.x - other.x;
    this.y = this.y - other.y;
    return this;
  }

  public nge(): this {
    this.x = -this.x;
    this.y = -this.y;
    return this;
  }
}
