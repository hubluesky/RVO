/*
 * Vector2.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */


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
  constructor(public x: number = 0, public y: number = 0) { }

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

  public negate(): this {
    this.x = -this.x;
    this.y = -this.y;
    return this;
  }

  public clone(): Vector2 {
    return new Vector2(this.x, this.y);
  }

  /**
   * Computes the dot product of the two specified
   * two-dimensional vectors.
   * @param vector1 The first two-dimensional vector.
   * @param vector2 The second two-dimensional vector.
   * @returns The dot product of the two specified two-dimensional
   * vectors.
   */
  public static dot(vector1: Vector2, vector2: Vector2): number {
    return vector1.x * vector2.x + vector1.y * vector2.y;
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
  public static multiply(vector: Vector2, scalar: number, out: Vector2 = new Vector2) {
    out.x = vector.x * scalar;
    out.y = vector.y * scalar;
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
  public static divide(vector: Vector2, scalar: number, out: Vector2 = new Vector2) {
    out.x = vector.x / scalar;
    out.y = vector.y / scalar;
    return out;
  }

  /**
   * Computes the vector sum of the two specified two-dimensional
   * vectors.
   * @param vector1 The first two-dimensional vector.
   * @param vector2 The second two-dimensional vector.
   * @param out The out vector.
   * @returns The vector sum of the two specified two-dimensional vectors.
   */
  public static add(vector1: Vector2, vector2: Vector2, out: Vector2 = new Vector2) {
    out.x = vector1.x + vector2.x;
    out.y = vector1.y + vector2.y;
    return out;
  }

  /**
   * Computes the vector difference of the two specified two-dimensional vectors
   * @param vector1 The first two-dimensional vector.
   * @param vector2 The second two-dimensional vector.
   * @param out The out vector.
   * @returns The vector difference of the two specified two-dimensional vectors.
   */
  public static subtract(vector1: Vector2, vector2: Vector2, out: Vector2 = new Vector2) {
    out.x = vector1.x - vector2.x;
    out.y = vector1.y - vector2.y;
    return out;
  }

  /**
   * Computes the negation of the specified two-dimensional vector.
   * @param vector The two-dimensional vector.
   * @param out The out vector.
   * @returns The negation of the specified two-dimensional vector.
   */
  public static negate(vector: Vector2, out: Vector2 = new Vector2) {
    out.x = -vector.x;
    out.y = -vector.y;
    return out;
  }
}
