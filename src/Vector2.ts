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
 * <summary>Defines a two-dimensional vector.</summary>
 */
export class Vector2 {
  /**
   * <summary>Constructs and initializes a two-dimensional vector from the
   * specified xy-coordinates.</summary>
   *
   * <param name="x">The x-coordinate of the two-dimensional vector.
   * </param>
   * <param name="y">The y-coordinate of the two-dimensional vector.
   * </param>
   */
  constructor(public x: number = 0, public y: number = 0) { }

  /**
   * <summary>Returns the string representation of this vector.</summary>
   *
   * <returns>The string representation of this vector.</returns>
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
   * <summary>Computes the dot product of the two specified
   * two-dimensional vectors.</summary>
   *
   * <returns>The dot product of the two specified two-dimensional
   * vectors.</returns>
   *
   * <param name="vector1">The first two-dimensional vector.</param>
   * <param name="vector2">The second two-dimensional vector.</param>
   */
  public static dot(vector1: Vector2, vector2: Vector2): number {
    return vector1.x * vector2.x + vector1.y * vector2.y;
  }

  /**
   * <summary>Computes the scalar multiplication of the specified
   * two-dimensional vector with the specified scalar value.</summary>
   *
   * <returns>The scalar multiplication of the specified two-dimensional
   * vector with the specified scalar value.</returns>
   *
   * <param name="scalar">The scalar value.</param>
   * <param name="vector">The two-dimensional vector.</param>
   */
  public static multiply(vector: Vector2, scalar: number, out: Vector2 = new Vector2) {
    out.x = vector.x * scalar;
    out.y = vector.y * scalar;
    return out;
  }

  /**
   * <summary>Computes the scalar division of the specified
   * two-dimensional vector with the specified scalar value.</summary>
   *
   * <returns>The scalar division of the specified two-dimensional vector
   * with the specified scalar value.</returns>
   *
   * <param name="vector">The two-dimensional vector.</param>
   * <param name="scalar">The scalar value.</param>
   */
  public static divide(vector: Vector2, scalar: number, out: Vector2 = new Vector2) {
    out.x = vector.x / scalar;
    out.y = vector.y / scalar;
    return out;
  }

  /**
   * <summary>Computes the vector sum of the two specified two-dimensional
   * vectors.</summary>
   *
   * <returns>The vector sum of the two specified two-dimensional vectors.
   * </returns>
   *
   * <param name="vector1">The first two-dimensional vector.</param>
   * <param name="vector2">The second two-dimensional vector.</param>
   */
  public static add(vector1: Vector2, vector2: Vector2, out: Vector2 = new Vector2) {
    out.x = vector1.x + vector2.x;
    out.y = vector1.y + vector2.y;
    return out;
  }

  /**
   * <summary>Computes the vector difference of the two specified
   * two-dimensional vectors</summary>
   *
   * <returns>The vector difference of the two specified two-dimensional
   * vectors.</returns>
   *
   * <param name="vector1">The first two-dimensional vector.</param>
   * <param name="vector2">The second two-dimensional vector.</param>
   */
  public static subtract(vector1: Vector2, vector2: Vector2, out: Vector2 = new Vector2) {
    out.x = vector1.x - vector2.x;
    out.y = vector1.y - vector2.y;
    return out;
  }

  /**
   * <summary>Computes the negation of the specified two-dimensional
   * vector.</summary>
   *
   * <returns>The negation of the specified two-dimensional vector.
   * </returns>
   *
   * <param name="vector">The two-dimensional vector.</param>
   */
  public static negate(vector: Vector2, out: Vector2 = new Vector2) {
    out.x = -vector.x;
    out.y = -vector.y;
    return out;
  }
}
