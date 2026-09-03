#pragma once

/* Put the necessary #include statements for your solution below to expose them
 * to the entrypoint driver.
 *
 * The driver expects the following methods and functions:
 *    - Constructor for CircularRobot.
 *    - Constructor for RectangularRobot.
 *    - A function isColliding() that can take in any two Robots.
 */

/* collision.hpp transitively pulls in the whole stack:
 *
 *      vec2.hpp       2D vector primitives
 *        └ geometry.hpp   the convex-core distance kernel (all the math)
 *            └ robot.hpp      Robot / CircularRobot / RectangularRobot
 *                └ collision.hpp  clearance(), isColliding()
 *
 * One include is enough because the layering is strictly bottom-up: each
 * header depends only on the one beneath it, and nothing depends on this file.
 * See NOTES.md for the design rationale behind that stack. */
#include "collision.hpp"
