#pragma once

/* Put the necessary #include statements for your solution below to expose them
 * to the entrypoint driver.
 *
 * The driver expects the following methods and functions:
 *    - Constructor for CircularRobot.
 *    - Constructor for RectangularRobot.
 *    - A function isColliding() that can take in any two Robots.
 */

/* The solution is two headers:
 *
 *      geometry.hpp   Vec2, and the convex-core distance kernel (the math)
 *        └ robots.hpp   Robot / CircularRobot / RectangularRobot, and
 *                       isColliding() (the model and the query)
 *
 * robots.hpp includes geometry.hpp, so this one include exposes everything the
 * driver needs. See NOTES.md for the design rationale. */
#include "robots.hpp"
