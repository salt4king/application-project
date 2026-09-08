#pragma once

#include "geometry.hpp"

#include <utility>
#include <vector>

/* robots.hpp -- the robots, and the collision query.
 *
 * Nothing here does geometry. The classes only describe their own shape; the
 * query layer only decides what a measurement means. All math stays in geom.
 */

/* Each chassis answers exactly two questions about itself. That is the whole
 * contract -- no subclass implements a collision test.
 *
 * The alternative, a virtual collidesWith() resolved by double dispatch, was
 * rejected: it brings back the O(N^2) coupling (every chassis must know every
 * other) and scatters the math across the hierarchy, which is what the
 * project's hint warns against. */
class Robot {
public:
    Robot(double x, double y) : m_center(x, y) {}

    // Polymorphic and held by base pointer in the all-pairs sweep, so this is
    // required, not optional.
    virtual ~Robot() = default;

    virtual geom::ConvexCore core() const = 0;   // my points, in world space
    virtual double skinRadius() const = 0;       // my thickness around them

protected:
    Vec2 m_center;
};

// A point, inflated by its radius. Only works because a 1-vertex core is a
// first-class citizen in the kernel.
class CircularRobot : public Robot {
public:
    CircularRobot(double x, double y, double radius)
        : Robot(x, y), m_radius(radius) {}

    geom::ConvexCore core() const override { return geom::makePoint(m_center); }
    double skinRadius() const override { return m_radius; }

private:
    double m_radius;
};

/* Four corners, inflated by nothing.
 *
 * headingDegrees defaults to 0, so every call in the provided driver still
 * compiles unchanged. It is also the cheapest proof the abstraction holds: if
 * rotation had needed changes in geom, the kernel wouldn't really be blind to
 * shape. */
class RectangularRobot : public Robot {
public:
    RectangularRobot(double x, double y, double width, double height,
                     double headingDegrees = 0.0)
        : Robot(x, y),
          m_width(width),
          m_height(height),
          m_heading(headingDegrees * 3.14159265358979323846 / 180.0) {}

    geom::ConvexCore core() const override {
        const double hw = m_width * 0.5, hh = m_height * 0.5;

        // Counter-clockwise. geom::contains() is order-agnostic, but keeping
        // it consistent makes the shape easy to reason about.
        const Vec2 local[4] = {{-hw,-hh}, {+hw,-hh}, {+hw,+hh}, {-hw,+hh}};

        geom::ConvexCore c;
        for (const Vec2& p : local) c.add(m_center + rotate(p, m_heading));
        return c;
    }

    double skinRadius() const override { return 0.0; }   // boundary IS the core

private:
    double m_width;
    double m_height;
    double m_heading;   // radians; the API speaks degrees
};

namespace collision {

/* How close counts as touching. Policy, not numerics, which is why it lives
 * here and not in geom.
 *
 * "In contact" means exact tangency must count, and the provided testcases
 * rely on it: r3 and r6 share the edge x = -13.5 exactly. The slack absorbs
 * rounding while staying far under the real margins (tightest genuine call in
 * the driver is 0.076). On a real robot this would be a safety margin. */
constexpr double kContactTolerance = 1e-9;

/* Exact gap between two robots. Positive means apart, and the value is the
 * true clearance -- more useful than a bool when deciding to slow down.
 *
 * Limitation: on overlap coreDistance saturates at 0, so a negative result
 * says THAT they interpenetrate, not by how much. True depth needs SAT or EPA;
 * nothing here asks for collision response. */
inline double clearance(const Robot& a, const Robot& b) {
    return geom::coreDistance(a.core(), b.core())
         - (a.skinRadius() + b.skinRadius());
}

inline bool areColliding(const Robot& a, const Robot& b) {
    return clearance(a, b) <= kContactTolerance;
}

inline std::vector<std::pair<int, int>>
collidingPairs(const std::vector<const Robot*>& robots) {
    std::vector<std::pair<int, int>> hits;
    for (std::size_t i = 0; i < robots.size(); ++i)
        for (std::size_t j = i + 1; j < robots.size(); ++j)
            if (areColliding(*robots[i], *robots[j]))
                hits.emplace_back(static_cast<int>(i), static_cast<int>(j));
    return hits;
}

} // namespace collision

/* What the driver calls. Free function taking both as const Robot&, so it is
 * symmetric -- a member would imply an asymmetry the problem doesn't have.
 *
 * One line because every hard part already lives where it belongs. */
inline bool isColliding(const Robot& a, const Robot& b) {
    return collision::areColliding(a, b);
}
