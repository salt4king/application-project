#pragma once

#include "geometry.hpp"

#include <utility>
#include <vector>

/* ===========================================================================
 * robots.hpp -- the robots, and the collision query.
 *
 * This is the "what am I measuring" half of the solution; geometry.hpp is the
 * "how do I measure it" half. Nothing in this file does geometry: the classes
 * only DESCRIBE their own shape, and the query layer only decides what a
 * measurement means. All the math stays behind geom::.
 *
 * SECTION 1: Robot, CircularRobot, RectangularRobot -- the model
 * SECTION 2: namespace collision, then isColliding() -- the query
 * ========================================================================= */


/* ===========================================================================
 * SECTION 1 -- the robots.
 *
 * WHAT THE BASE CLASS ASKS FOR, AND WHY
 * -------------------------------------
 * The tempting design is a virtual bool collidesWith(const Robot&) on the base
 * class, resolved by double dispatch (the visitor pattern). I deliberately did
 * not do that, for two reasons:
 *
 *   1. It reintroduces the O(N^2) explosion. Every new chassis type has to
 *      know how to collide with every existing one, so adding a hexagonal
 *      robot means editing every other robot class. That is precisely the
 *      coupling the core/skin model exists to remove.
 *   2. It scatters the math across the type hierarchy, which is what the
 *      project's own hint warns against.
 *
 * So a subclass never implements a collision test at all. It answers exactly
 * two questions about its own shape, and the kernel does the rest:
 *
 *      core()        -- its convex hull of points, in world space
 *      skinRadius()  -- how far that hull is inflated
 *
 * ========================================================================= */
class Robot {
public:
    Robot(double x, double y) : m_center(x, y) {}

    /* Virtual destructor: these are polymorphic types held by reference (and,
     * in the all-pairs sweep, by base-class pointer), so this is
     * non-negotiable. */
    virtual ~Robot() = default;

    /* --- the two questions every chassis must answer --- */

    /* The convex core, in WORLD space. World rather than local space keeps the
     * kernel free of transform plumbing: by the time geometry sees a shape,
     * the shape has already placed itself on the field. */
    virtual geom::ConvexCore core() const = 0;

    /* How far the core is inflated. Zero for a polygonal chassis. */
    virtual double skinRadius() const = 0;

protected:
    Vec2 m_center;
};

/* ---------------------------------------------------------------------------
 * CircularRobot -- a point, inflated by its radius.
 *
 * The entire shape lives in skinRadius(). Its core is a single vertex, which
 * is why the degenerate-edge handling in the kernel matters so much: this
 * class only works because a 1-vertex core is a first-class citizen there.
 * ------------------------------------------------------------------------- */
class CircularRobot : public Robot {
public:
    CircularRobot(double x, double y, double radius)
        : Robot(x, y), m_radius(radius) {}

    geom::ConvexCore core() const override {
        return geom::makePoint(m_center);
    }

    double skinRadius() const override { return m_radius; }

private:
    double m_radius;
};

/* ---------------------------------------------------------------------------
 * RectangularRobot -- four corners, inflated by nothing.
 *
 * The heading parameter is the one place I extended the required API. It
 * defaults to 0, so every call in the provided driver still compiles and means
 * exactly what it meant before; but a real robot on a real field is almost
 * never axis-aligned, and the core/skin model gets rotation essentially for
 * free (rotate the four corners; the kernel neither knows nor cares).
 *
 * It is also the cheapest possible proof that the abstraction is real: if
 * supporting rotation had required touching the geometry kernel, the claim
 * that the kernel is blind to shape would have been false.
 * ------------------------------------------------------------------------- */
class RectangularRobot : public Robot {
public:
    RectangularRobot(double x, double y, double width, double height,
                     double headingDegrees = 0.0)
        : Robot(x, y),
          m_width(width),
          m_height(height),
          m_heading(headingDegrees * 3.14159265358979323846 / 180.0) {}

    geom::ConvexCore core() const override {
        const double hw = m_width  * 0.5;
        const double hh = m_height * 0.5;

        // Listed counter-clockwise. Winding is documented rather than relied
        // upon -- geom::contains() is order-agnostic -- but keeping it
        // consistent makes the shape easy to reason about.
        const Vec2 local[4] = {
            {-hw, -hh}, {+hw, -hh}, {+hw, +hh}, {-hw, +hh}
        };

        geom::ConvexCore c;
        for (const Vec2& p : local) {
            c.add(m_center + rotate(p, m_heading));
        }
        return c;
    }

    /* A rectangle's boundary IS its core, so there is nothing to inflate. */
    double skinRadius() const override { return 0.0; }

private:
    double m_width;
    double m_height;
    double m_heading;   // stored in radians; the API speaks degrees
};


/* ===========================================================================
 * SECTION 2 -- the collision query.
 *
 * Nothing here does geometry. It decides POLICY (how close counts as
 * touching?) and then delegates the measuring to geom.
 *
 * That separation is the direct answer to the project's hint that the
 * collision math "shouldn't necessarily live within this specific function".
 * By the time control reaches isColliding(), the hard problem has already been
 * reduced to a subtraction and a comparison.
 * ========================================================================= */
namespace collision {

/* How close is "touching"?
 *
 * A policy choice, not a numerical accident, which is why it lives here rather
 * than in the geometry kernel. The problem statement asks whether robots are
 * "in contact", so exact tangency must count as a collision -- and the
 * provided testcases lean on this: r3 and r6 share the edge x = -13.5 exactly,
 * and c4 sits 0.424 from a corner of r3 against a 0.5 radius.
 *
 * A hair of positive slack absorbs rounding from the rotate/sqrt path without
 * coming anywhere near the real margins in play (the tightest genuine call in
 * the driver is 0.076 units). For an actual robot you would raise this to a
 * physical safety margin -- a centimetre or two of "close enough that I should
 * stop anyway". */
constexpr double kContactTolerance = 1e-9;

/* ---------------------------------------------------------------------------
 * Exact gap between two robots' hulls.
 *
 *   > 0  -- they are apart, and this is the true shortest distance between
 *           them. Useful in its own right: a proximity warning rather than
 *           just a yes/no, which is what you actually want when deciding
 *           whether to slow a robot down before it hits something.
 *   <= 0 -- they are in contact.
 *
 * Honest limitation: when the robots overlap, coreDistance() saturates at 0,
 * so a negative return tells you THAT they interpenetrate but not by how much.
 * Recovering true penetration depth needs a different algorithm (per-axis
 * overlap under SAT, or EPA on the Minkowski difference). I left it out
 * because nothing here asks for collision RESPONSE -- but that is the seam
 * where I would add it.
 * ------------------------------------------------------------------------- */
inline double clearance(const Robot& a, const Robot& b) {
    // The one inequality the whole system reduces to, in its subtracted form.
    return geom::coreDistance(a.core(), b.core())
         - (a.skinRadius() + b.skinRadius());
}

inline bool areColliding(const Robot& a, const Robot& b) {
    return clearance(a, b) <= kContactTolerance;
}

/* ---------------------------------------------------------------------------
 * Every colliding pair among N robots.
 *
 * The problem asks whether "any two robots are colliding", which is really
 * this question rather than the single-pair one, so it seemed worth answering
 * directly.
 *
 * Deliberately the naive all-pairs sweep. It is O(N^2), and for a field of
 * robots that is genuinely fine -- see NOTES.md for where I would take it if N
 * grew.
 * ------------------------------------------------------------------------- */
inline std::vector<std::pair<int, int>>
collidingPairs(const std::vector<const Robot*>& robots) {
    std::vector<std::pair<int, int>> hits;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            if (areColliding(*robots[i], *robots[j])) {
                hits.emplace_back(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
    return hits;
}

} // namespace collision

/* ---------------------------------------------------------------------------
 * The function the driver calls.
 *
 * A free function rather than a member, taking both operands as const Robot&,
 * so it is symmetric: neither robot is privileged as "the one being asked".
 * A member function would have implied an asymmetry that does not exist in the
 * problem.
 *
 * It is one line because every hard part has already been pushed somewhere it
 * belongs -- the shapes describe themselves (section 1), the kernel measures
 * them (geometry.hpp), and the policy layer decides what the measurement means
 * (section 2). This is the shape I was aiming for.
 * ------------------------------------------------------------------------- */
inline bool isColliding(const Robot& a, const Robot& b) {
    return collision::areColliding(a, b);
}
