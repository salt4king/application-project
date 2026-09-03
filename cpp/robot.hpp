#pragma once

#include "geometry.hpp"

#include <string>

/* ---------------------------------------------------------------------------
 * Robot -- the abstract base every chassis derives from.
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
 * So instead the base class asks each subclass only to DESCRIBE ITSELF, in the
 * one vocabulary the geometry kernel understands:
 *
 *      core()        -- its convex hull of points, in world space
 *      skinRadius()  -- how far that hull is inflated
 *
 * A subclass never implements a collision test. It answers two questions about
 * its own shape and the kernel does the rest. Adding a new chassis is one new
 * class and zero edits anywhere else -- that is the payoff.
 * ------------------------------------------------------------------------- */
class Robot {
public:
    Robot(double x, double y) : m_center(x, y) {}

    /* Virtual destructor: these are polymorphic types held by reference (and,
     * in the demo, by base-class pointer), so this is non-negotiable. */
    virtual ~Robot() = default;

    Vec2 position() const { return m_center; }

    void moveTo(Vec2 p) { m_center = p; }
    void translate(Vec2 delta) { m_center = m_center + delta; }

    /* --- the two questions every chassis must answer --- */

    /* The convex core, in WORLD space. World rather than local space keeps the
     * kernel free of transform plumbing: by the time geometry sees a shape,
     * the shape has already placed itself on the field. */
    virtual geom::ConvexCore core() const = 0;

    /* How far the core is inflated. Zero for a polygonal chassis. */
    virtual double skinRadius() const = 0;

    /* Radius of a circle centred on position() that fully encloses the robot.
     * Used purely as a cheap broad-phase reject (see collision.hpp). It is
     * always safe to overestimate this; never underestimate it. */
    virtual double boundingRadius() const = 0;

    virtual std::string describe() const = 0;

protected:
    Vec2 m_center;
};

/* ---------------------------------------------------------------------------
 * CircularRobot -- a point, inflated by its radius.
 *
 * The entire shape lives in skinRadius(). Its core is a single vertex, which
 * is why the degenerate-segment handling in the kernel matters so much: this
 * class only works because a 1-vertex core is a first-class citizen there.
 * ------------------------------------------------------------------------- */
class CircularRobot : public Robot {
public:
    CircularRobot(double x, double y, double radius)
        : Robot(x, y), m_radius(radius) {}

    double radius() const { return m_radius; }

    geom::ConvexCore core() const override {
        return geom::makePoint(m_center);
    }

    double skinRadius() const override { return m_radius; }

    double boundingRadius() const override { return m_radius; }

    std::string describe() const override {
        return "circle r=" + std::to_string(m_radius);
    }

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
 * free (rotate the four corners; the kernel neither knows nor cares). Solving
 * only the axis-aligned case here would have been leaving capability on the
 * table for no saving in complexity.
 * ------------------------------------------------------------------------- */
class RectangularRobot : public Robot {
public:
    RectangularRobot(double x, double y, double width, double height,
                     double headingDegrees = 0.0)
        : Robot(x, y),
          m_width(width),
          m_height(height),
          m_heading(headingDegrees * 3.14159265358979323846 / 180.0) {}

    double width()  const { return m_width; }
    double height() const { return m_height; }
    double headingDegrees() const {
        return m_heading * 180.0 / 3.14159265358979323846;
    }

    geom::ConvexCore core() const override {
        const double hw = m_width  * 0.5;
        const double hh = m_height * 0.5;

        // Listed counter-clockwise. Winding is documented rather than relied
        // upon -- geom::contains() is order-agnostic -- but keeping it
        // consistent makes the shape easy to reason about and to draw.
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

    /* Half the diagonal -- the corner is the farthest point from the centre.
     * Note this is invariant under rotation, so a spinning robot does not need
     * its broad-phase bound recomputed. */
    double boundingRadius() const override {
        const double hw = m_width  * 0.5;
        const double hh = m_height * 0.5;
        return std::sqrt(hw * hw + hh * hh);
    }

    std::string describe() const override {
        return "rect " + std::to_string(m_width) + "x" + std::to_string(m_height);
    }

private:
    double m_width;
    double m_height;
    double m_heading;   // stored in radians; the API speaks degrees
};
