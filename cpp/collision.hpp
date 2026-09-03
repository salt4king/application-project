#pragma once

#include "robot.hpp"

#include <utility>
#include <vector>

/* ---------------------------------------------------------------------------
 * collision -- the query layer.
 *
 * Nothing in this file does geometry. It decides POLICY (how close counts as
 * touching?) and it decides STRATEGY (how do we avoid doing expensive work?),
 * then delegates the actual math to geom.
 *
 * That separation is the direct answer to the project's hint that the
 * collision math "shouldn't necessarily live within this specific function".
 * By the time control reaches isColliding(), the hard problem has already been
 * reduced to a subtraction and a comparison.
 * ------------------------------------------------------------------------- */
namespace collision {

/* How close is "touching"?
 *
 * This is a policy choice, not a numerical accident, which is why it lives
 * here rather than in the geometry kernel. The problem statement asks whether
 * robots are "in contact", so exact tangency must count as a collision -- and
 * the provided testcases lean on this: r3 and r6 share the edge x = -13.5
 * exactly, and c4 sits 0.424 from a corner of r3 against a 0.5 radius.
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
 *           them. Useful in its own right: it is a proximity warning, not just
 *           a yes/no, which is what you actually want when deciding whether to
 *           slow a robot down before it hits something.
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

/* ---------------------------------------------------------------------------
 * Broad phase: a cheap, conservative reject.
 *
 * Every robot fits inside a circle of boundingRadius() about its centre. If
 * two such circles are disjoint the robots cannot possibly touch, and we skip
 * building hulls and running up to 16 segment tests.
 *
 * Compared in SQUARED space to dodge a sqrt. The comparison is strict (>) so
 * that bounding circles which exactly graze still fall through to the exact
 * test -- when in doubt, this stage must always defer rather than decide.
 *
 * With two robots this is a micro-optimisation. Its real value shows up in
 * collidingPairs() below, where the work is quadratic in the robot count.
 * ------------------------------------------------------------------------- */
inline bool broadPhaseReject(const Robot& a, const Robot& b) {
    const double reach = a.boundingRadius() + b.boundingRadius();
    return lengthSquared(a.position() - b.position()) > reach * reach;
}

inline bool areColliding(const Robot& a, const Robot& b) {
    if (broadPhaseReject(a, b)) return false;          // cheap, conservative
    return clearance(a, b) <= kContactTolerance;       // exact
}

/* ---------------------------------------------------------------------------
 * Every colliding pair among N robots.
 *
 * The naive all-pairs sweep, but with the broad phase in front of it -- which
 * is where that stage earns its keep, since it turns most of the N^2 pairs
 * into a subtraction and a compare. Beyond a few hundred robots the next step
 * would be a spatial hash or a sweep-and-prune over the bounding circles, so
 * that distant robots are never enumerated at all.
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
 * Free function rather than a member, and taking both operands as const Robot&,
 * so it is symmetric: neither robot is privileged as "the one being asked".
 * A member function would have implied an asymmetry that does not exist in the
 * problem.
 *
 * It is one line because every hard part has already been pushed somewhere it
 * belongs -- the shapes describe themselves (robot.hpp), the kernel measures
 * them (geometry.hpp), and the policy layer decides what the measurement means
 * (above). This is the shape I was aiming for.
 * ------------------------------------------------------------------------- */
inline bool isColliding(const Robot& a, const Robot& b) {
    return collision::areColliding(a, b);
}
