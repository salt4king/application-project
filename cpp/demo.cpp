// demo.cpp -- extended driver, in its own executable so main.cpp stays untouched.
#include "solution.hpp"

#include <cstdio>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Tiny check harness.
// ---------------------------------------------------------------------------
static int g_checks = 0;
static int g_failures = 0;

static void expect(const string& what, bool actual, bool wanted,
                   const Robot& a, const Robot& b) {
    ++g_checks;
    const bool ok = (actual == wanted);
    if (!ok) ++g_failures;
    printf("  [%s] %-46s %-13s (gap %+9.6f)\n",
                ok ? "PASS" : "FAIL",
                what.c_str(),
                actual ? "colliding" : "not colliding",
                collision::clearance(a, b));
}

static void heading(const char* title) {
    printf("\n\033[1m%s\033[0m\n", title);
    for (size_t i = 0; i < string(title).size(); ++i) putchar('-');
    putchar('\n');
}

int main() {
    // -----------------------------------------------------------------------
    // The eight cases from the provided driver, retained verbatim, but now
    // with their expected answers pinned and their exact clearances shown.
    // The gap column is the interesting part: it exposes how tight this test
    // set really is (c1/r2 collides by 0.1; c1/c3 misses by 0.36).
    // -----------------------------------------------------------------------
    CircularRobot c1( 12.5,  -2.5,  2.0);
    CircularRobot c2(  1.5,  14.5,  4.0);
    CircularRobot c3(  2.5, -22.5, 20.0);
    CircularRobot c4(-13.2,  -0.8,  0.5);

    RectangularRobot r1( -5.0,  5.0,  5.5, 13.0);
    RectangularRobot r2( 12.5,  7.5,  2.0, 16.2);
    RectangularRobot r3(-15.5,  6.5,  4.0, 14.0);
    RectangularRobot r4(  2.5,  4.5,  5.0,  9.0);
    RectangularRobot r5( -7.5, 20.5, 10.0,  4.0);
    RectangularRobot r6(-10.0,  4.5,  7.0,  3.0);

    heading("Provided testcases (with exact clearances)");
    expect("c1 / r2  circle vs tall rect, near contact", isColliding(c1, r2), true,  c1, r2);
    expect("c1 / c3  circle vs circle, near miss",       isColliding(c1, c3), false, c1, c3);
    expect("r1 / c2  rect vs circle",                    isColliding(r1, c2), false, r1, c2);
    expect("r1 / r5  rect vs rect, far apart",           isColliding(r1, r5), false, r1, r5);
    expect("r1 / r6  rect vs rect, overlapping",         isColliding(r1, r6), true,  r1, r6);
    expect("c2 / r5  circle vs rect",                    isColliding(c2, r5), false, c2, r5);
    expect("c4 / r3  circle vs rect CORNER",             isColliding(c4, r3), true,  c4, r3);
    expect("r3 / r6  rects sharing an edge EXACTLY",     isColliding(r3, r6), true,  r3, r6);

    // -----------------------------------------------------------------------
    // Cases the provided set does not reach. Each one targets a specific code
    // path that could be wrong without any of the eight above noticing.
    // -----------------------------------------------------------------------
    heading("Edge cases the provided set misses");

    // Exact tangency, both directions. The whole "is touching a collision?"
    // question, isolated.
    CircularRobot t1(0.0, 0.0, 1.0), t2(2.0, 0.0, 1.0);
    expect("circles touching at exactly one point", isColliding(t1, t2), true, t1, t2);

    CircularRobot t3(0.0, 0.0, 1.0), t4(2.001, 0.0, 1.0);
    expect("circles 0.001 apart", isColliding(t3, t4), false, t3, t4);

    // Containment. These are the cases the boundary-distance loop CANNOT see,
    // because the two boundaries are nowhere near each other -- they exist to
    // prove geom::contains() is pulling its weight.
    CircularRobot big(0.0, 0.0, 10.0), small(1.0, 0.0, 0.5);
    expect("circle entirely inside another circle", isColliding(big, small), true, big, small);

    RectangularRobot outer(0.0, 0.0, 10.0, 10.0), inner(0.0, 0.0, 2.0, 2.0);
    expect("rect entirely inside another rect", isColliding(outer, inner), true, outer, inner);

    CircularRobot swallowed(0.0, 0.0, 1.0);
    expect("circle entirely inside a rect", isColliding(outer, swallowed), true, outer, swallowed);

    RectangularRobot pebble(0.0, 0.0, 2.0, 2.0);
    expect("rect entirely inside a circle", isColliding(big, pebble), true, big, pebble);

    // The "plus sign": two rectangles crossing with NO corner of either inside
    // the other. A containment-only overlap test reports these as disjoint --
    // this is the case that forced the edge-pair loop to use segment/segment
    // distance rather than the simpler point/segment version.
    RectangularRobot vert(0.0, 0.0, 2.0, 10.0), horz(0.0, 0.0, 10.0, 2.0);
    expect("rects crossing in a plus, no corner inside", isColliding(vert, horz), true, vert, horz);

    // Corner-to-corner: the smallest possible contact patch, a single point.
    RectangularRobot k1(0.0, 0.0, 2.0, 2.0), k2(2.0, 2.0, 2.0, 2.0);
    expect("rects meeting at one corner exactly", isColliding(k1, k2), true, k1, k2);

    RectangularRobot k3(2.002, 2.002, 2.0, 2.0);
    expect("same corners, nudged apart", isColliding(k1, k3), false, k1, k3);

    // Degenerate but legal: two robots at the same place.
    CircularRobot z1(3.0, 3.0, 1.0), z2(3.0, 3.0, 1.0);
    expect("two identical robots at the same position", isColliding(z1, z2), true, z1, z2);

    // -----------------------------------------------------------------------
    // Rotation. Same centres, same dimensions -- only the heading changes, and
    // the answer flips. This is the capability the core/skin model gave us for
    // free: nothing in the geometry kernel knows what a "rotation" is.
    // -----------------------------------------------------------------------
    heading("Rotation (the extension to the required API)");

    RectangularRobot bar(0.0, 0.0, 4.0, 1.0);            // spans x in [-2, 2]
    RectangularRobot post(3.0, 0.0, 1.0, 4.0);           // spans x in [2.5, 3.5]
    expect("upright post clears the bar by 0.5", isColliding(bar, post), false, bar, post);

    RectangularRobot postTurned(3.0, 0.0, 1.0, 4.0, 90.0);   // now spans x in [1, 5]
    expect("SAME post rotated 90 deg now hits the bar", isColliding(bar, postTurned), true, bar, postTurned);

    // A square turned 45 degrees is a diamond; its corner reaches further than
    // its flat side did, which is a classic source of axis-aligned-only bugs.
    // Reach check: the square's flat side reaches x = 1.0, but its corner reaches x = sqrt(2) ~= 1.414.
    CircularRobot probe(1.8, 0.0, 0.5);
    RectangularRobot square(0.0, 0.0, 2.0, 2.0);
    expect("circle clears an axis-aligned square", isColliding(square, probe), false, square, probe);

    RectangularRobot diamond(0.0, 0.0, 2.0, 2.0, 45.0);
    expect("same square at 45 deg reaches out and hits it", isColliding(diamond, probe), true, diamond, probe);

    // -----------------------------------------------------------------------
    // Many-robot sweep. The provided driver checks 8 hand-picked pairs; this
    // asks the real operational question instead -- "who is touching whom,
    // right now?" -- across every robot on the field. Note r4 finally gets
    // used; the provided driver defines it but never tests it.
    // -----------------------------------------------------------------------
    heading("All-pairs sweep over the full field");

    const vector<string> names =
        {"c1","c2","c3","c4","r1","r2","r3","r4","r5","r6"};
    const vector<const Robot*> field =
        {&c1,&c2,&c3,&c4,&r1,&r2,&r3,&r4,&r5,&r6};

    const auto hits = collision::collidingPairs(field);
    printf("  %d robots, %d pairs examined, %d collisions found:\n",
                static_cast<int>(field.size()),
                static_cast<int>(field.size() * (field.size() - 1) / 2),
                static_cast<int>(hits.size()));
    for (const auto& [i, j] : hits) {
        printf("      %-3s <-> %-3s   (gap %+9.6f)\n",
                    names[i].c_str(), names[j].c_str(),
                    collision::clearance(*field[i], *field[j]));
    }

    heading("Result");
    printf("  %d/%d checks passed.\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
