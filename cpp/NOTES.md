# Design notes

## The idea

The straightforward way to solve this is one routine per pair of shape types:
circle/circle, circle/rectangle, rectangle/rectangle. That works, and for two
shapes it is honestly fine. What bothered me is how it ages. Shape types need
`N*(N+1)/2` routines between them, so adding a hexagonal or capsule chassis
later means writing three or four new functions and re-opening every class that
already exists. The collision logic ends up smeared across the type hierarchy.

So I looked for what the two shapes have in common, and there is a clean answer:

|           | core (a convex set of points) | skin (inflation radius) |
|-----------|-------------------------------|-------------------------|
| circle    | one point, its centre         | its radius              |
| rectangle | its four corners              | zero                    |

**A circle is a point with thickness. A rectangle is four points with no
thickness.** Both are a convex polygon grown outward by some radius — formally
a Minkowski sum of a convex polygon with a disc.

Once every chassis is described that way, collision stops being a case analysis
and becomes a single inequality:

```
distance(coreA, coreB)  <=  skinA + skinB
```

Inflating a set by `r` pushes its boundary out by exactly `r` in every
direction, so two inflated shapes touch precisely when the gap between their
cores has been consumed by the two skins. That inequality is the entire
collision system — everything else is machinery for evaluating its left side.

What this buys:

- **One code path.** No branching on shape type anywhere in the geometry.
- **New shapes are nearly free.** A capsule is two core points with a skin. A
  rounded-corner bumper is four core points with a skin. An octagonal chassis is
  eight core points. Each is one new class and *zero* edits to existing code.
- **Rotation is free.** Rotate the core's points; the kernel neither knows nor
  cares. That is why `RectangularRobot` can take a heading, and it is the
  cheapest available proof that the abstraction is real — if rotation had
  required touching the kernel, the claim that the kernel is blind to shape
  would have been false.

This is the same principle real physics engines use (Box2D gives polygons a
radius; GJK reduces every convex pair to one support-function loop). I chose the
explicit polygon-distance form over full GJK because it is exact, it is about
sixty lines, and I can defend every one of them.

## Why not double dispatch

The classic OO answer here is a virtual `collidesWith(const Robot&)` resolved by
the visitor pattern. I rejected it deliberately: it reintroduces the same
`O(N^2)` coupling, just expressed in vtables instead of `if`-statements, and it
puts the math back inside the robot classes — the thing the project's hint warns
against.

A subclass never implements a collision test. It answers exactly two questions
about itself — `core()` and `skinRadius()` — and that is the whole contract.
It is why `isColliding()` is a one-liner.

## Layout

Strictly bottom-up; each header depends only on the one beneath it, which is
why one `#include` in `solution.hpp` is enough.

| file | role |
|------|------|
| `vec2.hpp` | 2D vector primitives |
| `geometry.hpp` | the convex-core distance kernel — **all** the math |
| `robot.hpp` | `Robot`, `CircularRobot`, `RectangularRobot` |
| `collision.hpp` | `clearance()`, `isColliding()` — policy, not math |
| `solution.hpp` | the single include the driver needs |
| `main.cpp` | **untouched**, exactly as provided |
| `demo.cpp` | extended, self-checking driver |

## Correctness argument

The one genuinely subtle part is `geom::coreDistance`. Two convex sets overlap
in exactly one of two ways, and both must be caught:

1. **Their boundaries cross.** Handled by the edge-pair loop, since two crossing
   segments are at distance 0.
2. **One is entirely inside the other, boundaries never touching.** Invisible to
   the edge loop — all those edges are far apart — so it needs the explicit
   containment test.

Case 2 is why the containment check is not redundant, and case 1 is why the loop
uses full *segment-to-segment* distance rather than the simpler
*point-to-segment* version. My first sketch used point-to-segment, which is
correct for two **disjoint** convex polygons but fails on two rectangles crossing
in a plus sign: no corner of either is inside the other, yet they plainly
overlap. `demo.cpp` pins that case specifically.

Degenerate cores are handled by construction rather than by special-casing. A
core's edges are `(v[i], v[(i+1) % n])`, so a 1-vertex core yields the
degenerate edge `(v0, v0)` and the segment routine — which tolerates zero-length
segments — collapses to a plain point-to-point distance. Circle-vs-circle
therefore falls out of the general polygon code as `|c1 - c2| <= r1 + r2`, the
textbook formula, without anyone having written it.

## Touching counts as colliding

The problem asks whether robots are "in contact", so exact tangency is a
collision. The provided testcases lean on this hard: `r3` and `r6` share the
edge `x = -13.5` exactly, and the answer must be *colliding*.

`kContactTolerance` lives in `collision.hpp` rather than the geometry kernel
because it is a **policy** decision, not a numerical one. It is set to `1e-9`,
which absorbs rounding from the rotate/sqrt path while staying far below the
real margins in play — the tightest genuine call in the provided set is 0.076.
On a real robot I would raise it to a physical safety margin.

## Beyond a boolean

`collision::clearance()` returns the exact shortest distance between two robots
when they are apart, not just a yes/no. That is the more useful quantity in
practice — it answers "how close am I?", which is what you need to slow a robot
*before* it hits something. `demo.cpp` prints it for every case, which is also
what exposed how tight this test set is.

## Known limitations

Being explicit about what this does **not** do:

- **No penetration depth.** When robots overlap, `coreDistance` saturates at 0,
  so a negative clearance says *that* they interpenetrate, not by how much.
  Recovering true depth needs per-axis overlap under SAT, or EPA on the
  Minkowski difference. Nothing here asks for collision *response*, so I left it
  out — but that is the seam where it would go.
- **Convex chassis only.** A concave robot must be decomposed into convex parts.
  This is the standard trade and every mainstream engine makes it.
- **`collidingPairs()` is `O(N^2)`.** Fine for a field of robots. If N grew I
  would put a cheap bounding-circle reject in front of the exact test, and past
  a few hundred robots switch to a spatial hash or sweep-and-prune so distant
  robots are never paired at all.
- **`ConvexCore` caps at 8 vertices,** deliberately, to keep collision queries
  allocation-free. Ample for any real chassis; it would need to change for
  arbitrary polygons.

## A note on the build

The provided `Makefile` invokes bare `g++` with no `-std` flag. On macOS `g++`
is Apple clang, which **still defaults to C++98** (`__cplusplus == 199711`), and
in that mode even `<new>` fails to parse — no modern C++ can build at all. I
added `-std=c++17` (plus `-Wall -Wextra`, since a geometry kernel is exactly
where a dropped term hides). This is a fix, not a preference.

## Running it

```
make run        # the required driver, exactly as provided
make run-demo   # extended driver: edge cases, rotation, all-pairs sweep
make clean
```

`main.cpp` is byte-for-byte unmodified, per `cpp/README.md`. The root README
separately invites additions to the driver, so that work lives in `demo.cpp` as
a second executable — the required driver stays provably pristine either way.

`run_demo` **checks itself**: every case declares its expected result and the
program exits non-zero if any disagree (currently 22/22). Printing booleans and
eyeballing them is fine for eight cases, but it does not protect a refactor of
the geometry kernel. The cases it adds beyond the provided eight each target a
code path that could be wrong without any of those eight noticing:

- circles touching at exactly one point, and 0.001 apart
- circle inside circle, rect inside rect, circle inside rect, rect inside circle
  (the containment path)
- two rects crossing in a plus with no corner inside either (the crossing-edge
  path)
- rects meeting at exactly one corner, and nudged apart
- two identical robots at the same position
- a rectangle whose 90° rotation flips the answer, at fixed centre and size
- a square whose 45° rotation reaches past a circle its flat side cleared
- an all-pairs sweep over all ten robots (which finally uses `r4`, defined but
  never tested by the provided driver)
