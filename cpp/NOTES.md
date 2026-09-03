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

Two headers, split on the one boundary that carries meaning — measuring versus
modelling. The project's hint warns against putting collision math inside
`isColliding()`, and keeping the math in its own file makes it structurally
awkward to drift back there.

| file | role |
|------|------|
| `geometry.hpp` | `Vec2`, plus the convex-core distance kernel — **all** the math |
| `robots.hpp` | `Robot` / `CircularRobot` / `RectangularRobot`, and `isColliding()` |
| `solution.hpp` | the single include the driver needs |
| `main.cpp` | **untouched**, exactly as provided |
| `demo.cpp` | extended, self-checking driver |

Each header is split into two clearly marked sections, so the four original
layers are still visible: `geometry.hpp` is `Vec2` then `namespace geom`, and
`robots.hpp` is the robot classes then `namespace collision`. An earlier
version had those four layers as four separate files. That was tidier in the
abstract, but ~190 lines of code spread over six files reads as ceremony, and
the namespaces already enforce the separation that mattered.

### What each file contains

**`geometry.hpp`** — two sections. Section 1 is `Vec2`: a 2D vector with `dot`,
`cross`, `lengthSquared`, `distance` and `rotate`. Section 2 is `namespace
geom`: `ConvexCore` (a fixed-capacity convex hull, max 8 vertices, so a
collision query never allocates) and the three primitives that compose into
`coreDistance`.

**`robots.hpp`** — two sections. Section 1 is the model: `Robot` (abstract,
demanding only `core()` and `skinRadius()`), `CircularRobot` and
`RectangularRobot`. Section 2 is `namespace collision`: `kContactTolerance`,
`clearance`, `areColliding`, `collidingPairs`, then the global `isColliding`
the driver calls.

**`solution.hpp`** — two lines. `#pragma once` and `#include "robots.hpp"`.
Its only job is exposing the API to `main.cpp` so that file never changes.

**`demo.cpp`** — the extended driver, built as a separate executable so the
provided one stays untouched. Self-checking: every case declares its expected
result and the program exits non-zero if any disagree.

### Adding a testcase

Add an `expect(...)` line to `demo.cpp`, then `make run-demo`:

```cpp
CircularRobot    a(0.0, 0.0, 1.0);
RectangularRobot b(1.5, 0.0, 2.0, 2.0, 30.0);   // last arg = heading, optional
expect("short description of the case", isColliding(a, b), /*expected*/ true, a, b);
```

`expect` prints PASS/FAIL plus the exact clearance, and tallies failures into
the program's exit code. The provided eight cases in `main.cpp` are not touched.

## How it is built, and the correctness argument

The kernel is three primitives, each a few lines of plain vector algebra:

| primitive | what it does |
|-----------|--------------|
| `pointSegmentDistance` | project a point onto a segment, clamp to the ends, measure |
| `segmentsCross` | do two segments straddle each other? (sign of a cross product) |
| `contains` | is a point inside a convex polygon? (same side of every edge) |

`coreDistance` composes them by asking three questions in order:

1. **Does either core contain a vertex of the other?** Catches total
   containment — one robot swallowed by another, boundaries nowhere near each
   other. Step 3 cannot see this case at all, so the check is not redundant.
   Testing one vertex suffices: if the boundaries do not cross, then either
   every vertex of one core is inside the other or none is.
2. **Do any edges cross?** Catches partial overlap, including two rectangles
   crossing in a plus sign where *neither* has a corner inside the other yet
   they plainly overlap. `demo.cpp` pins that case specifically.
3. **Otherwise they are disjoint,** and the shortest distance between two
   disjoint convex polygons always has at least one endpoint at a vertex — so
   checking every vertex against every edge of the other shape, in both
   directions, finds it exactly.

Steps 1 and 2 together are a *complete* overlap test, because two convex shapes
can only overlap by containment or by crossing boundaries.

One subtlety worth being able to defend: `segmentsCross` is a **strict** test,
so segments that merely touch, or lie along one another, are reported as *not*
crossing. That is deliberate. In every such case a vertex of one shape lies on
the other's boundary, and `contains` already counts that as inside — so the
case is handled, just by a different step. Splitting the two tests this way
avoids the fiddly collinear special cases that make the textbook
segment-intersection routine unpleasant.

I first wrote this using the standard segment-to-segment distance routine,
which solves a constrained minimisation over the unit square and handles all of
the above in one pass. It works, and it is fewer moving parts — but it is much
harder to derive from scratch, and I would rather ship something I can defend
on a whiteboard than something I can only cite. The two versions were checked
against each other over 400,000 random shape pairs and 106,000 lattice-aligned
ones (chosen so exact shared edges and corners occur constantly, since random
coordinates essentially never produce them). They agree on every case, worst
difference 2.7e-15.

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
