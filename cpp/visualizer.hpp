#pragma once

#include "collision.hpp"

#include <algorithm>
#include <cstdio>
#include <ostream>
#include <string>
#include <vector>

/* ---------------------------------------------------------------------------
 * viz -- an ASCII renderer for the field.
 *
 * Motivation: a column of "is/is not colliding" booleans is impossible to
 * sanity-check by eye. If the answer for r3/r6 had been wrong, nothing in that
 * output would have told me. Drawing the field turns every testcase into
 * something I can verify at a glance, which is how I caught myself reasoning
 * about c4/r3 as an edge contact when it is really a CORNER contact.
 *
 * The nice part is that it needs no new geometry. "Is this pixel inside that
 * robot?" is just a collision query against a zero-radius point:
 *
 *      coreDistance(robot.core(), point) <= robot.skinRadius()
 *
 * which is the same inequality from geometry.hpp with one skin set to zero.
 * Containment and collision were never two different problems.
 * ------------------------------------------------------------------------- */
namespace viz {

struct Entry {
    std::string  label;
    const Robot* robot;
    char         glyph;
};

/* Point-in-robot, expressed through the existing kernel rather than reinvented. */
inline bool covers(const Robot& r, Vec2 p) {
    return geom::coreDistance(r.core(), geom::makePoint(p)) <= r.skinRadius();
}

class Field {
public:
    void add(const std::string& label, const Robot& r) {
        /* Each robot gets its own glyph from a fixed pool, resolved by the
         * legend below.
         *
         * My first attempt derived the glyph from the label ('c1' -> '1'),
         * which read nicely until the field contained both c1 and r1 and they
         * drew as the same character -- the picture silently lied about which
         * robot was where. A pool indexed by insertion order cannot collide. */
        static const char kPool[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        const std::size_t i = m_entries.size();
        const char glyph = (i < sizeof(kPool) - 1) ? kPool[i] : '?';
        m_entries.push_back({label, &r, glyph});
    }

    /* Restrict the drawing to an explicit window.
     *
     * Needed because auto-framing the whole field puts roughly 0.6 world units
     * in every cell, and some of the most interesting contacts here are far
     * thinner than that -- c1 and r2 genuinely overlap, but only in a band
     * 0.1 units tall, which auto-framing cannot resolve and therefore draws as
     * a clean miss. Rather than let the picture quietly disagree with the
     * numbers, the caller can zoom in until the contact is actually on screen. */
    void setBounds(double minX, double maxX, double minY, double maxY) {
        m_minX = minX; m_maxX = maxX; m_minY = minY; m_maxY = maxY;
        m_explicitBounds = true;
    }

    void render(std::ostream& os, int maxCols = 108, int maxRows = 30) const {
        if (m_entries.empty()) return;

        // --- work out the world-space window to draw
        double minX, maxX, minY, maxY;
        if (m_explicitBounds) {
            minX = m_minX; maxX = m_maxX; minY = m_minY; maxY = m_maxY;
        } else {
            // Auto-frame: the bounding box of everything, plus a small margin.
            minX = 1e300; maxX = -1e300; minY = 1e300; maxY = -1e300;
            for (const Entry& e : m_entries) {
                const Vec2   c = e.robot->position();
                const double b = e.robot->boundingRadius();
                minX = std::min(minX, c.x - b); maxX = std::max(maxX, c.x + b);
                minY = std::min(minY, c.y - b); maxY = std::max(maxY, c.y + b);
            }
            const double pad = 0.04 * std::max(maxX - minX, maxY - minY) + 0.5;
            minX -= pad; maxX += pad; minY -= pad; maxY += pad;
        }

        const double worldW = maxX - minX;
        const double worldH = maxY - minY;

        // Uniform scale, halved on X because a terminal cell is about twice as
        // tall as it is wide -- without this correction every circle renders as
        // an ellipse and the picture stops being trustworthy.
        const double rowsPerUnit =
            std::min(maxRows / worldH, (maxCols * 0.5) / worldW);
        const int rows = std::max(3, static_cast<int>(worldH * rowsPerUnit));
        const int cols = std::max(3, static_cast<int>(worldW * rowsPerUnit * 2.0));

        for (int row = 0; row < rows; ++row) {
            std::string line;
            for (int col = 0; col < cols; ++col) {
                // +0.5 samples the CENTRE of each cell rather than its corner.
                const double x = minX + (col + 0.5) / (rowsPerUnit * 2.0);
                const double y = maxY - (row + 0.5) / rowsPerUnit;   // y grows up

                int   hits  = 0;
                char  glyph = ' ';
                for (const Entry& e : m_entries) {
                    if (covers(*e.robot, {x, y})) { ++hits; glyph = e.glyph; }
                }
                // A cell claimed by two robots at once is, by definition, a
                // region where they overlap -- so '#' marks collisions visually.
                line += (hits > 1) ? '#' : glyph;
            }
            // Trailing blanks are noise in a terminal; strip them.
            while (!line.empty() && line.back() == ' ') line.pop_back();
            os << "  " << line << '\n';
        }

        // Reporting the cell size keeps the drawing honest: any overlap thinner
        // than this simply cannot appear, and the reader should know that.
        char scale[96];
        std::snprintf(scale, sizeof(scale), "  (cell = %.3f x %.3f world units)\n",
                      1.0 / (rowsPerUnit * 2.0), 1.0 / rowsPerUnit);

        os << "\n  legend: ";
        for (const Entry& e : m_entries) os << e.label << "=" << e.glyph << "  ";
        os << " '#' = overlapping\n" << scale;
    }

private:
    std::vector<Entry> m_entries;
    bool   m_explicitBounds = false;
    double m_minX = 0.0, m_maxX = 0.0, m_minY = 0.0, m_maxY = 0.0;
};

} // namespace viz
