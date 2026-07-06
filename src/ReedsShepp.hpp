#pragma once

#include <vector>

namespace ugv_nav4d
{

/** A single sampled pose along a Reeds-Shepp curve.
 *  Coordinates are in the same frame as the inputs to ReedsShepp::sample().
 *  @p forward encodes the driving direction of the segment this sample belongs
 *  to (true = forward, false = reverse). A change of @p forward between two
 *  consecutive samples marks a cusp (direction reversal). */
struct RSSample
{
    double x;
    double y;
    double theta;
    bool forward;
};

/** Self-contained Reeds-Shepp steering.
 *
 * Computes the shortest path between two SE(2) poses for a car-like vehicle that
 * can drive forward and backward and has a fixed minimum turning radius.
 *
 * The implementation follows the classic Reeds & Shepp (1990) closed-form
 * solution (48 word families grouped into CSC/CCC/CCCC/CCSC/CCSCC). It is kept
 * dependency-free (only <cmath>/<vector>) so it can be unit tested in isolation.
 */
class ReedsShepp
{
public:
    /** Length of the shortest Reeds-Shepp path (in the same length unit as the
     *  x/y inputs, i.e. meters) between the two poses.
     *  @return the length, or +infinity if @p turningRadius <= 0. */
    static double distance(double x0, double y0, double th0,
                           double x1, double y1, double th1,
                           double turningRadius);

    /** Sample the shortest Reeds-Shepp path from (x0,y0,th0) to (x1,y1,th1).
     *  Samples are spaced at approximately @p stepSize (meters) and always
     *  include the exact start and end pose. Each sample carries its driving
     *  direction so the caller can split the curve at cusps.
     *  @return false if @p turningRadius <= 0. */
    static bool sample(double x0, double y0, double th0,
                       double x1, double y1, double th1,
                       double turningRadius, double stepSize,
                       std::vector<RSSample>& outSamples);
};

}
