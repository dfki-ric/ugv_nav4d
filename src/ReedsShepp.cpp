#include "ReedsShepp.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

/* The closed-form Reeds-Shepp solver below is a self-contained reimplementation
 * of the classic algorithm (Reeds, J.A. & Shepp, L.A., "Optimal paths for a car
 * that goes both forwards and backwards", Pacific J. Math, 1990). The structure
 * (word families, time-flip / reflect / backwards symmetries) mirrors the
 * widely used public-domain formulation. It intentionally has no external
 * dependencies so it can be verified in isolation. */

namespace ugv_nav4d
{

namespace
{
const double RS_PI    = M_PI;
const double RS_TWOPI = 2.0 * M_PI;
const double RS_ZERO  = 10.0 * std::numeric_limits<double>::epsilon();

/** Segment steering of one Reeds-Shepp word element. */
enum RSSeg
{
    RS_NOP = 0,
    RS_LEFT,
    RS_STRAIGHT,
    RS_RIGHT
};

/** The 18 canonical word patterns (up to 5 segments each). */
const RSSeg RS_TYPES[18][5] = {
    {RS_LEFT,  RS_RIGHT,    RS_LEFT,     RS_NOP,      RS_NOP},   //  0  CCC
    {RS_RIGHT, RS_LEFT,     RS_RIGHT,    RS_NOP,      RS_NOP},   //  1
    {RS_LEFT,  RS_RIGHT,    RS_LEFT,     RS_RIGHT,    RS_NOP},   //  2  CCCC
    {RS_RIGHT, RS_LEFT,     RS_RIGHT,    RS_LEFT,     RS_NOP},   //  3
    {RS_LEFT,  RS_RIGHT,    RS_STRAIGHT, RS_LEFT,     RS_NOP},   //  4  CCSC
    {RS_RIGHT, RS_LEFT,     RS_STRAIGHT, RS_RIGHT,    RS_NOP},   //  5
    {RS_LEFT,  RS_STRAIGHT, RS_RIGHT,    RS_LEFT,     RS_NOP},   //  6
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT,     RS_RIGHT,    RS_NOP},   //  7
    {RS_LEFT,  RS_RIGHT,    RS_STRAIGHT, RS_RIGHT,    RS_NOP},   //  8
    {RS_RIGHT, RS_LEFT,     RS_STRAIGHT, RS_LEFT,     RS_NOP},   //  9
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT,    RS_LEFT,     RS_NOP},   // 10
    {RS_LEFT,  RS_STRAIGHT, RS_LEFT,     RS_RIGHT,    RS_NOP},   // 11
    {RS_LEFT,  RS_STRAIGHT, RS_RIGHT,    RS_NOP,      RS_NOP},   // 12  CSC
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT,     RS_NOP,      RS_NOP},   // 13
    {RS_LEFT,  RS_STRAIGHT, RS_LEFT,     RS_NOP,      RS_NOP},   // 14
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT,    RS_NOP,      RS_NOP},   // 15
    {RS_LEFT,  RS_RIGHT,    RS_STRAIGHT, RS_LEFT,     RS_RIGHT}, // 16  CCSCC
    {RS_RIGHT, RS_LEFT,     RS_STRAIGHT, RS_RIGHT,    RS_LEFT}   // 17
};

struct RSPath
{
    const RSSeg* type;
    double len[5];

    RSPath()
        : type(RS_TYPES[14])
    {
        len[0] = std::numeric_limits<double>::infinity();
        len[1] = len[2] = len[3] = len[4] = 0.0;
    }

    RSPath(const RSSeg* t, double a, double b, double c, double d = 0.0, double e = 0.0)
        : type(t)
    {
        len[0] = a; len[1] = b; len[2] = c; len[3] = d; len[4] = e;
    }

    double length() const
    {
        return std::fabs(len[0]) + std::fabs(len[1]) + std::fabs(len[2]) +
               std::fabs(len[3]) + std::fabs(len[4]);
    }

    /** Number of direction reversals (cusps / gear changes) along the path. */
    unsigned int cusps() const
    {
        unsigned int n = 0;
        double prev = 0.0;
        for (int i = 0; i < 5; ++i)
        {
            if (type[i] == RS_NOP || std::fabs(len[i]) <= RS_ZERO)
                continue;
            if (prev != 0.0 && ((len[i] > 0.0) != (prev > 0.0)))
                ++n;
            prev = len[i];
        }
        return n;
    }
};

/** Keep the better of the two candidate paths in @p best. Selection is
 *  lexicographic: fewest direction reversals first (a cusp means stopping and
 *  switching gear, which is expensive to execute on a real vehicle), shortest
 *  length as tie-break.
 *
 *  With @p forwardOnly, reverse segments are not allowed. A reverse *arc* of
 *  angle a can be replaced by a forward arc of 2*pi - a on the same steering
 *  circle (same endpoint and heading), so it is lifted instead of rejected;
 *  a reverse *straight* has no forward equivalent and rejects the candidate. */
inline void consider(RSPath& best, bool forwardOnly, RSPath cand)
{
    if (forwardOnly)
    {
        for (int i = 0; i < 5; ++i)
        {
            if (cand.type[i] == RS_NOP || cand.len[i] >= -RS_ZERO)
                continue;
            if (cand.type[i] == RS_STRAIGHT)
                return;
            cand.len[i] += RS_TWOPI;
        }
    }
    if (!std::isfinite(best.length()))
    {
        best = cand;
        return;
    }
    const unsigned int cc = cand.cusps();
    const unsigned int bc = best.cusps();
    if (cc < bc || (cc == bc && cand.length() < best.length()))
        best = cand;
}

inline double mod2pi(double x)
{
    double v = std::fmod(x, RS_TWOPI);
    if (v < -RS_PI)
        v += RS_TWOPI;
    else if (v > RS_PI)
        v -= RS_TWOPI;
    return v;
}

inline void polar(double x, double y, double& r, double& theta)
{
    r = std::hypot(x, y);
    theta = std::atan2(y, x);
}

inline void tauOmega(double u, double v, double xi, double eta, double phi,
                     double& tau, double& omega)
{
    double delta = mod2pi(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = (t2 < 0) ? mod2pi(t1 + RS_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
}

/* ---- CSC ---- */
inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v)
{
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
    if (t >= -RS_ZERO)
    {
        v = mod2pi(phi - t);
        if (v >= -RS_ZERO)
            return true;
    }
    return false;
}

inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v)
{
    double t1, u1;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.0)
    {
        double theta;
        u = std::sqrt(u1 - 4.0);
        theta = std::atan2(2.0, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        if (t >= -RS_ZERO && v >= -RS_ZERO)
            return true;
    }
    return false;
}

void CSC(double x, double y, double phi, bool forwardOnly, RSPath& path)
{
    double t, u, v;
    if (LpSpLp(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[14], t, u, v));
    if (LpSpLp(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[14], -t, -u, -v));
    if (LpSpLp(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[15], t, u, v));
    if (LpSpLp(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[15], -t, -u, -v));
    if (LpSpRp(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[12], t, u, v));
    if (LpSpRp(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[12], -t, -u, -v));
    if (LpSpRp(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[13], t, u, v));
    if (LpSpRp(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[13], -t, -u, -v));
}

/* ---- CCC ---- */
inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi), u1, theta;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.0)
    {
        u = -2.0 * std::asin(0.25 * u1);
        t = mod2pi(theta + 0.5 * u + RS_PI);
        v = mod2pi(phi - t + u);
        if (t >= -RS_ZERO && u <= RS_ZERO)
            return true;
    }
    return false;
}

void CCC(double x, double y, double phi, bool forwardOnly, RSPath& path)
{
    double t, u, v;
    if (LpRmL(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[0], t, u, v));
    if (LpRmL(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[0], -t, -u, -v));
    if (LpRmL(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[1], t, u, v));
    if (LpRmL(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[1], -t, -u, -v));

    // backwards
    double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[0], v, u, t));
    if (LpRmL(-xb, yb, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[0], -v, -u, -t));
    if (LpRmL(xb, -yb, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[1], v, u, t));
    if (LpRmL(-xb, -yb, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[1], -v, -u, -t));
}

/* ---- CCCC ---- */
inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi),
           rho = 0.25 * (2.0 + std::hypot(xi, eta));
    if (rho <= 1.0)
    {
        u = std::acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        if (t >= -RS_ZERO && v <= RS_ZERO)
            return true;
    }
    return false;
}

inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi),
           rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho >= 0.0 && rho <= 1.0)
    {
        u = -std::acos(rho);
        if (u >= -0.5 * RS_PI)
        {
            tauOmega(u, u, xi, eta, phi, t, v);
            if (t >= -RS_ZERO && v >= -RS_ZERO)
                return true;
        }
    }
    return false;
}

void CCCC(double x, double y, double phi, bool forwardOnly, RSPath& path)
{
    double t, u, v;
    if (LpRupLumRm(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[2], t, u, -u, v));
    if (LpRupLumRm(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[2], -t, -u, u, -v));
    if (LpRupLumRm(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[3], t, u, -u, v));
    if (LpRupLumRm(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[3], -t, -u, u, -v));

    if (LpRumLumRp(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[2], t, u, u, v));
    if (LpRumLumRp(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[2], -t, -u, -u, -v));
    if (LpRumLumRp(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[3], t, u, u, v));
    if (LpRumLumRp(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[3], -t, -u, -u, -v));
}

/* ---- CCSC ---- */
inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0)
    {
        double r = std::sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + std::atan2(r, -2.0));
        v = mod2pi(phi - 0.5 * RS_PI - t);
        if (t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO)
            return true;
    }
    return false;
}

inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi), rho, theta;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.0)
    {
        t = theta;
        u = 2.0 - rho;
        v = mod2pi(t + 0.5 * RS_PI - phi);
        if (t >= -RS_ZERO && u <= RS_ZERO && v <= RS_ZERO)
            return true;
    }
    return false;
}

void CCSC(double x, double y, double phi, bool forwardOnly, RSPath& path)
{
    double t, u, v;
    if (LpRmSmLm(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[4], t, -0.5 * RS_PI, u, v));
    if (LpRmSmLm(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[4], -t, 0.5 * RS_PI, -u, -v));
    if (LpRmSmLm(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[5], t, -0.5 * RS_PI, u, v));
    if (LpRmSmLm(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[5], -t, 0.5 * RS_PI, -u, -v));

    if (LpRmSmRm(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[8], t, -0.5 * RS_PI, u, v));
    if (LpRmSmRm(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[8], -t, 0.5 * RS_PI, -u, -v));
    if (LpRmSmRm(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[9], t, -0.5 * RS_PI, u, v));
    if (LpRmSmRm(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[9], -t, 0.5 * RS_PI, -u, -v));

    // backwards
    double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[6], v, u, -0.5 * RS_PI, t));
    if (LpRmSmLm(-xb, yb, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[6], -v, -u, 0.5 * RS_PI, -t));
    if (LpRmSmLm(xb, -yb, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[7], v, u, -0.5 * RS_PI, t));
    if (LpRmSmLm(-xb, -yb, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[7], -v, -u, 0.5 * RS_PI, -t));

    if (LpRmSmRm(xb, yb, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[10], v, u, -0.5 * RS_PI, t));
    if (LpRmSmRm(-xb, yb, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[10], -v, -u, 0.5 * RS_PI, -t));
    if (LpRmSmRm(xb, -yb, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[11], v, u, -0.5 * RS_PI, t));
    if (LpRmSmRm(-xb, -yb, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[11], -v, -u, 0.5 * RS_PI, -t));
}

/* ---- CCSCC ---- */
inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v)
{
    double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.0)
    {
        u = 4.0 - std::sqrt(rho * rho - 4.0);
        if (u <= RS_ZERO)
        {
            t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = mod2pi(t - phi);
            if (t >= -RS_ZERO && v >= -RS_ZERO)
                return true;
        }
    }
    return false;
}

void CCSCC(double x, double y, double phi, bool forwardOnly, RSPath& path)
{
    double t, u, v;
    if (LpRmSLmRp(x, y, phi, t, u, v))
        consider(path, forwardOnly, RSPath(RS_TYPES[16], t, -0.5 * RS_PI, u, -0.5 * RS_PI, v));
    if (LpRmSLmRp(-x, y, -phi, t, u, v))  // timeflip
        consider(path, forwardOnly, RSPath(RS_TYPES[16], -t, 0.5 * RS_PI, -u, 0.5 * RS_PI, -v));
    if (LpRmSLmRp(x, -y, -phi, t, u, v))  // reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[17], t, -0.5 * RS_PI, u, -0.5 * RS_PI, v));
    if (LpRmSLmRp(-x, -y, phi, t, u, v))  // timeflip + reflect
        consider(path, forwardOnly, RSPath(RS_TYPES[17], -t, 0.5 * RS_PI, -u, 0.5 * RS_PI, -v));
}

RSPath reedsShepp(double x, double y, double phi, bool forwardOnly)
{
    RSPath path;
    CSC(x, y, phi, forwardOnly, path);
    CCC(x, y, phi, forwardOnly, path);
    CCCC(x, y, phi, forwardOnly, path);
    CCSC(x, y, phi, forwardOnly, path);
    CCSCC(x, y, phi, forwardOnly, path);
    return path;
}

/** Transform the goal pose into the start-relative, radius-normalized frame in
 *  which the closed-form solver operates, and return the resulting path. */
RSPath normalizedPath(double x0, double y0, double th0,
                      double x1, double y1, double th1, double radius,
                      bool forwardOnly)
{
    double dx = x1 - x0, dy = y1 - y0;
    double c = std::cos(th0), s = std::sin(th0);
    double x = (c * dx + s * dy) / radius;
    double y = (-s * dx + c * dy) / radius;
    double phi = th1 - th0;
    return reedsShepp(x, y, phi, forwardOnly);
}

}  // namespace

double ReedsShepp::distance(double x0, double y0, double th0,
                            double x1, double y1, double th1,
                            double turningRadius)
{
    if (turningRadius <= 0.0)
        return std::numeric_limits<double>::infinity();
    return normalizedPath(x0, y0, th0, x1, y1, th1, turningRadius, false).length() * turningRadius;
}

bool ReedsShepp::sample(double x0, double y0, double th0,
                        double x1, double y1, double th1,
                        double turningRadius, double stepSize,
                        std::vector<RSSample>& outSamples,
                        bool forwardOnly)
{
    outSamples.clear();
    if (turningRadius <= 0.0)
        return false;

    const RSPath path = normalizedPath(x0, y0, th0, x1, y1, th1, turningRadius, forwardOnly);
    if (!std::isfinite(path.length()))
        return false;

    const double c = std::cos(th0), s = std::sin(th0);

    // Convert a pose expressed in the normalized (unit-radius, start-relative)
    // frame back into the input frame.
    auto emit = [&](double lx, double ly, double lth, bool fwd)
    {
        RSSample smp;
        smp.x = x0 + turningRadius * (c * lx - s * ly);
        smp.y = y0 + turningRadius * (s * lx + c * ly);
        smp.theta = th0 + lth;
        smp.forward = fwd;
        outSamples.push_back(smp);
    };

    double dsUnit = stepSize / turningRadius;
    if (!(dsUnit > 0.0))
        dsUnit = 0.05;

    double px = 0.0, py = 0.0, pth = 0.0;
    bool firstEmitted = false;

    for (int i = 0; i < 5; ++i)
    {
        const RSSeg seg = path.type[i];
        if (seg == RS_NOP)
            continue;

        const double segLen = path.len[i];
        const double gear = (segLen >= 0.0) ? 1.0 : -1.0;
        const double steer = (seg == RS_LEFT) ? 1.0 : (seg == RS_RIGHT) ? -1.0 : 0.0;
        double remaining = std::fabs(segLen);

        if (!firstEmitted)
        {
            emit(px, py, pth, gear > 0.0);
            firstEmitted = true;
        }

        while (remaining > 1e-9)
        {
            const double ds = std::min(dsUnit, remaining);
            const double signedDs = gear * ds;
            if (steer == 0.0)
            {
                px += signedDs * std::cos(pth);
                py += signedDs * std::sin(pth);
            }
            else
            {
                const double thNew = pth + steer * signedDs;
                px += (std::sin(thNew) - std::sin(pth)) / steer;
                py -= (std::cos(thNew) - std::cos(pth)) / steer;
                pth = thNew;
            }
            remaining -= ds;
            emit(px, py, pth, gear > 0.0);
        }
    }

    if (!firstEmitted)  // degenerate: start == goal
        emit(0.0, 0.0, 0.0, true);

    return true;
}

}  // namespace ugv_nav4d
