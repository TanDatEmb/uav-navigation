#include "utils/optimization/root_finder.h"

#include <limits>
#include <numeric>

int math_utils::RootFinderPriv::polyMod(double *u, double *v, double *r, int lu, int lv) {
    int orderu = lu - 1;
    int orderv = lv - 1;

    memcpy(r, u, lu * sizeof(double));

    if (v[0] < 0.0) {
        for (int i = orderv + 1; i <= orderu; i += 2) {
            r[i] = -r[i];
        }
        for (int i = 0; i <= orderu - orderv; i++) {
            for (int j = i + 1; j <= orderv + i; j++) {
                r[j] = -r[j] - r[i] * v[j - i];
            }
        }
    } else {
        for (int i = 0; i <= orderu - orderv; i++) {
            for (int j = i + 1; j <= orderv + i; j++) {
                r[j] = r[j] - r[i] * v[j - i];
            }
        }
    }

    int k = orderv - 1;
    while (k >= 0 && fabs(r[orderu - k]) < DBL_EPSILON) {
        r[orderu - k] = 0.0;
        k--;
    }

    return (k <= 0) ? 1 : (k + 1);
}

double math_utils::RootFinderPriv::polyEval(double *p, int len, double x) {
    double retVal = 0.0;

    if (len > 0) {
        if (fabs(x) < DBL_EPSILON) {
            retVal = p[len - 1];
        } else if (x == 1.0) {
            for (int i = len - 1; i >= 0; i--) {
                retVal += p[i];
            }
        } else {
            double xn = 1.0;

            for (int i = len - 1; i >= 0; i--) {
                retVal += p[i] * xn;
                xn *= x;
            }
        }
    }

    return retVal;
}


std::set<double> math_utils::RootFinderPriv::solveCub(double a, double b, double c, double d) {
    std::set<double> roots;

    constexpr double cos120 = -0.50;
    constexpr double sin120 = 0.866025403784438646764;

    if (fabs(d) < DBL_EPSILON) {
        // First solution is x = 0
        roots.insert(0.0);

        // Converting to a quadratic equation
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }

    if (fabs(a) < DBL_EPSILON) {
        if (fabs(b) < DBL_EPSILON) {
            // Linear equation
            if (fabs(c) > DBL_EPSILON)
                roots.insert(-d / c);
        } else {
            // Quadratic equation
            double discriminant = c * c - 4.0 * b * d;
            if (discriminant >= 0) {
                double inv2b = 1.0 / (2.0 * b);
                double y = sqrt(discriminant);
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }
    } else {
        // Cubic equation
        double inva = 1.0 / a;
        double invaa = inva * inva;
        double bb = b * b;
        double bover3a = b * (1.0 / 3.0) * inva;
        double p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
        double halfq = (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
        double yy = p * p * p / 27.0 + halfq * halfq;

        if (yy > DBL_EPSILON) {
            // Sqrt is positive: one real solution
            double y = sqrt(yy);
            double uuu = -halfq + y;
            double vvv = -halfq - y;
            double www = fabs(uuu) > fabs(vvv) ? uuu : vvv;
            double w = (www < 0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            roots.insert(w - p / (3.0 * w) - bover3a);
        } else if (yy < -DBL_EPSILON) {
            // Sqrt is negative: three real solutions
            double x = -halfq;
            double y = sqrt(-yy);
            double theta;
            double r;
            double ux;
            double uyi;
            // Convert to polar form
            if (fabs(x) > DBL_EPSILON) {
                theta = (x > 0.0) ? atan(y / x) : (atan(y / x) + M_PI);
                r = sqrt(x * x - yy);
            } else {
                // Vertical line
                theta = M_PI / 2.0;
                r = y;
            }
            // Calculate cube root
            theta /= 3.0;
            r = pow(r, 1.0 / 3.0);
            // Convert to complex coordinate
            ux = cos(theta) * r;
            uyi = sin(theta) * r;
            // First solution
            roots.insert(ux + ux - bover3a);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * (ux * cos120 - uyi * sin120) - bover3a);
            // Third solution, rotate -120 degrees
            roots.insert(2.0 * (ux * cos120 + uyi * sin120) - bover3a);
        } else {
            // Sqrt is zero: two real solutions
            double www = -halfq;
            double w = (www < 0.0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            // First solution
            roots.insert(w + w - bover3a);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * w * cos120 - bover3a);
        }
    }
    return roots;
}

int math_utils::RootFinderPriv::solveResolvent(double *x, double a, double b, double c) {
    double a2 = a * a;
    double q = (a2 - 3.0 * b) / 9.0;
    double r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
    double r2 = r * r;
    double q3 = q * q * q;
    double A, B;
    if (r2 < q3) {
        double t = r / sqrt(q3);
        if (t < -1.0) {
            t = -1.0;
        }
        if (t > 1.0) {
            t = 1.0;
        }
        t = acos(t);
        a /= 3.0;
        q = -2.0 * sqrt(q);
        x[0] = q * cos(t / 3.0) - a;
        x[1] = q * cos((t + M_PI * 2.0) / 3.0) - a;
        x[2] = q * cos((t - M_PI * 2.0) / 3.0) - a;
        return 3;
    } else {
        A = -pow(fabs(r) + sqrt(r2 - q3), 1.0 / 3.0);
        if (r < 0.0) {
            A = -A;
        }
        B = (0.0 == A ? 0.0 : q / A);

        a /= 3.0;
        x[0] = (A + B) - a;
        x[1] = -0.5 * (A + B) - a;
        x[2] = 0.5 * sqrt(3.0) * (A - B);
        if (fabs(x[2]) < DBL_EPSILON) {
            x[2] = x[1];
            return 2;
        }

        return 1;
    }
}

std::set<double> math_utils::RootFinderPriv::solveQuartMonic(double a, double b, double c, double d) {
    std::set<double> roots;

    double a3 = -b;
    double b3 = a * c - 4.0 * d;
    double c3 = -a * a * d - c * c + 4.0 * b * d;

    // Solve the resolvent: y^3 - b*y^2 + (ac - 4*d)*y - a^2*d - c^2 + 4*b*d = 0
    double x3[3];
    int iZeroes = solveResolvent(x3, a3, b3, c3);

    double q1, q2, p1, p2, D, sqrtD, y;

    y = x3[0];
    // Choosing Y with maximal absolute value.
    if (iZeroes != 1) {
        if (fabs(x3[1]) > fabs(y)) {
            y = x3[1];
        }
        if (fabs(x3[2]) > fabs(y)) {
            y = x3[2];
        }
    }

    // h1 + h2 = y && h1*h2 = d  <=>  h^2 - y*h + d = 0    (h === q)

    D = y * y - 4.0 * d;
    if (fabs(D) < DBL_EPSILON) //In other words: D == 0
    {
        q1 = q2 = y * 0.5;
        // g1 + g2 = a && g1 + g2 = b - y   <=>   g^2 - a*g + b - y = 0    (p === g)
        D = a * a - 4.0 * (b - y);
        if (fabs(D) < DBL_EPSILON) //In other words: D == 0
        {
            p1 = p2 = a * 0.5;
        } else {
            sqrtD = sqrt(D);
            p1 = (a + sqrtD) * 0.5;
            p2 = (a - sqrtD) * 0.5;
        }
    } else {
        sqrtD = sqrt(D);
        q1 = (y + sqrtD) * 0.5;
        q2 = (y - sqrtD) * 0.5;
        // g1 + g2 = a && g1*h2 + g2*h1 = c   ( && g === p )  Krammer
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    // Solve the quadratic equation: x^2 + p1*x + q1 = 0
    D = p1 * p1 - 4.0 * q1;
    if (fabs(D) < DBL_EPSILON) {
        roots.insert(-p1 * 0.5);
    } else if (D > 0.0) {
        sqrtD = sqrt(D);
        roots.insert((-p1 + sqrtD) * 0.5);
        roots.insert((-p1 - sqrtD) * 0.5);
    }

    // Solve the quadratic equation: x^2 + p2*x + q2 = 0
    D = p2 * p2 - 4.0 * q2;
    if (fabs(D) < DBL_EPSILON) {
        roots.insert(-p2 * 0.5);
    } else if (D > 0.0) {
        sqrtD = sqrt(D);
        roots.insert((-p2 + sqrtD) * 0.5);
        roots.insert((-p2 - sqrtD) * 0.5);
    }

    return roots;
}

std::set<double> math_utils::RootFinderPriv::solveQuart(double a, double b, double c, double d, double e) {
    if (fabs(a) < DBL_EPSILON) {
        return solveCub(b, c, d, e);
    } else {
        return solveQuartMonic(b / a, c / a, d / a, e / a);
    }
}

std::set<double>
math_utils::RootFinderPriv::eigenSolveRealRoots(const Eigen::VectorXd &coeffs, double lbound, double ubound,
                                                double tol) {
    std::set<double> rts;

    int order = (int) coeffs.size() - 1;
    Eigen::VectorXd monicCoeffs(order + 1);
    monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

    Eigen::MatrixXd companionMat(order, order);
    companionMat.setZero();
    companionMat(0, order - 1) = -monicCoeffs(order);
    for (int i = 1; i < order; i++) {
        companionMat(i, i - 1) = 1.0;
        companionMat(i, order - 1) = -monicCoeffs(order - i);
    }
    Eigen::VectorXcd eivals = companionMat.eigenvalues();
    double real;
    int eivalsNum = eivals.size();
    for (int i = 0; i < eivalsNum; i++) {
        real = eivals(i).real();
        if (std::abs(eivals(i).imag()) < tol && real > lbound && real < ubound)
            rts.insert(real);
    }

    return rts;
}

double math_utils::RootFinderPriv::numSignVar(double x, double **sturmSeqs, int *szSeq, int len) {
    double y, lasty;
    int signVar = 0;
    lasty = polyEval(sturmSeqs[0], szSeq[0], x);
    for (int i = 1; i < len; i++) {
        y = polyEval(sturmSeqs[i], szSeq[i], x);
        if (lasty == 0.0 || lasty * y < 0.0) {
            ++signVar;
        }
        lasty = y;
    }

    return signVar;
};

void math_utils::RootFinderPriv::polyDeri(double *coeffs, double *dcoeffs, int len) {
    int horder = len - 1;
    for (int i = 0; i < horder; i++) {
        dcoeffs[i] = (horder - i) * coeffs[i];
    }
    return;
}

template<typename F, typename DF>
double math_utils::RootFinderPriv::safeNewton(const F &func, const DF &dfunc, const double &l, const double &h,
                                              const double &tol, const int &maxIts) {
    if (!std::isfinite(l) || !std::isfinite(h) || !(l < h) ||
        !std::isfinite(tol) || tol <= 0.0 || maxIts <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double xh, xl;
    double fl = func(l);
    double fh = func(h);
    if (!std::isfinite(fl) || !std::isfinite(fh)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (fl == 0.0) {
        return l;
    }
    if (fh == 0.0) {
        return h;
    }
    if ((fl < 0.0) == (fh < 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (fl < 0.0) {
        xl = l;
        xh = h;
    } else {
        xh = l;
        xl = h;
    }

    double rts = std::midpoint(xl, xh);
    long double dxold = std::abs(static_cast<long double>(xh) -
                                 static_cast<long double>(xl));
    long double dx = dxold;
    double f = func(rts);
    double df = dfunc(rts);
    if (!std::isfinite(f) || !std::isfinite(df)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (f == 0.0) {
        return rts;
    }
    double temp;
    for (int j = 0; j < maxIts; j++) {
        const long double left_test =
            (static_cast<long double>(rts) - xh) * df - f;
        const long double right_test =
            (static_cast<long double>(rts) - xl) * df - f;
        const long double scaled_derivative = dxold * df;
        const bool use_bisection = !std::isfinite(left_test) ||
            !std::isfinite(right_test) ||
            left_test * right_test > 0.0 ||
            !std::isfinite(scaled_derivative) ||
            std::abs(2.0L * f) > std::abs(scaled_derivative) ||
            df == 0.0;
        if (use_bisection) {
            dxold = dx;
            dx = 0.5L * (static_cast<long double>(xh) - xl);
            const double next_rts = static_cast<double>(
                static_cast<long double>(xl) + dx);
            if (!std::isfinite(next_rts)) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            rts = next_rts;
            if (xl == rts) {
                break;
            }
        } else {
            dxold = dx;
            dx = static_cast<long double>(f) / df;
            if (!std::isfinite(dx)) {
                dxold = dx;
                dx = 0.5L * (static_cast<long double>(xh) - xl);
                const double next_rts = static_cast<double>(
                    static_cast<long double>(xl) + dx);
                if (!std::isfinite(next_rts)) {
                    return std::numeric_limits<double>::quiet_NaN();
                }
                rts = next_rts;
            } else {
            temp = rts;
            rts = static_cast<double>(static_cast<long double>(rts) - dx);
            if (!std::isfinite(rts)) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            if (temp == rts) {
                break;
            }
            }
        }

        if (std::abs(dx) < static_cast<long double>(tol)) {
            break;
        }

        f = func(rts);
        df = dfunc(rts);
        if (!std::isfinite(f) || !std::isfinite(df)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (f == 0.0) {
            return rts;
        }
        if (f < 0.0) {
            xl = rts;
        } else {
            xh = rts;
        }
    }

    return rts;
}

double
math_utils::RootFinderPriv::shrinkInterval(double *coeffs, int numCoeffs, double lbound, double ubound, double tol) {
    double *dcoeffs = new double[numCoeffs - 1];
    polyDeri(coeffs, dcoeffs, numCoeffs);
    auto func = [&coeffs, &numCoeffs](double x) { return polyEval(coeffs, numCoeffs, x); };
    auto dfunc = [&dcoeffs, &numCoeffs](double x) { return polyEval(dcoeffs, numCoeffs - 1, x); };
    constexpr int maxDblIts = 128;
    double rts = safeNewton(func, dfunc, lbound, ubound, tol, maxDblIts);
    delete[] dcoeffs;
    return rts;
}

void math_utils::RootFinderPriv::recurIsolate(double l, double r, double fl, double fr, int lnv, int rnv, double tol,
                                              double **sturmSeqs, int *szSeq, int len, std::set<double> &rts) {
    int nrts = lnv - rnv;
    double fm;
    double m;

    if (nrts == 0) {
        return;
    } else if (nrts == 1) {
        if (fl * fr < 0) {
            const double root = shrinkInterval(sturmSeqs[0], szSeq[0], l, r, tol);
            if (std::isfinite(root)) rts.insert(root);
            return;
        } else {
            // Bisect when non of above works
            int maxDblIts = 128;

            for (int i = 0; i < maxDblIts; i++) {
                // Calculate the root with even multiplicity
                if (fl * fr < 0) {
                    const double root = shrinkInterval(sturmSeqs[1], szSeq[1], l, r, tol);
                    if (std::isfinite(root)) rts.insert(root);
                    return;
                }

                m = (l + r) / 2.0;
                fm = polyEval(sturmSeqs[0], szSeq[0], m);

                if (fm == 0 || fabs(r - l) < tol) {
                    rts.insert(m);
                    return;
                } else {
                    if (lnv == numSignVar(m, sturmSeqs, szSeq, len)) {
                        l = m;
                        fl = fm;
                    } else {
                        r = m;
                        fr = fm;
                    }
                }
            }

            rts.insert(m);
            return;
        }
    } else if (nrts > 1) {
        // More than one root exists in the interval
        int maxDblIts = 128;

        int mnv;
        int bias = 0;
        bool biased = false;
        for (int i = 0; i < maxDblIts; i++) {
            bias = biased ? bias : 0;
            if (!biased) {
                m = (l + r) / 2.0;
            } else {
                m = (r - l) / pow(2.0, bias + 1.0) + l;
                biased = false;
            }
            mnv = numSignVar(m, sturmSeqs, szSeq, len);

            if (fabs(r - l) < tol) {
                rts.insert(m);
                return;
            } else {
                fm = polyEval(sturmSeqs[0], szSeq[0], m);
                if (fm == 0) {
                    bias++;
                    biased = true;
                } else if (lnv != mnv && rnv != mnv) {
                    recurIsolate(l, m, fl, fm, lnv, mnv, tol, sturmSeqs, szSeq, len, rts);
                    recurIsolate(m, r, fm, fr, mnv, rnv, tol, sturmSeqs, szSeq, len, rts);
                    return;
                } else if (lnv == mnv) {
                    l = m;
                    fl = fm;
                } else {
                    r = m;
                    fr = fm;
                }
            }
        }

        rts.insert(m);
        return;
    }
};

std::set<double>
math_utils::RootFinderPriv::isolateRealRoots(const Eigen::VectorXd &coeffs, double lbound, double ubound, double tol) {
    std::set<double> rts;

    // Calculate monic coefficients
    int order = (int) coeffs.size() - 1;
    Eigen::VectorXd monicCoeffs(order + 1);
    monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

    // Calculate Cauchy’s bound for the roots of a polynomial
    double rho_c = 1 + monicCoeffs.tail(order).cwiseAbs().maxCoeff();

    // Calculate Kojima’s bound for the roots of a polynomial
    Eigen::VectorXd nonzeroCoeffs(order + 1);
    nonzeroCoeffs.setZero();
    int nonzeros = 0;
    double tempEle;
    for (int i = 0; i < order + 1; i++) {
        tempEle = monicCoeffs(i);
        if (fabs(tempEle) >= DBL_EPSILON) {
            nonzeroCoeffs(nonzeros++) = tempEle;
        }
    }
    nonzeroCoeffs = nonzeroCoeffs.head(nonzeros).eval();
    Eigen::VectorXd kojimaVec = nonzeroCoeffs.tail(nonzeros - 1).cwiseQuotient(
            nonzeroCoeffs.head(nonzeros - 1)).cwiseAbs();
    kojimaVec.tail(1) /= 2.0;
    double rho_k = 2.0 * kojimaVec.maxCoeff();

    // Choose a sharper one then loosen it by 1.0 to get an open interval
    double rho = std::min(rho_c, rho_k) + 1.0;

    // Tighten the bound to search in
    lbound = std::max(lbound, -rho);
    ubound = std::min(ubound, rho);

    // Build Sturm sequence
    int len = monicCoeffs.size();
    double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
    int szSeq[RootFinderParam::highestOrder + 1] = {0}; // Explicit ini as zero (gcc may neglect this in -O3)
    double *offsetSeq[RootFinderParam::highestOrder + 1];
    int num = 0;

    for (int i = 0; i < len; i++) {
        sturmSeqs[i] = monicCoeffs(i);
        sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
    }
    szSeq[0] = len;
    szSeq[1] = len - 1;
    offsetSeq[0] = sturmSeqs + len - szSeq[0];
    offsetSeq[1] = sturmSeqs + 2 * len - szSeq[1];

    num += 2;

    bool remainderConstant = false;
    int idx = 0;
    while (!remainderConstant) {
        szSeq[idx + 2] = polyMod(offsetSeq[idx],
                                 offsetSeq[idx + 1],
                                 &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                                 szSeq[idx], szSeq[idx + 1]);
        offsetSeq[idx + 2] = sturmSeqs + (idx + 3) * len - szSeq[idx + 2];

        remainderConstant = szSeq[idx + 2] == 1;
        for (int i = 1; i < szSeq[idx + 2]; i++) {
            offsetSeq[idx + 2][i] /= -fabs(offsetSeq[idx + 2][0]);
        }
        offsetSeq[idx + 2][0] = offsetSeq[idx + 2][0] > 0.0 ? -1.0 : 1.0;
        num++;
        idx++;
    }

    // Isolate all distinct roots inside the open interval recursively
    recurIsolate(lbound, ubound,
                 polyEval(offsetSeq[0], szSeq[0], lbound),
                 polyEval(offsetSeq[0], szSeq[0], ubound),
                 numSignVar(lbound, offsetSeq, szSeq, len),
                 numSignVar(ubound, offsetSeq, szSeq, len),
                 tol, offsetSeq, szSeq, len, rts);

    return rts;
}


Eigen::VectorXd math_utils::RootFinder::polySqr(const Eigen::VectorXd &coef)       {
    if (coef.size() == 0) {
        return Eigen::VectorXd();
    }
    int coefSize = coef.size();
    int resultSize = coefSize * 2 - 1;
    int lbound, rbound;
    Eigen::VectorXd result(resultSize);
    double temp;
    for (int i = 0; i < resultSize; i++) {
        temp = 0;
        lbound = i - coefSize + 1;
        lbound = lbound > 0 ? lbound : 0;
        rbound = coefSize < (i + 1) ? coefSize : (i + 1);
        rbound += lbound;
        if (rbound & 1) //faster than rbound % 2 == 1
        {
            rbound >>= 1; //faster than rbound /= 2
            temp += coef(rbound) * coef(rbound);
        } else {
            rbound >>= 1; //faster than rbound /= 2
        }

        for (int j = lbound; j < rbound; j++) {
            temp += 2.0 * coef(j) * coef(i - j);
        }
        result(i) = temp;
    }

    return result;
}


double math_utils::RootFinder::polyVal(const Eigen::VectorXd &coeffs, double x, bool numericalStability)   {
    double retVal = 0.0;
    int order = (int) coeffs.size() - 1;

    if (order >= 0) {
        if (fabs(x) < DBL_EPSILON) {
            retVal = coeffs(order);
        } else if (x == 1.0) {
            retVal = coeffs.sum();
        } else {
            if (numericalStability) {
                double xn = 1.0;

                for (int i = order; i >= 0; i--) {
                    retVal += coeffs(i) * xn;
                    xn *= x;
                }
            } else {
                int len = coeffs.size();

                for (int i = 0; i < len; i++) {
                    retVal = retVal * x + coeffs(i);
                }
            }
        }
    }

    return retVal;
}

int math_utils::RootFinder::countRoots(const Eigen::VectorXd &coeffs, double l, double r)         {
    int nRoots = 0;

    int originalSize = coeffs.size();
    if (!std::isfinite(l) || !std::isfinite(r) || l >= r ||
        !coeffs.allFinite()) {
        return -1;
    }
    if (originalSize > static_cast<int>(RootFinderParam::highestOrder + 1)) {
        return -1;
    }
    const double coefficientScale = originalSize > 0
            ? coeffs.cwiseAbs().maxCoeff() : 0.0;
    const double zeroTolerance = 64.0 * DBL_EPSILON *
            static_cast<double>(std::max(1, originalSize)) * coefficientScale;
    int valid = originalSize;
    for (int i = 0; i < originalSize; i++) {
        if (fabs(coeffs(i)) <= zeroTolerance) {
            valid--;
        } else {
            break;
        }
    }

    // A nonzero constant has no roots.  It must not enter the Sturm setup:
    // that sequence has order zero and the derivative initialization divides
    // by the order.
    const int first_coefficient = originalSize - valid;
    int effective_valid = valid;
    int trailing_zero_count = 0;
    while (effective_valid > 1 &&
           fabs(coeffs(originalSize - trailing_zero_count - 1)) <= zeroTolerance) {
        --effective_valid;
        ++trailing_zero_count;
    }
    if (trailing_zero_count > 0 && l < 0.0 && r > 0.0) {
        nRoots = 1;
    }
    if (effective_valid <= 1) {
        return nRoots;
    }

    {
        Eigen::VectorXd monicCoeffs(effective_valid);
        monicCoeffs << 1.0,
            coeffs.segment(first_coefficient + 1, effective_valid - 1) /
                coeffs(first_coefficient);

        // Build the Sturm sequence
        int len = monicCoeffs.size();
        int order = len - 1;
        double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
        int szSeq[
                RootFinderParam::highestOrder + 1] = {0}; // Explicit ini as zero (gcc may neglect this in -O3)
        int num = 0;

        for (int i = 0; i < len; i++) {
            sturmSeqs[i] = monicCoeffs(i);
            sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
        }
        szSeq[0] = len;
        szSeq[1] = len - 1;
        num += 2;

        bool remainderConstant = false;
        int idx = 0;
        while (!remainderConstant) {
            szSeq[idx + 2] = RootFinderPriv::polyMod(&(sturmSeqs[(idx + 1) * len - szSeq[idx]]),
                                                     &(sturmSeqs[(idx + 2) * len - szSeq[idx + 1]]),
                                                     &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                                                     szSeq[idx], szSeq[idx + 1]);
            remainderConstant = szSeq[idx + 2] == 1;
            for (int i = 1; i < szSeq[idx + 2]; i++) {
                sturmSeqs[(idx + 3) * len - szSeq[idx + 2] + i] /= -fabs(
                        sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
            }
            sturmSeqs[(idx + 3) * len - szSeq[idx + 2]] /= -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
            num++;
            idx++;
        }

        // Count numbers of sign variations at two boundaries
        double yl, lastyl, yr, lastyr;
        lastyl = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], l);
        lastyr = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], r);
        for (int i = 1; i < num; i++) {
            yl = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], l);
            yr = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], r);
            if (lastyl == 0.0 || lastyl * yl < 0.0) {
                ++nRoots;
            }
            if (lastyr == 0.0 || lastyr * yr < 0.0) {
                --nRoots;
            }
            lastyl = yl;
            lastyr = yr;
        }
    }

    return nRoots;
}

std::set<double>
math_utils::RootFinder::solvePolynomial(const Eigen::VectorXd &coeffs, double lbound, double ubound, double tol,
                                        bool isolation)    {
    std::set<double> rts;

    if (!std::isfinite(lbound) || !std::isfinite(ubound) || lbound >= ubound ||
        !std::isfinite(tol) || tol <= 0.0 || !coeffs.allFinite() ||
        coeffs.size() > static_cast<Eigen::Index>(RootFinderParam::highestOrder + 1)) {
        return rts;
    }

    const double coefficientScale = coeffs.size() > 0
            ? coeffs.cwiseAbs().maxCoeff() : 0.0;
    const double zeroTolerance = 64.0 * DBL_EPSILON *
            static_cast<double>(std::max<Eigen::Index>(1, coeffs.size())) *
            coefficientScale;
    int valid = coeffs.size();
    for (int i = 0; i < coeffs.size(); i++) {
        if (fabs(coeffs(i)) <= zeroTolerance) {
            valid--;
        } else {
            break;
        }
    }

    int offset = 0;
    int nonzeros = valid;
    if (valid > 0) {
        for (int i = 0; i < valid; i++) {
            if (fabs(coeffs(coeffs.size() - i - 1)) <= zeroTolerance) {
                nonzeros--;
                offset++;
            } else {
                break;
            }
        }
    }

    if (nonzeros == 0) {
        rts.insert(INFINITY);
        rts.insert(-INFINITY);
    } else if (nonzeros == 1 && offset == 0) {
        rts.clear();
    } else {
        Eigen::VectorXd ncoeffs(std::max(5, nonzeros));
        ncoeffs.setZero();
        ncoeffs.tail(nonzeros) <<
                coeffs.segment(coeffs.size() - valid, nonzeros) /
                coefficientScale;

        if (nonzeros <= 5) {
            rts = RootFinderPriv::solveQuart(ncoeffs(0), ncoeffs(1), ncoeffs(2), ncoeffs(3), ncoeffs(4));
        } else {
            if (isolation) {
                rts = RootFinderPriv::isolateRealRoots(ncoeffs, lbound, ubound, tol);
            } else {
                rts = RootFinderPriv::eigenSolveRealRoots(ncoeffs, lbound, ubound, tol);
            }
        }

        if (offset > 0) {
            rts.insert(0.0);
        }
    }

    for (auto it = rts.begin(); it != rts.end();) {
        if (*it > lbound && *it < ubound) {
            it++;
        } else {
            it = rts.erase(it);
        }
    }

    return rts;
}
