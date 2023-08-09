// detria - a delaunay triangulation library
//
// Example usage:
/*

// create a square, and triangulate it

// list of points (positions)
std::vector<detria::PointD> points =
{
    { 0.0, 0.0 },
    { 1.0, 0.0 },
    { 1.0, 1.0 },
    { 0.0, 1.0 }
};

// list of point indices
std::vector<uint32_t> outline = { 0, 1, 2, 3 };

bool delaunay = true;

detria::Triangulation tri;
tri.setPoints(points);
tri.addOutline(outline);

bool success = tri.triangulate(delaunay);

if (success)
{
    bool cwTriangles = true;

    tri.forEachTriangle([&](detria::Triangle<uint32_t> triangle)
    {
        // `triangle` contains the point indices

        detria::PointD firstPointOfTriangle = points[triangle.x];
        detria::PointD secondPointOfTriangle = points[triangle.y];
        detria::PointD thirdPointOfTriangle = points[triangle.z];
    }, cwTriangles);
}

*/
// See https://github.com/Kimbatt/detria for more information and full documentation.
// License: WTFPL or MIT, at your choice. You can find the license texts at the bottom of this file.

#ifndef DETRIA_HPP_INCLUDED
#define DETRIA_HPP_INCLUDED

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <optional>
#include <variant>
#include <string>
#include <sstream>

#if !defined(NDEBUG)
#include <iostream>

#if !defined(_WIN32) || !_WIN32
#include <csignal>
#endif

#endif

namespace detria
{
    template <typename T>
    struct Vec2
    {
        T x;
        T y;
    };

    using PointF = Vec2<float>;
    using PointD = Vec2<double>;

    template <typename Point>
    struct DefaultPointAdapter
    {
        // Adapter classes are used to convert point types, to types that have `x` and `y` fields.
        // This is the default accessor, so the points must already have an `x` and `y` field, which must be the same type.
        // For point types that don't have an `x` and `y` field, a custom adapter class must be created.
        // That class must have the following static function called `adapt`: `static PointWithXY adapt(CustomPoint p)`
        // where `CustomPoint` is the type of the custom point class, and `PointWithXY` is any type that has `x` and `y` fields
        // `inline` and const references can also be used for these functions (see the implementation for this class below)
        // For example, for a point class, which uses the index operator to get the components, the adapter class would look like this:
        /*
        struct PointAdapter
        {
            static Vec2<float> adapt(const CustomPoint& p)
            {
                return Vec2<float>{ .x = p[0], .y = p[1] };
            }
        };
        */

        // You can also reinterpret to avoid copies (make sure you know what you're doing)
        /*
        struct CustomPoint
        {
            std::array<float, 2> position;
        };

        struct PointWithXY
        {
            float x;
            float y;
        };

        struct PointAdapter
        {
            inline static const PointWithXY& adapt(const CustomPoint& p)
            {
                return *reinterpret_cast<const PointWithXY*>(p.position.data());
            }
        };
        */

        // If you get an error here, then it's likely that your Point class has no field 'x'
        // In this case, a custom adapter must be created, see the comment above
        using Scalar = decltype(Point::x);

        // If you get an error here, then it's likely that the type of 'x' in your Point class is not a float/double
        static_assert(std::is_floating_point_v<Scalar>, "The Point type's x and y fields must be floating point type");

        // If you get an error here, then it's likely that the type of 'x' and 'y' in your Point class are different
        // (for example, if 'x' is float, then 'y' also must be a float, not a double)
        static_assert(std::is_same_v<Scalar, decltype(Point::y)>, "The Point type's x and y fields must be the same type");

        inline static const Point& adapt(const Point& p)
        {
            return p;
        }
    };

    template <typename Idx>
    struct Triangle
    {
        Idx x;
        Idx y;
        Idx z;
    };

    template <typename Idx>
    struct Edge
    {
        Idx x;
        Idx y;
    };

#if (defined(_MSVC_LANG) && _MSVC_LANG >= 202002L) || __cplusplus >= 202002L
#define DETRIA_LIKELY [[likely]]
#define DETRIA_UNLIKELY [[unlikely]]
#else
#define DETRIA_LIKELY
#define DETRIA_UNLIKELY
#endif

    namespace predicates
    {
        // http://www.cs.cmu.edu/~quake/robust.html
        // Routines for Arbitrary Precision Floating-point Arithmetic and Fast Robust Geometric Predicates
        // Placed in the public domain by Jonathan Richard Shewchuk

        template <typename Scalar>
        struct ErrorBounds
        {
            Scalar splitter;
            Scalar epsilon;
            Scalar resulterrbound;
            Scalar ccwerrboundA, ccwerrboundB, ccwerrboundC;
            Scalar o3derrboundA, o3derrboundB, o3derrboundC;
            Scalar iccerrboundA, iccerrboundB, iccerrboundC;
            Scalar isperrboundA, isperrboundB, isperrboundC;
        };

        template <typename Scalar>
        constexpr ErrorBounds<Scalar> calculateErrorBounds()
        {
            Scalar check = Scalar(1);
            bool everyOther = true;

            Scalar epsilon = Scalar(1);
            Scalar splitter = Scalar(1);

            Scalar lastcheck{ };
            do
            {
                lastcheck = check;
                epsilon *= Scalar(0.5);
                if (everyOther)
                {
                    splitter *= Scalar(2);
                }
                everyOther = !everyOther;
                check = Scalar(1) + epsilon;
            } while (check != Scalar(1) && (check != lastcheck));

            splitter += Scalar(1);

            ErrorBounds<Scalar> bounds{ };
            bounds.splitter = splitter;
            bounds.epsilon = epsilon;
            bounds.resulterrbound = (Scalar(3) + Scalar(8) * epsilon) * epsilon;
            bounds.ccwerrboundA = (Scalar(3) + Scalar(16) * epsilon) * epsilon;
            bounds.ccwerrboundB = (Scalar(2) + Scalar(12) * epsilon) * epsilon;
            bounds.ccwerrboundC = (Scalar(9) + Scalar(64) * epsilon) * epsilon * epsilon;
            bounds.o3derrboundA = (Scalar(7) + Scalar(56) * epsilon) * epsilon;
            bounds.o3derrboundB = (Scalar(3) + Scalar(28) * epsilon) * epsilon;
            bounds.o3derrboundC = (Scalar(26) + Scalar(288) * epsilon) * epsilon * epsilon;
            bounds.iccerrboundA = (Scalar(10) + Scalar(96) * epsilon) * epsilon;
            bounds.iccerrboundB = (Scalar(4) + Scalar(48) * epsilon) * epsilon;
            bounds.iccerrboundC = (Scalar(44) + Scalar(576) * epsilon) * epsilon * epsilon;
            bounds.isperrboundA = (Scalar(16) + Scalar(224) * epsilon) * epsilon;
            bounds.isperrboundB = (Scalar(5) + Scalar(72) * epsilon) * epsilon;
            bounds.isperrboundC = (Scalar(71) + Scalar(1408) * epsilon) * epsilon * epsilon;

            return bounds;
        }

        template <typename Scalar>
        static constexpr predicates::ErrorBounds errorBounds = predicates::calculateErrorBounds<Scalar>();

#define DETRIA_INEXACT

#define DETRIA_Fast_Two_Sum_Tail(a, b, x, y) \
  bvirt = x - a; \
  y = b - bvirt

#define DETRIA_Fast_Two_Sum(a, b, x, y) \
  x = Scalar(a + b); \
  DETRIA_Fast_Two_Sum_Tail(a, b, x, y)

#define DETRIA_Two_Sum_Tail(a, b, x, y) \
  bvirt = Scalar(x - a); \
  avirt = x - bvirt; \
  bround = b - bvirt; \
  around = a - avirt; \
  y = around + bround

#define DETRIA_Two_Sum(a, b, x, y) \
  x = Scalar(a + b); \
  DETRIA_Two_Sum_Tail(a, b, x, y)

#define DETRIA_Two_Diff_Tail(a, b, x, y) \
  bvirt = Scalar(a - x); \
  avirt = x + bvirt; \
  bround = bvirt - b; \
  around = a - avirt; \
  y = around + bround

#define DETRIA_Two_Diff(a, b, x, y) \
  x = Scalar(a - b); \
  DETRIA_Two_Diff_Tail(a, b, x, y)

#define DETRIA_Split(a, ahi, alo) \
  c = Scalar(errorBounds<Scalar>.splitter * a); \
  abig = Scalar(c - a); \
  ahi = c - abig; \
  alo = a - ahi

#define DETRIA_Two_Product_Tail(a, b, x, y) \
  DETRIA_Split(a, ahi, alo); \
  DETRIA_Split(b, bhi, blo); \
  err1 = x - (ahi * bhi); \
  err2 = err1 - (alo * bhi); \
  err3 = err2 - (ahi * blo); \
  y = (alo * blo) - err3

#define DETRIA_Two_Product(a, b, x, y) \
  x = Scalar(a * b); \
  DETRIA_Two_Product_Tail(a, b, x, y)

#define DETRIA_Two_Product_Presplit(a, b, bhi, blo, x, y) \
  x = Scalar(a * b); \
  DETRIA_Split(a, ahi, alo); \
  err1 = x - (ahi * bhi); \
  err2 = err1 - (alo * bhi); \
  err3 = err2 - (ahi * blo); \
  y = (alo * blo) - err3

#define DETRIA_Square_Tail(a, x, y) \
  DETRIA_Split(a, ahi, alo); \
  err1 = x - (ahi * ahi); \
  err3 = err1 - ((ahi + ahi) * alo); \
  y = (alo * alo) - err3

#define DETRIA_Square(a, x, y) \
  x = Scalar(a * a); \
  DETRIA_Square_Tail(a, x, y)

#define DETRIA_Two_One_Sum(a1, a0, b, x2, x1, x0) \
  DETRIA_Two_Sum(a0, b , _i, x0); \
  DETRIA_Two_Sum(a1, _i, x2, x1)

#define DETRIA_Two_One_Diff(a1, a0, b, x2, x1, x0) \
  DETRIA_Two_Diff(a0, b , _i, x0); \
  DETRIA_Two_Sum( a1, _i, x2, x1)

#define DETRIA_Two_Two_Sum(a1, a0, b1, b0, x3, x2, x1, x0) \
  DETRIA_Two_One_Sum(a1, a0, b0, _j, _0, x0); \
  DETRIA_Two_One_Sum(_j, _0, b1, x3, x2, x1)

#define DETRIA_Two_Two_Diff(a1, a0, b1, b0, x3, x2, x1, x0) \
  DETRIA_Two_One_Diff(a1, a0, b0, _j, _0, x0); \
  DETRIA_Two_One_Diff(_j, _0, b1, x3, x2, x1)

        template <typename Scalar>
        inline Scalar estimate(int elen, const Scalar* e)
        {
            Scalar Q = e[0];
            for (int eindex = 1; eindex < elen; eindex++)
            {
                Q += e[eindex];
            }
            return Q;
        }

        template <typename Scalar>
        inline constexpr Scalar Absolute(Scalar a)
        {
            return a >= Scalar(0) ? a : -a;
        }

        template <typename Scalar>
        int fast_expansion_sum_zeroelim(int elen, const Scalar* e, int flen, const Scalar* f, Scalar* h)
        {
            Scalar Q;
            DETRIA_INEXACT Scalar Qnew;
            DETRIA_INEXACT Scalar hh;
            DETRIA_INEXACT Scalar bvirt;
            Scalar avirt, bround, around;
            int eindex, findex, hindex;
            Scalar enow, fnow;

            enow = e[0];
            fnow = f[0];
            eindex = findex = 0;
            if ((fnow > enow) == (fnow > -enow))
            {
                Q = enow;
                enow = e[++eindex];
            }
            else
            {
                Q = fnow;
                fnow = f[++findex];
            }
            hindex = 0;
            if ((eindex < elen) && (findex < flen))
            {
                if ((fnow > enow) == (fnow > -enow))
                {
                    DETRIA_Fast_Two_Sum(enow, Q, Qnew, hh);
                    enow = e[++eindex];
                }
                else
                {
                    DETRIA_Fast_Two_Sum(fnow, Q, Qnew, hh);
                    fnow = f[++findex];
                }
                Q = Qnew;
                if (hh != Scalar(0))
                {
                    h[hindex++] = hh;
                }
                while ((eindex < elen) && (findex < flen))
                {
                    if ((fnow > enow) == (fnow > -enow))
                    {
                        DETRIA_Two_Sum(Q, enow, Qnew, hh);
                        enow = e[++eindex];
                    }
                    else
                    {
                        DETRIA_Two_Sum(Q, fnow, Qnew, hh);
                        fnow = f[++findex];
                    }
                    Q = Qnew;
                    if (hh != Scalar(0))
                    {
                        h[hindex++] = hh;
                    }
                }
            }
            while (eindex < elen)
            {
                DETRIA_Two_Sum(Q, enow, Qnew, hh);
                enow = e[++eindex];
                Q = Qnew;
                if (hh != Scalar(0))
                {
                    h[hindex++] = hh;
                }
            }
            while (findex < flen)
            {
                DETRIA_Two_Sum(Q, fnow, Qnew, hh);
                fnow = f[++findex];
                Q = Qnew;
                if (hh != Scalar(0))
                {
                    h[hindex++] = hh;
                }
            }
            if ((Q != Scalar(0)) || (hindex == 0))
            {
                h[hindex++] = Q;
            }
            return hindex;
        }

        template <typename Scalar>
        Scalar orient2dadapt(Scalar pa[2], Scalar pb[2], Scalar pc[2], Scalar detsum)
        {
            DETRIA_INEXACT Scalar acx, acy, bcx, bcy;
            Scalar acxtail, acytail, bcxtail, bcytail;
            DETRIA_INEXACT Scalar detleft, detright;
            Scalar detlefttail, detrighttail;
            Scalar det, errbound;
            Scalar B[4]{ }, C1[8]{ }, C2[12]{ }, D[16]{ };
            DETRIA_INEXACT Scalar B3;
            int C1length, C2length, Dlength;
            Scalar u[4]{ };
            DETRIA_INEXACT Scalar u3;
            DETRIA_INEXACT Scalar s1, t1;
            Scalar s0, t0;

            DETRIA_INEXACT Scalar bvirt;
            Scalar avirt, bround, around;
            DETRIA_INEXACT Scalar c;
            DETRIA_INEXACT Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;
            DETRIA_INEXACT Scalar _i, _j;
            Scalar _0;

            acx = pa[0] - pc[0];
            bcx = pb[0] - pc[0];
            acy = pa[1] - pc[1];
            bcy = pb[1] - pc[1];

            DETRIA_Two_Product(acx, bcy, detleft, detlefttail);
            DETRIA_Two_Product(acy, bcx, detright, detrighttail);

            DETRIA_Two_Two_Diff(detleft, detlefttail, detright, detrighttail, B3, B[2], B[1], B[0]);
            B[3] = B3;

            det = estimate(4, B);
            errbound = errorBounds<Scalar>.ccwerrboundB * detsum;
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            DETRIA_Two_Diff_Tail(pa[0], pc[0], acx, acxtail);
            DETRIA_Two_Diff_Tail(pb[0], pc[0], bcx, bcxtail);
            DETRIA_Two_Diff_Tail(pa[1], pc[1], acy, acytail);
            DETRIA_Two_Diff_Tail(pb[1], pc[1], bcy, bcytail);

            if ((acxtail == Scalar(0)) && (acytail == Scalar(0)) && (bcxtail == Scalar(0)) && (bcytail == Scalar(0)))
            {
                return det;
            }

            errbound = errorBounds<Scalar>.ccwerrboundC * detsum + errorBounds<Scalar>.resulterrbound * Absolute(det);
            det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            DETRIA_Two_Product(acxtail, bcy, s1, s0);
            DETRIA_Two_Product(acytail, bcx, t1, t0);
            DETRIA_Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
            u[3] = u3;
            C1length = fast_expansion_sum_zeroelim(4, B, 4, u, C1);

            DETRIA_Two_Product(acx, bcytail, s1, s0);
            DETRIA_Two_Product(acy, bcxtail, t1, t0);
            DETRIA_Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
            u[3] = u3;
            C2length = fast_expansion_sum_zeroelim(C1length, C1, 4, u, C2);

            DETRIA_Two_Product(acxtail, bcytail, s1, s0);
            DETRIA_Two_Product(acytail, bcxtail, t1, t0);
            DETRIA_Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
            u[3] = u3;
            Dlength = fast_expansion_sum_zeroelim(C2length, C2, 4, u, D);

            return(D[Dlength - 1]);
        }

        template <bool Robust, typename Scalar>
        inline Scalar orient2d(Scalar pa[2], Scalar pb[2], Scalar pc[2])
        {
            Scalar detsum = Scalar(0);

            Scalar detleft = (pa[0] - pc[0]) * (pb[1] - pc[1]);
            Scalar detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
            Scalar det = detleft - detright;

            if constexpr (Robust)
            {
                if (detleft > Scalar(0))
                {
                    if (detright <= Scalar(0))
                    {
                        return det;
                    }
                    else
                    {
                        detsum = detleft + detright;
                    }
                }
                else if (detleft < Scalar(0))
                {
                    if (detright >= Scalar(0))
                    {
                        return det;
                    }
                    else
                    {
                        detsum = -detleft - detright;
                    }
                }
                else
                {
                    return det;
                }

                Scalar errbound = errorBounds<Scalar>.ccwerrboundA * detsum;
                if (Absolute(det) >= errbound) DETRIA_LIKELY
                {
                    return det;
                }

                return orient2dadapt(pa, pb, pc, detsum);
            }
            else
            {
                return det;
            }
        }

        template <typename Scalar>
        int scale_expansion_zeroelim(int elen, const Scalar* e, Scalar b, Scalar* h)
        {
            DETRIA_INEXACT Scalar Q, sum;
            Scalar hh;
            DETRIA_INEXACT Scalar product1;
            Scalar product0;
            int eindex, hindex;
            Scalar enow;
            DETRIA_INEXACT Scalar bvirt;
            Scalar avirt, bround, around;
            DETRIA_INEXACT Scalar c;
            DETRIA_INEXACT Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;

            DETRIA_Split(b, bhi, blo);
            DETRIA_Two_Product_Presplit(e[0], b, bhi, blo, Q, hh);
            hindex = 0;
            if (hh != Scalar(0))
            {
                h[hindex++] = hh;
            }

            for (eindex = 1; eindex < elen; eindex++)
            {
                enow = e[eindex];
                DETRIA_Two_Product_Presplit(enow, b, bhi, blo, product1, product0);
                DETRIA_Two_Sum(Q, product0, sum, hh);
                if (hh != Scalar(0))
                {
                    h[hindex++] = hh;
                }

                DETRIA_Fast_Two_Sum(product1, sum, Q, hh);
                if (hh != Scalar(0))
                {
                    h[hindex++] = hh;
                }
            }

            if (Q != Scalar(0) || hindex == 0)
            {
                h[hindex++] = Q;
            }

            return hindex;
        }

        template <typename Scalar>
        Scalar incircleadapt(Scalar pa[2], Scalar pb[2], Scalar pc[2], Scalar pd[2], Scalar permanent)
        {
            DETRIA_INEXACT Scalar adx, bdx, cdx, ady, bdy, cdy;
            Scalar det, errbound;

            DETRIA_INEXACT Scalar bdxcdy1, cdxbdy1, cdxady1, adxcdy1, adxbdy1, bdxady1;
            Scalar bdxcdy0, cdxbdy0, cdxady0, adxcdy0, adxbdy0, bdxady0;
            Scalar bc[4], ca[4], ab[4];
            DETRIA_INEXACT Scalar bc3, ca3, ab3;
            Scalar axbc[8], axxbc[16], aybc[8], ayybc[16], adet[32];
            int axbclen, axxbclen, aybclen, ayybclen, alen;
            Scalar bxca[8], bxxca[16], byca[8], byyca[16], bdet[32];
            int bxcalen, bxxcalen, bycalen, byycalen, blen;
            Scalar cxab[8], cxxab[16], cyab[8], cyyab[16], cdet[32];
            int cxablen, cxxablen, cyablen, cyyablen, clen;
            Scalar abdet[64];
            int ablen;
            Scalar fin1[1152], fin2[1152];
            Scalar* finnow, * finother, * finswap;
            int finlength;

            Scalar adxtail, bdxtail, cdxtail, adytail, bdytail, cdytail;
            DETRIA_INEXACT Scalar adxadx1, adyady1, bdxbdx1, bdybdy1, cdxcdx1, cdycdy1;
            Scalar adxadx0, adyady0, bdxbdx0, bdybdy0, cdxcdx0, cdycdy0;
            Scalar aa[4], bb[4], cc[4];
            DETRIA_INEXACT Scalar aa3, bb3, cc3;
            DETRIA_INEXACT Scalar ti1, tj1;
            Scalar ti0, tj0;
            Scalar u[4], v[4];
            DETRIA_INEXACT Scalar u3, v3;
            Scalar temp8[8], temp16a[16], temp16b[16], temp16c[16];
            Scalar temp32a[32], temp32b[32], temp48[48], temp64[64];
            int temp8len, temp16alen, temp16blen, temp16clen;
            int temp32alen, temp32blen, temp48len, temp64len;
            Scalar axtbb[8], axtcc[8], aytbb[8], aytcc[8];
            int axtbblen, axtcclen, aytbblen, aytcclen;
            Scalar bxtaa[8], bxtcc[8], bytaa[8], bytcc[8];
            int bxtaalen, bxtcclen, bytaalen, bytcclen;
            Scalar cxtaa[8], cxtbb[8], cytaa[8], cytbb[8];
            int cxtaalen, cxtbblen, cytaalen, cytbblen;
            Scalar axtbc[8], aytbc[8], bxtca[8], bytca[8], cxtab[8], cytab[8];
            int axtbclen, aytbclen, bxtcalen, bytcalen, cxtablen, cytablen;
            Scalar axtbct[16], aytbct[16], bxtcat[16], bytcat[16], cxtabt[16], cytabt[16];
            int axtbctlen, aytbctlen, bxtcatlen, bytcatlen, cxtabtlen, cytabtlen;
            Scalar axtbctt[8], aytbctt[8], bxtcatt[8];
            Scalar bytcatt[8], cxtabtt[8], cytabtt[8];
            int axtbcttlen, aytbcttlen, bxtcattlen, bytcattlen, cxtabttlen, cytabttlen;
            Scalar abt[8], bct[8], cat[8];
            int abtlen, bctlen, catlen;
            Scalar abtt[4], bctt[4], catt[4];
            int abttlen, bcttlen, cattlen;
            DETRIA_INEXACT Scalar abtt3, bctt3, catt3;
            Scalar negate;

            DETRIA_INEXACT Scalar bvirt;
            Scalar avirt, bround, around;
            DETRIA_INEXACT Scalar c;
            DETRIA_INEXACT Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;
            DETRIA_INEXACT Scalar _i, _j;
            Scalar _0;

            axtbclen = 0;
            aytbclen = 0;
            bxtcalen = 0;
            bytcalen = 0;
            cxtablen = 0;
            cytablen = 0;

            adx = Scalar(pa[0] - pd[0]);
            bdx = Scalar(pb[0] - pd[0]);
            cdx = Scalar(pc[0] - pd[0]);
            ady = Scalar(pa[1] - pd[1]);
            bdy = Scalar(pb[1] - pd[1]);
            cdy = Scalar(pc[1] - pd[1]);

            DETRIA_Two_Product(bdx, cdy, bdxcdy1, bdxcdy0);
            DETRIA_Two_Product(cdx, bdy, cdxbdy1, cdxbdy0);
            DETRIA_Two_Two_Diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0, bc3, bc[2], bc[1], bc[0]);
            bc[3] = bc3;
            axbclen = scale_expansion_zeroelim(4, bc, adx, axbc);
            axxbclen = scale_expansion_zeroelim(axbclen, axbc, adx, axxbc);
            aybclen = scale_expansion_zeroelim(4, bc, ady, aybc);
            ayybclen = scale_expansion_zeroelim(aybclen, aybc, ady, ayybc);
            alen = fast_expansion_sum_zeroelim(axxbclen, axxbc, ayybclen, ayybc, adet);

            DETRIA_Two_Product(cdx, ady, cdxady1, cdxady0);
            DETRIA_Two_Product(adx, cdy, adxcdy1, adxcdy0);
            DETRIA_Two_Two_Diff(cdxady1, cdxady0, adxcdy1, adxcdy0, ca3, ca[2], ca[1], ca[0]);
            ca[3] = ca3;
            bxcalen = scale_expansion_zeroelim(4, ca, bdx, bxca);
            bxxcalen = scale_expansion_zeroelim(bxcalen, bxca, bdx, bxxca);
            bycalen = scale_expansion_zeroelim(4, ca, bdy, byca);
            byycalen = scale_expansion_zeroelim(bycalen, byca, bdy, byyca);
            blen = fast_expansion_sum_zeroelim(bxxcalen, bxxca, byycalen, byyca, bdet);

            DETRIA_Two_Product(adx, bdy, adxbdy1, adxbdy0);
            DETRIA_Two_Product(bdx, ady, bdxady1, bdxady0);
            DETRIA_Two_Two_Diff(adxbdy1, adxbdy0, bdxady1, bdxady0, ab3, ab[2], ab[1], ab[0]);
            ab[3] = ab3;
            cxablen = scale_expansion_zeroelim(4, ab, cdx, cxab);
            cxxablen = scale_expansion_zeroelim(cxablen, cxab, cdx, cxxab);
            cyablen = scale_expansion_zeroelim(4, ab, cdy, cyab);
            cyyablen = scale_expansion_zeroelim(cyablen, cyab, cdy, cyyab);
            clen = fast_expansion_sum_zeroelim(cxxablen, cxxab, cyyablen, cyyab, cdet);

            ablen = fast_expansion_sum_zeroelim(alen, adet, blen, bdet, abdet);
            finlength = fast_expansion_sum_zeroelim(ablen, abdet, clen, cdet, fin1);

            det = estimate(finlength, fin1);
            errbound = errorBounds<Scalar>.iccerrboundB * permanent;
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            DETRIA_Two_Diff_Tail(pa[0], pd[0], adx, adxtail);
            DETRIA_Two_Diff_Tail(pa[1], pd[1], ady, adytail);
            DETRIA_Two_Diff_Tail(pb[0], pd[0], bdx, bdxtail);
            DETRIA_Two_Diff_Tail(pb[1], pd[1], bdy, bdytail);
            DETRIA_Two_Diff_Tail(pc[0], pd[0], cdx, cdxtail);
            DETRIA_Two_Diff_Tail(pc[1], pd[1], cdy, cdytail);

            if (adxtail == Scalar(0) && bdxtail == Scalar(0) && cdxtail == Scalar(0) &&
                adytail == Scalar(0) && bdytail == Scalar(0) && cdytail == Scalar(0))
            {
                return det;
            }

            errbound = errorBounds<Scalar>.iccerrboundC * permanent + errorBounds<Scalar>.resulterrbound * Absolute(det);
            det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail)
                - (bdy * cdxtail + cdx * bdytail))
                + Scalar(2) * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
                + ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail)
                    - (cdy * adxtail + adx * cdytail))
                    + Scalar(2) * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
                + ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail)
                    - (ady * bdxtail + bdx * adytail))
                    + Scalar(2) * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));

            if (det >= errbound || -det >= errbound)
            {
                return det;
            }

            finnow = fin1;
            finother = fin2;

            if (bdxtail != Scalar(0) || bdytail != Scalar(0) || cdxtail != Scalar(0) || cdytail != Scalar(0))
            {
                DETRIA_Square(adx, adxadx1, adxadx0);
                DETRIA_Square(ady, adyady1, adyady0);
                DETRIA_Two_Two_Sum(adxadx1, adxadx0, adyady1, adyady0, aa3, aa[2], aa[1], aa[0]);
                aa[3] = aa3;
            }
            if (cdxtail != Scalar(0) || cdytail != Scalar(0) || adxtail != Scalar(0) || adytail != Scalar(0))
            {
                DETRIA_Square(bdx, bdxbdx1, bdxbdx0);
                DETRIA_Square(bdy, bdybdy1, bdybdy0);
                DETRIA_Two_Two_Sum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0, bb3, bb[2], bb[1], bb[0]);
                bb[3] = bb3;
            }
            if (adxtail != Scalar(0) || adytail != Scalar(0) || bdxtail != Scalar(0) || bdytail != Scalar(0))
            {
                DETRIA_Square(cdx, cdxcdx1, cdxcdx0);
                DETRIA_Square(cdy, cdycdy1, cdycdy0);
                DETRIA_Two_Two_Sum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0, cc3, cc[2], cc[1], cc[0]);
                cc[3] = cc3;
            }

            if (adxtail != Scalar(0))
            {
                axtbclen = scale_expansion_zeroelim(4, bc, adxtail, axtbc);
                temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, Scalar(2) * adx, temp16a);

                axtcclen = scale_expansion_zeroelim(4, cc, adxtail, axtcc);
                temp16blen = scale_expansion_zeroelim(axtcclen, axtcc, bdy, temp16b);

                axtbblen = scale_expansion_zeroelim(4, bb, adxtail, axtbb);
                temp16clen = scale_expansion_zeroelim(axtbblen, axtbb, -cdy, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (adytail != Scalar(0))
            {
                aytbclen = scale_expansion_zeroelim(4, bc, adytail, aytbc);
                temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, Scalar(2) * ady, temp16a);

                aytbblen = scale_expansion_zeroelim(4, bb, adytail, aytbb);
                temp16blen = scale_expansion_zeroelim(aytbblen, aytbb, cdx, temp16b);

                aytcclen = scale_expansion_zeroelim(4, cc, adytail, aytcc);
                temp16clen = scale_expansion_zeroelim(aytcclen, aytcc, -bdx, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (bdxtail != Scalar(0))
            {
                bxtcalen = scale_expansion_zeroelim(4, ca, bdxtail, bxtca);
                temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, Scalar(2) * bdx, temp16a);

                bxtaalen = scale_expansion_zeroelim(4, aa, bdxtail, bxtaa);
                temp16blen = scale_expansion_zeroelim(bxtaalen, bxtaa, cdy, temp16b);

                bxtcclen = scale_expansion_zeroelim(4, cc, bdxtail, bxtcc);
                temp16clen = scale_expansion_zeroelim(bxtcclen, bxtcc, -ady, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (bdytail != Scalar(0))
            {
                bytcalen = scale_expansion_zeroelim(4, ca, bdytail, bytca);
                temp16alen = scale_expansion_zeroelim(bytcalen, bytca, Scalar(2) * bdy, temp16a);

                bytcclen = scale_expansion_zeroelim(4, cc, bdytail, bytcc);
                temp16blen = scale_expansion_zeroelim(bytcclen, bytcc, adx, temp16b);

                bytaalen = scale_expansion_zeroelim(4, aa, bdytail, bytaa);
                temp16clen = scale_expansion_zeroelim(bytaalen, bytaa, -cdx, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (cdxtail != Scalar(0))
            {
                cxtablen = scale_expansion_zeroelim(4, ab, cdxtail, cxtab);
                temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, Scalar(2) * cdx, temp16a);

                cxtbblen = scale_expansion_zeroelim(4, bb, cdxtail, cxtbb);
                temp16blen = scale_expansion_zeroelim(cxtbblen, cxtbb, ady, temp16b);

                cxtaalen = scale_expansion_zeroelim(4, aa, cdxtail, cxtaa);
                temp16clen = scale_expansion_zeroelim(cxtaalen, cxtaa, -bdy, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (cdytail != Scalar(0))
            {
                cytablen = scale_expansion_zeroelim(4, ab, cdytail, cytab);
                temp16alen = scale_expansion_zeroelim(cytablen, cytab, Scalar(2) * cdy, temp16a);

                cytaalen = scale_expansion_zeroelim(4, aa, cdytail, cytaa);
                temp16blen = scale_expansion_zeroelim(cytaalen, cytaa, bdx, temp16b);

                cytbblen = scale_expansion_zeroelim(4, bb, cdytail, cytbb);
                temp16clen = scale_expansion_zeroelim(cytbblen, cytbb, -adx, temp16c);

                temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (adxtail != Scalar(0) || adytail != Scalar(0))
            {
                if (bdxtail != Scalar(0) || bdytail != Scalar(0) || cdxtail != Scalar(0) || cdytail != Scalar(0))
                {
                    DETRIA_Two_Product(bdxtail, cdy, ti1, ti0);
                    DETRIA_Two_Product(bdx, cdytail, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
                    u[3] = u3;
                    negate = -bdy;
                    DETRIA_Two_Product(cdxtail, negate, ti1, ti0);
                    negate = -bdytail;
                    DETRIA_Two_Product(cdx, negate, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
                    v[3] = v3;
                    bctlen = fast_expansion_sum_zeroelim(4, u, 4, v, bct);

                    DETRIA_Two_Product(bdxtail, cdytail, ti1, ti0);
                    DETRIA_Two_Product(cdxtail, bdytail, tj1, tj0);
                    DETRIA_Two_Two_Diff(ti1, ti0, tj1, tj0, bctt3, bctt[2], bctt[1], bctt[0]);
                    bctt[3] = bctt3;
                    bcttlen = 4;
                }
                else
                {
                    bct[0] = Scalar(0);
                    bctlen = 1;
                    bctt[0] = Scalar(0);
                    bcttlen = 1;
                }

                if (adxtail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, adxtail, temp16a);
                    axtbctlen = scale_expansion_zeroelim(bctlen, bct, adxtail, axtbct);
                    temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, Scalar(2) * adx, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (bdytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, cc, adxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (cdytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, bb, -adxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, adxtail, temp32a);
                    axtbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adxtail, axtbctt);
                    temp16alen = scale_expansion_zeroelim(axtbcttlen, axtbctt, Scalar(2) * adx, temp16a);
                    temp16blen = scale_expansion_zeroelim(axtbcttlen, axtbctt, adxtail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (adytail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, adytail, temp16a);
                    aytbctlen = scale_expansion_zeroelim(bctlen, bct, adytail, aytbct);
                    temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, Scalar(2) * ady, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, adytail, temp32a);
                    aytbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adytail, aytbctt);
                    temp16alen = scale_expansion_zeroelim(aytbcttlen, aytbctt, Scalar(2) * ady, temp16a);
                    temp16blen = scale_expansion_zeroelim(aytbcttlen, aytbctt, adytail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
            }

            if (bdxtail != Scalar(0) || bdytail != Scalar(0))
            {
                if (cdxtail != Scalar(0) || cdytail != Scalar(0) || adxtail != Scalar(0) || adytail != Scalar(0))
                {
                    DETRIA_Two_Product(cdxtail, ady, ti1, ti0);
                    DETRIA_Two_Product(cdx, adytail, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
                    u[3] = u3;
                    negate = -cdy;
                    DETRIA_Two_Product(adxtail, negate, ti1, ti0);
                    negate = -cdytail;
                    DETRIA_Two_Product(adx, negate, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
                    v[3] = v3;
                    catlen = fast_expansion_sum_zeroelim(4, u, 4, v, cat);

                    DETRIA_Two_Product(cdxtail, adytail, ti1, ti0);
                    DETRIA_Two_Product(adxtail, cdytail, tj1, tj0);
                    DETRIA_Two_Two_Diff(ti1, ti0, tj1, tj0, catt3, catt[2], catt[1], catt[0]);
                    catt[3] = catt3;
                    cattlen = 4;
                }
                else
                {
                    cat[0] = Scalar(0);
                    catlen = 1;
                    catt[0] = Scalar(0);
                    cattlen = 1;
                }

                if (bdxtail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, bdxtail, temp16a);
                    bxtcatlen = scale_expansion_zeroelim(catlen, cat, bdxtail, bxtcat);
                    temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, Scalar(2) * bdx, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (cdytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, aa, bdxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (adytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, cc, -bdxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, bdxtail, temp32a);
                    bxtcattlen = scale_expansion_zeroelim(cattlen, catt, bdxtail, bxtcatt);
                    temp16alen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, Scalar(2) * bdx, temp16a);
                    temp16blen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, bdxtail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (bdytail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(bytcalen, bytca, bdytail, temp16a);
                    bytcatlen = scale_expansion_zeroelim(catlen, cat, bdytail, bytcat);
                    temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, Scalar(2) * bdy, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, bdytail, temp32a);
                    bytcattlen = scale_expansion_zeroelim(cattlen, catt, bdytail, bytcatt);
                    temp16alen = scale_expansion_zeroelim(bytcattlen, bytcatt, Scalar(2) * bdy, temp16a);
                    temp16blen = scale_expansion_zeroelim(bytcattlen, bytcatt, bdytail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
            }

            if (cdxtail != Scalar(0) || cdytail != Scalar(0))
            {
                if (adxtail != Scalar(0) || adytail != Scalar(0) || bdxtail != Scalar(0) || bdytail != Scalar(0))
                {
                    DETRIA_Two_Product(adxtail, bdy, ti1, ti0);
                    DETRIA_Two_Product(adx, bdytail, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
                    u[3] = u3;
                    negate = -ady;
                    DETRIA_Two_Product(bdxtail, negate, ti1, ti0);
                    negate = -adytail;
                    DETRIA_Two_Product(bdx, negate, tj1, tj0);
                    DETRIA_Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
                    v[3] = v3;
                    abtlen = fast_expansion_sum_zeroelim(4, u, 4, v, abt);

                    DETRIA_Two_Product(adxtail, bdytail, ti1, ti0);
                    DETRIA_Two_Product(bdxtail, adytail, tj1, tj0);
                    DETRIA_Two_Two_Diff(ti1, ti0, tj1, tj0, abtt3, abtt[2], abtt[1], abtt[0]);
                    abtt[3] = abtt3;
                    abttlen = 4;
                }
                else
                {
                    abt[0] = Scalar(0);
                    abtlen = 1;
                    abtt[0] = Scalar(0);
                    abttlen = 1;
                }

                if (cdxtail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, cdxtail, temp16a);
                    cxtabtlen = scale_expansion_zeroelim(abtlen, abt, cdxtail, cxtabt);
                    temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, Scalar(2) * cdx, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (adytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, bb, cdxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (bdytail != Scalar(0))
                    {
                        temp8len = scale_expansion_zeroelim(4, aa, -cdxtail, temp8);
                        temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail, temp16a);
                        finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, cdxtail, temp32a);
                    cxtabttlen = scale_expansion_zeroelim(abttlen, abtt, cdxtail, cxtabtt);
                    temp16alen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, Scalar(2) * cdx, temp16a);
                    temp16blen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, cdxtail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (cdytail != Scalar(0))
                {
                    temp16alen = scale_expansion_zeroelim(cytablen, cytab, cdytail, temp16a);
                    cytabtlen = scale_expansion_zeroelim(abtlen, abt, cdytail, cytabt);
                    temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, Scalar(2) * cdy, temp32a);
                    temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, cdytail, temp32a);
                    cytabttlen = scale_expansion_zeroelim(abttlen, abtt, cdytail, cytabtt);
                    temp16alen = scale_expansion_zeroelim(cytabttlen, cytabtt, Scalar(2) * cdy, temp16a);
                    temp16blen = scale_expansion_zeroelim(cytabttlen, cytabtt, cdytail, temp16b);
                    temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
            }

            return finnow[finlength - 1];
        }

        template <bool Robust, typename Scalar>
        inline Scalar incircle(Scalar pa[2], Scalar pb[2], Scalar pc[2], Scalar pd[2])
        {
            Scalar adx = pa[0] - pd[0];
            Scalar bdx = pb[0] - pd[0];
            Scalar cdx = pc[0] - pd[0];
            Scalar ady = pa[1] - pd[1];
            Scalar bdy = pb[1] - pd[1];
            Scalar cdy = pc[1] - pd[1];

            Scalar bdxcdy = bdx * cdy;
            Scalar cdxbdy = cdx * bdy;
            Scalar alift = adx * adx + ady * ady;

            Scalar cdxady = cdx * ady;
            Scalar adxcdy = adx * cdy;
            Scalar blift = bdx * bdx + bdy * bdy;

            Scalar adxbdy = adx * bdy;
            Scalar bdxady = bdx * ady;
            Scalar clift = cdx * cdx + cdy * cdy;

            Scalar det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
            if constexpr (Robust)
            {
                Scalar permanent = (Absolute(bdxcdy) + Absolute(cdxbdy)) * alift
                    + (Absolute(cdxady) + Absolute(adxcdy)) * blift
                    + (Absolute(adxbdy) + Absolute(bdxady)) * clift;

                Scalar errbound = errorBounds<Scalar>.iccerrboundA * permanent;
                if (Absolute(det) > errbound) DETRIA_LIKELY
                {
                    return det;
                }

                return incircleadapt(pa, pb, pc, pd, permanent);
            }
            else
            {
                return det;
            }
        }

#undef DETRIA_INEXACT
#undef DETRIA_Fast_Two_Sum_Tail
#undef DETRIA_Fast_Two_Sum
#undef DETRIA_Two_Sum_Tail
#undef DETRIA_Two_Sum
#undef DETRIA_Two_Diff_Tail
#undef DETRIA_Two_Diff
#undef DETRIA_Split
#undef DETRIA_Two_Product_Tail
#undef DETRIA_Two_Product
#undef DETRIA_Two_Product_Presplit
#undef DETRIA_Square_Tail
#undef DETRIA_Square
#undef DETRIA_Two_One_Sum
#undef DETRIA_Two_One_Diff
#undef DETRIA_Two_Two_Sum
#undef DETRIA_Two_Two_Diff
    };

    namespace detail
    {
        // forward declare

        bool detriaAssert(bool condition, const char* message = nullptr);
    }

    namespace math
    {
        enum class Orientation : uint8_t
        {
            CW = 0, CCW = 1, Collinear = 2
        };

        template <bool Robust, typename Vec2>
        inline Orientation orient2d(const Vec2& a, const Vec2& b, const Vec2& c)
        {
            using Scalar = decltype(Vec2::x);
            Scalar pa[2]{ a.x, a.y };
            Scalar pb[2]{ b.x, b.y };
            Scalar pc[2]{ c.x, c.y };
            Scalar result = predicates::orient2d<Robust, Scalar>(pa, pb, pc);
            if (result < Scalar(0))
            {
                return Orientation::CW;
            }
            else if (result > Scalar(0))
            {
                return Orientation::CCW;
            }
            else
            {
                return Orientation::Collinear;
            }
        }

        enum class CircleLocation : uint8_t
        {
            Inside = 0, Outside = 1, Cocircular = 2
        };

        template <bool Robust, typename Vec2>
        inline CircleLocation incircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d)
        {
#ifndef NDEBUG
            // The points pa, pb, and pc must be in counterclockwise order, or the sign of the result will be reversed.
            detail::detriaAssert(math::orient2d<Robust, Vec2>(a, b, c) == math::Orientation::CCW);
#endif

            using Scalar = decltype(Vec2::x);
            Scalar pa[2]{ a.x, a.y };
            Scalar pb[2]{ b.x, b.y };
            Scalar pc[2]{ c.x, c.y };
            Scalar pd[2]{ d.x, d.y };
            Scalar result = predicates::incircle<Robust, Scalar>(pa, pb, pc, pd);
            if (result > Scalar(0))
            {
                return CircleLocation::Inside;
            }
            else if (result < Scalar(0))
            {
                return CircleLocation::Outside;
            }
            else
            {
                return CircleLocation::Cocircular;
            }
        }

        inline constexpr bool isPowerOfTwo(uint64_t num)
        {
            return (num != 0) && ((num - 1) & num) == 0;
        }

        inline constexpr uint64_t nextPowerOfTwo(uint64_t num)
        {
            num |= num >> 1;
            num |= num >> 2;
            num |= num >> 4;
            num |= num >> 8;
            num |= num >> 16;
            num |= num >> 32;
            return num + 1;
        }

        inline constexpr uint64_t nextOrEqualPowerOfTwo(uint64_t num)
        {
            if (!isPowerOfTwo(num))
            {
                num = nextPowerOfTwo(num);
            }

            return num;
        }
    }

    namespace memory
    {
        struct DefaultAllocator
        {
            template <typename T>
            struct StlAllocator
            {
                using value_type = T;
                using size_type = size_t;
                using difference_type = ptrdiff_t;

                StlAllocator()
                {
                }

                template <typename U>
                StlAllocator(const StlAllocator<U>&)
                {
                }

                inline T* allocate(size_t count)
                {
                    return new T[count];
                }

                inline void deallocate(T* ptr, size_t)
                {
                    delete[] ptr;
                }
            };

            template <typename T>
            inline StlAllocator<T> createStlAllocator()
            {
                return StlAllocator<T>();
            }
        };
    }

    namespace detail
    {
        // simple zero-sized struct
        struct Empty { };

        template <typename T, typename Idx, template <typename, typename> typename Collection, typename Allocator>
        struct FlatLinkedList
        {
            // simple cyclic doubly-linked list, but the elements are not allocated one-by-one
            // the indices of the deleted elements are stored in `free`

            struct Node
            {
                Idx prevId;
                Idx nextId;
                T data;
            };

            FlatLinkedList(Allocator allocator) :
                _nodes(allocator.template createStlAllocator<Node>()),
                _free(allocator.template createStlAllocator<Idx>())
            {
            }

            inline Idx create(const T& data)
            {
                Idx newId{ };
                Node* newNode = nullptr;
                if (_free.empty())
                {
                    newId = Idx(_nodes.size());
                    newNode = &_nodes.emplace_back();
                }
                else
                {
                    newId = _free.back();
                    _free.pop_back();
                    newNode = &_nodes[size_t(newId)];
                }

                newNode->data = data;
                newNode->prevId = newId;
                newNode->nextId = newId;

                return newId;
            }

            inline Idx addAfter(const Idx& afterId, const T& data)
            {
                Idx nodeId = create(data);
                Node& node = _nodes[size_t(nodeId)];
                Node& after = _nodes[size_t(afterId)];

                _nodes[size_t(after.nextId)].prevId = nodeId;
                node.nextId = after.nextId;
                after.nextId = nodeId;
                node.prevId = afterId;

                return nodeId;
            }

            inline Idx addBefore(const Idx& beforeId, const T& data)
            {
                Idx nodeId = create(data);
                Node& node = _nodes[size_t(nodeId)];
                Node& before = _nodes[size_t(beforeId)];

                _nodes[size_t(before.prevId)].nextId = nodeId;
                node.prevId = before.prevId;
                before.prevId = nodeId;
                node.nextId = before;

                return nodeId;
            }

            inline void remove(const Idx& nodeId)
            {
                Node& node = _nodes[size_t(nodeId)];
                _nodes[size_t(node.prevId)].nextId = node.nextId;
                _nodes[size_t(node.nextId)].prevId = node.prevId;

                _free.push_back(nodeId);
            }

            inline const Node& getNode(const Idx& nodeId) const
            {
                return _nodes[size_t(nodeId)];
            }

            inline size_t size() const
            {
                return _nodes.size();
            }

            inline void clear()
            {
                _nodes.clear();
                _free.clear();
            }

        private:
            Collection<Node, typename Allocator::template StlAllocator<Node>> _nodes;
            Collection<Idx, typename Allocator::template StlAllocator<Idx>> _free;
        };

        inline bool detriaAssert(bool condition, const char* message)
        {
            if (condition)
            {
                return true;
            }

#ifdef NDEBUG
            (void)message;
#else
            // only in debug mode

            std::cerr << "Assertion failed";
            if (message)
            {
                std::cerr << ": " << message;
            }

            std::cerr << std::endl;

#if defined(_WIN32) && _WIN32
            __debugbreak();
#else
#if defined(SIGTRAP)
            std::raise(SIGTRAP);
#endif
#endif

#endif
            return false;
        }

        // similar to std::span, but without C++20, also much simpler
        template <typename T>
        struct ReadonlySpan
        {
            ReadonlySpan() : _ptr(nullptr), _count(0)
            {
            }

            ReadonlySpan(const T* ptr, size_t count) : _ptr(ptr), _count(count)
            {
            }

            template <template <typename, typename> typename Collection, typename Allocator>
            ReadonlySpan(const Collection<T, Allocator>& collection) : _ptr(collection.data()), _count(collection.size())
            {
            }

            inline size_t size() const
            {
                return _count;
            }

            inline void reset()
            {
                _ptr = nullptr;
                _count = 0;
            }

            inline const T& front() const
            {
                return _ptr[0];
            }

            inline const T& back() const
            {
                return _ptr[_count - 1];
            }

            inline const T& operator[](size_t index) const
            {
                return _ptr[index];
            }

        private:
            const T* _ptr;
            size_t _count;
        };

        // helper to decide if a type has a `reserve` function
        template <template <typename, typename> typename, typename = void>
        struct HasReserve
        {
            static constexpr bool value = false;
        };

        template <template <typename, typename> typename T>
        struct HasReserve<T, std::void_t<decltype(T<char, memory::DefaultAllocator::StlAllocator<char>>().reserve(size_t()))>>
        {
            static constexpr bool value = true;
        };
    }

    namespace topology
    {
        template <typename Idx>
        inline Idx next3(const Idx& idx)
        {
            // 0 -> 1
            // 1 -> 2
            // 2 -> 0
            return idx == 2 ? 0 : idx + 1;
        }

        template <typename Idx>
        inline Idx prev3(const Idx& idx)
        {
            // 0 -> 2
            // 1 -> 0
            // 2 -> 1
            return idx == 0 ? 2 : idx - 1;
        }

        template <typename Idx>
        inline Idx other3(const Idx& a, const Idx& b)
        {
            // 0 1 or 1 0 -> 2
            // 0 2 or 2 0 -> 1
            // 1 2 or 2 1 -> 0
            return 3 - (a + b);
        }

        template <typename T, typename Idx, template <typename, typename> typename Collection, typename Allocator>
        class FreeList
        {
            // class to store elements sequentially, with the ability to delete elements from anywhere
            // `firstFree` indicates the first free slot (if any), that free slot indicates the next one (if any), and so on

            static constexpr bool collectionHasReserve = detail::HasReserve<Collection>::value;

        public:
            FreeList(Allocator allocator) : _allData(allocator.template createStlAllocator<Data>())
            {
            }

            inline void reserve(Idx capacity)
            {
                if constexpr (collectionHasReserve)
                {
                    _allData.reserve(size_t(capacity));
                }
            }

            inline Idx add(const T& data)
            {
                Idx result{ };
                if (_firstFree.has_value())
                {
                    // free slot available, add it there
                    result = *_firstFree;
                    Data& currentData = _allData[size_t(result)];

                    if (const FreeData* freeData = std::get_if<FreeData>(&currentData))
                    {
                        _firstFree = freeData->nextFree;
                    }
                    else
                    {
                        detail::detriaAssert(false, "Free slot expected");
                    }

                    currentData = ValidData{ data };
                }
                else
                {
                    // no free slots, add at the end
                    result = Idx(_allData.size());
                    _allData.emplace_back(ValidData{ data });
                }

                ++_validCount;

                return result;
            }

            inline bool remove(Idx index)
            {
                if (!isValid(index) || index > Idx(_allData.size())) DETRIA_UNLIKELY
                {
                    return false;
                }

                if (index == Idx(_allData.size() - 1))
                {
                    // remove from the end
                    _allData.pop_back();
                }
                else
                {
                    _allData[size_t(index)] = FreeData{ _firstFree };
                    _firstFree = index;
                }

                --_validCount;
                return true;
            }

            inline bool isValid(const Idx& id) const
            {
                return size_t(id) < _allData.size() && std::holds_alternative<ValidData>(_allData[size_t(id)]);
            }

            inline void clear()
            {
                _validCount = 0;
                _firstFree.reset();
                _allData.clear();
            }

            // total number of valid + deleted elements
            inline Idx getUsedCount() const
            {
                return Idx(_allData.size());
            }

            inline const T* getData(const Idx& index) const
            {
                if (const ValidData* data = std::get_if<ValidData>(&_allData[size_t(index)])) DETRIA_LIKELY
                {
                    return &data->data;
                }

                detail::detriaAssert(false, "No valid element at the given index");
                return nullptr;
            }

            inline T* getData(const Idx& index)
            {
                if (ValidData* data = std::get_if<ValidData>(&_allData[size_t(index)])) DETRIA_LIKELY
                {
                    return &data->data;
                }

                detail::detriaAssert(false, "No valid element at the given index");
                return nullptr;
            }

            inline void forEachValid(auto&& callback) const
            {
                constexpr bool callbackReturnsBool = std::is_same_v<bool, std::invoke_result_t<decltype(callback), Idx>>;

                for (Idx i = 0; i < Idx(_allData.size()); ++i)
                {
                    if (!isValid(i))
                    {
                        continue;
                    }

                    if constexpr (callbackReturnsBool)
                    {
                        bool shouldContinue = callback(i);
                        if (!shouldContinue)
                        {
                            break;
                        }
                    }
                    else
                    {
                        callback(i);
                    }
                }
            }

        private:
            struct FreeData
            {
                std::optional<Idx> nextFree;
            };

            struct ValidData
            {
                T data;
            };

            using Data = std::variant<FreeData, ValidData>;

            Idx _validCount = 0; // number of valid, non-deleted elements

            Collection<Data, typename Allocator::template StlAllocator<Data>> _allData;
            std::optional<Idx> _firstFree{ };
        };

        template <typename Idx, Idx Degree, template <typename, typename> typename Collection, typename Allocator>
        class PrimitiveRelations
        {
            // class to store relations between primitives (e.g. triangle-vertex)
            // we always have two different primitive types:
            // triangle-vertex (3 vertices per triangle), triangle-edge (3 edges per triangle), edge-vertex (2 vertices per edge)
            // the parent type is the higher dimensional type, the child type is the lower dimensional one (e.g. parent: triangle, child: vertex)
            //
            // in the parent -> child direction, each element must have `Degree` primitives:
            // a triangle must always have 3 vertices and edges, and an edge must always have 2 vertices
            // in the child -> parent direction, each element can have any number of primitives:
            // a vertex can have any number of triangles and any number of edges,
            // and an edge can have any number of triangles (though in practice, we'll only have 2 triangles per edge at most)
            //
            // the `RelationId` type uniquely identifies a child of a parent, e.g. the second vertex of a triangle
            // it simply contains the parent's id shifted to the left, or-ed with the index of the child in the parent
            // for triangles, 4 child elements are stored for faster calculation (`something * 3` is slower to calculate than `something << 2`)
            // for example, vertex 2 of triangle 19 is stored at index 78 (== (19 << 2) | 2)
            // 
            // `_children` simply contains all children, indexed by `RelationId` values
            // (e.g. first vertex of first triangle, then second vertex of first triangle, etc.)
            // `_firstParent` and `_nextParent` work like a linked list
            // `_firstParent` is indexed with child id-s, and contains the first relation for each child (if any)
            // `_nextParent` is indexed with relation id-s, and contains the next relation (if any)
            // the end of the list is indicated with an `EmptyIndex` value

        private:
            static_assert(Degree == 2 || Degree == 3);
            // using power of 2 multipliers for faster multiplications / divisions
            // for example, for triangles, we use 4 elements instead of 3
            // the 4th element is never used, so it wastes some space, but still better than dividing by 3
            static constexpr Idx DegreePow2 = Degree <= 2 ? 2 : 4;
            static constexpr Idx Pow2 = DegreePow2 == 4 ? 2 : 1;

            static constexpr Idx EmptyIndex = Idx(-1);

        public:
            struct RelationId
            {
                inline RelationId(Idx value_ = EmptyIndex) : value(value_)
                {
                }

                Idx value;
            };

            PrimitiveRelations(Allocator allocator) :
                _children(allocator.template createStlAllocator<Idx>()),
                _firstParent(allocator.template createStlAllocator<RelationId>()),
                _nextParent(allocator.template createStlAllocator<RelationId>())
            {
            }

            inline void reserveChildToParent(Idx capacity)
            {
                if (_childToParentCapacity >= capacity)
                {
                    return;
                }

                capacity = Idx(math::nextOrEqualPowerOfTwo(uint64_t(capacity)));
                _nextParent.resize(size_t(capacity) << size_t(Pow2), EmptyIndex);
                _children.resize(size_t(capacity) << size_t(Pow2), EmptyIndex);

                _childToParentCapacity = capacity;
            }

            inline void reserveParentToChild(Idx capacity)
            {
                if (_parentToChildCapacity >= capacity)
                {
                    return;
                }

                capacity = Idx(math::nextOrEqualPowerOfTwo(uint64_t(capacity)));
                _firstParent.resize(size_t(capacity), EmptyIndex);

                _parentToChildCapacity = capacity;
            }

            inline static RelationId getRelationId(Idx parentId, Idx childIndexInParent)
            {
                return (parentId << Pow2) | childIndexInParent;
            }

            inline Idx getChildId(RelationId relationId) const
            {
                return _children[size_t(relationId.value)];
            }

            inline Idx getChildId(Idx parentId, Idx childIndexInParent) const
            {
                return getChildId(getRelationId(parentId, childIndexInParent));
            }

            inline Idx getChildIndexInParent(Idx parentId, Idx childId) const
            {
                for (Idx i = 0; i < Degree; ++i)
                {
                    if (_children[size_t(getRelationId(parentId, i).value)] == childId)
                    {
                        return i;
                    }
                }

                return EmptyIndex;
            }

            inline RelationId getFirstParent(Idx childId) const
            {
                return _firstParent[size_t(childId)];
            }

            inline RelationId getNextParent(RelationId relationId) const
            {
                return _nextParent[size_t(relationId.value)];
            }

            inline void addRelation(RelationId relationId, Idx childId)
            {
                _children[size_t(relationId.value)] = childId;

                // append new parent at front
                _nextParent[size_t(relationId.value)] = _firstParent[size_t(childId)];
                _firstParent[size_t(childId)] = relationId.value;
            }

            inline void addRelation(Idx parentId, Idx childId, Idx childIndexInParent)
            {
                addRelation(getRelationId(parentId, childIndexInParent), childId);
            }

            inline void removeRelation(RelationId relationIdToRemove)
            {
                Idx childId = _children[size_t(relationIdToRemove.value)];
                _children[size_t(relationIdToRemove.value)] = EmptyIndex;

                RelationId* relationId = &_firstParent[size_t(childId)];

                // find `relationIdToRemove`
                while (relationId->value != EmptyIndex)
                {
                    if (relationId->value == relationIdToRemove.value)
                    {
                        // relation found, replace it with the next one
                        *relationId = _nextParent[size_t(relationId->value)];
                        break;
                    }
                    else
                    {
                        relationId = &_nextParent[size_t(relationId->value)];
                    }
                }
            }

            inline void replaceChild(RelationId relationId, Idx newChildId)
            {
                removeRelation(relationId);
                addRelation(relationId, newChildId);
            }

            inline void removeParent(Idx parentId)
            {
                for (Idx i = 0; i < Degree; ++i)
                {
                    removeRelation(getRelationId(parentId, i));
                }
            }

            inline void clear()
            {
                _parentToChildCapacity = 0;
                _childToParentCapacity = 0;

                _children.clear();
                _firstParent.clear();
                _nextParent.clear();
            }

        private:
            Idx _parentToChildCapacity = Idx(0);
            Idx _childToParentCapacity = Idx(0);

            Collection<Idx, typename Allocator::template StlAllocator<Idx>> _children;
            Collection<RelationId, typename Allocator::template StlAllocator<RelationId>> _firstParent;
            Collection<RelationId, typename Allocator::template StlAllocator<RelationId>> _nextParent;
        };

        template <
            typename Idx,
            typename VertexData,
            typename EdgeData,
            typename TriangleData,
            template <typename, typename> typename Collection,
            typename Allocator
        >
        class TopologyMesh
        {
            // class for creating topology between vertices, edges, and triangles
            // we can get neighbors, iterate over edges of a vertex, etc.
            // we can also store data for each primitive
            // most of the implementation is already done in `FreeList` and `PrimitiveRelations`, this class connects it all together

            static constexpr bool collectionHasReserve = detail::HasReserve<Collection>::value;

        public:
            TopologyMesh(Allocator allocator) :
                _triangleToVertex(allocator),
                _triangleToEdge(allocator),
                _edgeToVertex(allocator),
                _vertices(allocator),
                _edges(allocator),
                _triangles(allocator)
            {
            }

        private:
            static constexpr Idx EmptyIndex = Idx(-1);

        public:
            struct Vertex
            {
                inline explicit Vertex(Idx idx = EmptyIndex) : index(idx)
                {
                }

                inline bool valid() const
                {
                    return index != EmptyIndex;
                }

                Idx index;
            };

            struct Edge
            {
                inline explicit Edge(Idx idx = EmptyIndex) : index(idx)
                {
                }

                inline bool valid() const
                {
                    return index != EmptyIndex;
                }

                Idx index;
            };

            struct Triangle
            {
                inline explicit Triangle(Idx idx = EmptyIndex) : index(idx)
                {
                }

                inline bool valid() const
                {
                    return index != EmptyIndex;
                }

                Idx index;
            };

            struct VertexToEdgeRelation
            {
                inline VertexToEdgeRelation(Idx relationId) : value(relationId)
                {
                }

                inline VertexToEdgeRelation(Edge edge, Idx indexInEdge) : value((edge.index << 1) | indexInEdge)
                {
                }

                inline Edge edge() const
                {
                    return Edge(value >> 1);
                }

                inline Idx indexInEdge() const
                {
                    return value & 1;
                }

                inline bool valid() const
                {
                    return value != EmptyIndex;
                }

                Idx value;
            };

            struct VertexToTriangleRelation
            {
                inline VertexToTriangleRelation(Idx relationId) : value(relationId)
                {
                }

                inline VertexToTriangleRelation(Triangle triangle, Idx indexInTriangle) : value((triangle.index << 2) | indexInTriangle)
                {
                }

                inline Triangle triangle() const
                {
                    return Triangle(value >> 2);
                }

                inline Idx indexInTriangle() const
                {
                    return value & 3;
                }

                inline bool valid() const
                {
                    return value != EmptyIndex;
                }

                Idx value;
            };

            struct EdgeToTriangleRelation
            {
                inline EdgeToTriangleRelation(Idx relationId) : value(relationId)
                {
                }

                inline EdgeToTriangleRelation(Triangle triangle, Idx indexInTriangle) : value((triangle.index << 2) | indexInTriangle)
                {
                }

                inline Triangle triangle() const
                {
                    return Triangle(value >> 2);
                }

                inline Idx indexInTriangle() const
                {
                    return value & 3;
                }

                inline bool valid() const
                {
                    return value != EmptyIndex;
                }

                Idx value;
            };

            inline Vertex getVertexOfTriangle(Triangle triangle, Idx vertexIndexInTriangle) const
            {
                return Vertex(_triangleToVertex.getChildId(triangle.index, vertexIndexInTriangle));
            }

            inline Edge getEdgeOfTriangle(Triangle triangle, Idx edgeIndexInTriangle) const
            {
                return Edge(_triangleToEdge.getChildId(triangle.index, edgeIndexInTriangle));
            }

            inline Vertex getVertexOfEdge(Edge edge, Idx vertexIndexInEdge) const
            {
                return Vertex(_edgeToVertex.getChildId(edge.index, vertexIndexInEdge));
            }

            inline Vertex getOtherVertexOfEdge(VertexToEdgeRelation relation) const
            {
                return Vertex(_edgeToVertex.getChildId(relation.value ^ 1));
            }

            inline Vertex getOppositeVertexInTriangle(EdgeToTriangleRelation relation) const
            {
                return Vertex(_triangleToVertex.getChildId(relation.value));
            }

            inline Edge getOppositeEdgeInTriangle(VertexToTriangleRelation relation) const
            {
                return Edge(_triangleToEdge.getChildId(relation.value));
            }

            inline Idx getVertexIndexInTriangle(Triangle triangle, Vertex vertex) const
            {
                return _triangleToVertex.getChildIndexInParent(triangle.index, vertex.index);
            }

            inline Idx getEdgeIndexInTriangle(Triangle triangle, Edge edge) const
            {
                return _triangleToEdge.getChildIndexInParent(triangle.index, edge.index);
            }

            inline Idx getVertexIndexInEdge(Edge edge, Vertex vertex) const
            {
                return _edgeToVertex.getChildIndexInParent(edge.index, vertex.index);
            }

            inline VertexToEdgeRelation getFirstEdgeOfVertex(Vertex vertex) const
            {
                return _edgeToVertex.getFirstParent(vertex.index).value;
            }

            inline VertexToTriangleRelation getFirstTriangleOfVertex(Vertex vertex) const
            {
                return _triangleToVertex.getFirstParent(vertex.index).value;
            }

            inline EdgeToTriangleRelation getFirstTriangleOfEdge(Edge edge) const
            {
                return _triangleToEdge.getFirstParent(edge.index).value;
            }

            inline VertexToEdgeRelation getNext(VertexToEdgeRelation relation) const
            {
                return _edgeToVertex.getNextParent(relation.value).value;
            }

            inline VertexToTriangleRelation getNext(VertexToTriangleRelation relation) const
            {
                return _triangleToVertex.getNextParent(relation.value).value;
            }

            inline EdgeToTriangleRelation getNext(EdgeToTriangleRelation relation) const
            {
                return _triangleToEdge.getNextParent(relation.value).value;
            }

            inline Idx getVertexUsedCount() const
            {
                return _vertices.getUsedCount();
            }

            inline Idx getEdgeUsedCount() const
            {
                return _edges.getUsedCount();
            }

            inline Idx getTriangleUsedCount() const
            {
                return _triangles.getUsedCount();
            }

            inline bool isVertexValid(const Vertex& v) const
            {
                return _vertices.isValid(v.index);
            }

            inline bool isEdgeValid(const Edge& e) const
            {
                return _edges.isValid(e.index);
            }

            inline bool isTriangleValid(const Triangle& t) const
            {
                return _triangles.isValid(t.index);
            }

            inline void forEachVertex(auto&& callback) const
            {
                _vertices.forEachValid([&](const Idx& id)
                {
                    callback(Vertex(id));
                });
            }

            inline void forEachEdge(auto&& callback) const
            {
                _edges.forEachValid([&](const Idx& id)
                {
                    callback(Edge(id));
                });
            }

            inline void forEachTriangle(auto&& callback) const
            {
                _triangles.forEachValid([&](const Idx& id)
                {
                    callback(Triangle(id));
                });
            }

            inline void forEachEdgeOfVertex(const Vertex& v, auto&& callback) const
            {
                constexpr bool callbackReturnsBool = std::is_same_v<bool, std::invoke_result_t<decltype(callback), VertexToEdgeRelation>>;

                for (VertexToEdgeRelation relation = getFirstEdgeOfVertex(v); relation.valid(); relation = getNext(relation))
                {
                    if constexpr (callbackReturnsBool)
                    {
                        bool shouldContinue = callback(relation);
                        if (!shouldContinue)
                        {
                            break;
                        }
                    }
                    else
                    {
                        callback(relation);
                    }
                }
            }

            inline void forEachTriangleOfVertex(const Vertex& v, auto&& callback) const
            {
                constexpr bool callbackReturnsBool = std::is_same_v<bool, std::invoke_result_t<decltype(callback), VertexToTriangleRelation>>;

                for (VertexToTriangleRelation relation = getFirstTriangleOfVertex(v); relation.valid(); relation = getNext(relation))
                {
                    if constexpr (callbackReturnsBool)
                    {
                        bool shouldContinue = callback(relation);
                        if (!shouldContinue)
                        {
                            break;
                        }
                    }
                    else
                    {
                        callback(relation);
                    }
                }
            }

            inline void forEachTriangleOfEdge(const Edge& e, auto&& callback) const
            {
                constexpr bool callbackReturnsBool = std::is_same_v<bool, std::invoke_result_t<decltype(callback), EdgeToTriangleRelation>>;

                for (EdgeToTriangleRelation relation = getFirstTriangleOfEdge(e); relation.valid(); relation = getNext(relation))
                {
                    if constexpr (callbackReturnsBool)
                    {
                        bool shouldContinue = callback(relation);
                        if (!shouldContinue)
                        {
                            break;
                        }
                    }
                    else
                    {
                        callback(relation);
                    }
                }
            }

            inline const VertexData* getVertexData(const Vertex& vertex) const
            {
                return _vertices.getData(vertex.index);
            }

            inline VertexData* getVertexData(const Vertex& vertex)
            {
                return _vertices.getData(vertex.index);
            }

            inline bool setVertexData(const Vertex& vertex, const VertexData& newData)
            {
                VertexData* data = getVertexData(vertex);
                if (data == nullptr) DETRIA_UNLIKELY
                {
                    return false;
                }

                *data = newData;
                return true;
            }

            inline const EdgeData* getEdgeData(const Edge& edge) const
            {
                return _edges.getData(edge.index);
            }

            inline EdgeData* getEdgeData(const Edge& edge)
            {
                return _edges.getData(edge.index);
            }

            inline bool setEdgeData(const Edge& edge, const EdgeData& newData)
            {
                EdgeData* data = getEdgeData(edge);
                if (data == nullptr) DETRIA_UNLIKELY
                {
                    return false;
                }

                *data = newData;
                return true;
            }

            inline const TriangleData* getTriangleData(const Triangle& triangle) const
            {
                return _triangles.getData(triangle.index);
            }

            inline TriangleData* getTriangleData(const Triangle& triangle)
            {
                return _triangles.getData(triangle.index);
            }

            inline bool setTriangleData(const Triangle& triangle, const TriangleData& newData)
            {
                TriangleData* data = getTriangleData(triangle);
                if (data == nullptr) DETRIA_UNLIKELY
                {
                    return false;
                }

                *data = newData;
                return true;
            }

            inline void reserveVertices(const Idx& capacity)
            {
                _edgeToVertex.reserveParentToChild(capacity);
                _triangleToVertex.reserveParentToChild(capacity);

                if constexpr (collectionHasReserve)
                {
                    _vertices.reserve(capacity);
                }
            }

            inline void reserveEdges(const Idx& capacity)
            {
                _edgeToVertex.reserveChildToParent(capacity);
                _triangleToEdge.reserveParentToChild(capacity);

                if constexpr (collectionHasReserve)
                {
                    _edges.reserve(capacity);
                }
            }

            inline void reserveTriangles(const Idx& capacity)
            {
                _triangleToEdge.reserveChildToParent(capacity);
                _triangleToVertex.reserveChildToParent(capacity);

                if constexpr (collectionHasReserve)
                {
                    _triangles.reserve(capacity);
                }
            }

            inline Vertex createVertex(const VertexData& vertexData = { })
            {
                Vertex v(_vertices.add(vertexData));

                _edgeToVertex.reserveParentToChild(_vertices.getUsedCount());
                _triangleToVertex.reserveParentToChild(_vertices.getUsedCount());

                return v;
            }

            inline void replaceVertexOfEdge(VertexToEdgeRelation relation, Vertex newVertex)
            {
                _edgeToVertex.replaceChild(relation.value, newVertex.index);
            }

            inline void replaceVertexOfTriangle(VertexToTriangleRelation relation, Vertex newVertex)
            {
                _triangleToVertex.replaceChild(relation.value, newVertex.index);
            }

            inline Edge createEdge(const Vertex& v0, const Vertex& v1, const EdgeData& data = { })
            {
                Edge e(_edges.add(data));

                _edgeToVertex.reserveChildToParent(_edges.getUsedCount());
                _triangleToEdge.reserveParentToChild(_edges.getUsedCount());

                _edgeToVertex.addRelation(e.index, v0.index, 0);
                _edgeToVertex.addRelation(e.index, v1.index, 1);

                return e;
            }

            inline Edge getEdgeBetweenVertices(const Vertex& v0, const Vertex& v1) const
            {
                for (VertexToEdgeRelation relation = getFirstEdgeOfVertex(v0); relation.valid(); relation = getNext(relation))
                {
                    if (getOtherVertexOfEdge(relation).index == v1.index)
                    {
                        return relation.edge();
                    }
                }

                return Edge();
            }

            inline Edge getOrCreateEdge(const Vertex& v0, const Vertex& v1)
            {
                Edge edge = getEdgeBetweenVertices(v0, v1);
                if (edge.valid())
                {
                    return edge;
                }

                return createEdge(v0, v1);
            }

            inline void replaceEdgeOfTriangle(EdgeToTriangleRelation relation, Edge newEdge)
            {
                _triangleToEdge.replaceChild(relation.value, newEdge.index);
            }

            inline Triangle createTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, Edge e0, Edge e1, Edge e2, const TriangleData& data = { })
            {
                Triangle t(_triangles.add(data));

                _triangleToEdge.reserveChildToParent(_triangles.getUsedCount());
                _triangleToVertex.reserveChildToParent(_triangles.getUsedCount());

                _triangleToEdge.addRelation(t.index, e0.index, 0);
                _triangleToEdge.addRelation(t.index, e1.index, 1);
                _triangleToEdge.addRelation(t.index, e2.index, 2);

                _triangleToVertex.addRelation(t.index, v0.index, 0);
                _triangleToVertex.addRelation(t.index, v1.index, 1);
                _triangleToVertex.addRelation(t.index, v2.index, 2);

                return t;
            }

            inline Triangle createTriangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, const TriangleData& data = { })
            {
                return createTriangle(v0, v1, v2, getOrCreateEdge(v1, v2), getOrCreateEdge(v2, v0), getOrCreateEdge(v0, v1), data);
            }

            inline bool removeVertex(const Vertex& v)
            {
                return _vertices.remove(v.index);
            }

            inline bool removeEdge(const Edge& e)
            {
                if (!_edges.remove(e.index)) DETRIA_UNLIKELY
                {
                    return false;
                }

                _edgeToVertex.removeParent(e.index);
                return true;
            }

            inline bool removeTriangle(const Triangle& t)
            {
                if (!_triangles.remove(t.index)) DETRIA_UNLIKELY
                {
                    return false;
                }

                _triangleToEdge.removeParent(t.index);
                _triangleToVertex.removeParent(t.index);
                return true;
            }

            inline void clear()
            {
                _triangleToVertex.clear();
                _triangleToEdge.clear();
                _edgeToVertex.clear();
                _vertices.clear();
                _edges.clear();
                _triangles.clear();
            }

        private:
            PrimitiveRelations<Idx, 3, Collection, Allocator> _triangleToVertex;
            PrimitiveRelations<Idx, 3, Collection, Allocator> _triangleToEdge;
            PrimitiveRelations<Idx, 2, Collection, Allocator> _edgeToVertex;
            FreeList<VertexData, Idx, Collection, Allocator> _vertices;
            FreeList<EdgeData, Idx, Collection, Allocator> _edges;
            FreeList<TriangleData, Idx, Collection, Allocator> _triangles;
        };
    }

    enum class TriangleLocation : uint8_t
    {
        // inside an outline
        Interior = 1,

        // part of a hole
        Hole = 2,

        // not part of any outlines or holes, only part of the initial triangulation
        ConvexHull = 4,

        All = Interior | Hole | ConvexHull
    };

    enum class TriangulationError
    {
        // triangulation was successful
        NoError,

        // the triangulation was created, but no triangulation was performed yet
        TriangulationNotStarted,

        // less than three points were added to the triangulation
        LessThanThreePoints,


        // errors of the initial triangulation phase

        // the input points contained a NaN or infinite value
        NonFinitePositionFound,

        // the list of input points contained duplicates
        DuplicatePointsFound,

        // all of the input points were collinear, so no valid triangles could be created
        AllPointsAreCollinear,


        // errors of constrained edge creation

        // a polyline (outline or hole) contained less than 3 points
        PolylineTooShort,

        // an index in a polyline was out-of-bounds (idx < 0 or idx >= number of point)
        PolylineIndexOutOfBounds,

        // two consecutive points in a polyline were the same
        PolylineDuplicateConsecutivePoints,

        // an edge was part of both an outline and a hole
        EdgeWithDifferentConstrainedTypes,

        // a point was exactly on a constrained edge
        PointOnConstrainedEdge,

        // two constrained edges were intersecting
        ConstrainedEdgeIntersection,


        // errors of triangle classification

        // found a hole which was not inside any outlines
        HoleNotInsideOutline,

        // an outline was directly inside another outline, or a hole was directly inside another hole
        StackedPolylines,


        // a condition that should always be true was false, this error indicates a bug in the code
        AssertionFailed
    };

#define DETRIA_ASSERT_MSG(cond, msg) do { if (!detail::detriaAssert((cond), msg)) DETRIA_UNLIKELY { return fail(TE_AssertionFailed{ }); } } while(0)
#define DETRIA_ASSERT(cond) DETRIA_ASSERT_MSG(cond, nullptr)

#ifndef NDEBUG
#define DETRIA_DEBUG_ASSERT(cond) detail::detriaAssert((cond))
#else
#define DETRIA_DEBUG_ASSERT(cond)
#endif

#define DETRIA_CHECK(cond) do { if (!(cond)) DETRIA_UNLIKELY { return false; } } while (0)

    template <typename Point>
    struct DefaultTriangulationConfig
    {
        // Default configuration for a triangulation.
        // To change the values below, you can inherit from this class, and override the things you want to change.
        // For example:
        /*

        struct MyTriangulationConfig : public DefaultTriangulationConfig<detria::PointD>
        {
            using Allocator = MyCustomAllocatorType;

            // the rest are used from the default configuration
        }

        void doStuff()
        {
            detria::Triangulation<detria::PointD, uint32_t, MyTriangulationConfig> tri(getMyCustomAllocator());
            ...
        }

        */

        using PointAdapter = DefaultPointAdapter<Point>;

        using Allocator = memory::DefaultAllocator;

        template <typename T, typename Allocator>
        using Collection = std::vector<T, Allocator>;

        constexpr static bool UseRobustOrientationTests = true;
        constexpr static bool UseRobustIncircleTests = true;

        // If enabled, then all user-provided indices are checked, and if anything is invalid (out-of-bounds or negative indices,
        // or two consecutive duplicate indices), then an error is generated.
        // If disabled, then no checks are done, so if the input has any invalid indices, then it's undefined behavior (might crash).
        constexpr static bool IndexChecks = true;

        // If enabled, then all the input points are checked, and if any of the points have a NaN or infinity value, then an error is generated.
        // If there are no such values in the list of input points, then this can be disabled.
        constexpr static bool NaNChecks = true;
    };

    template <
        typename Point = PointD,
        typename Idx = uint32_t,
        typename Config = DefaultTriangulationConfig<Point>
    >
    class Triangulation
    {
    private:
        using PointAdapter = typename Config::PointAdapter;

        using Allocator = typename Config::Allocator;

        template <typename T, typename Allocator>
        using Collection = typename Config::template Collection<T, Allocator>;

        template <typename T>
        using CollectionWithAllocator = Collection<T, typename Allocator::template StlAllocator<T>>;

        using Vector2 = std::invoke_result_t<decltype(PointAdapter::adapt), const Point&>; // can be const reference

        using Scalar = decltype(std::decay_t<Vector2>::x);
        static_assert(std::is_same_v<Scalar, decltype(std::decay_t<Vector2>::y)>,
            "Adapted point must have `x` and `y` fields, and they must be the same type");

        static_assert(std::is_integral_v<Idx>, "Index type must be an integer type");

        static constexpr bool collectionHasReserve = detail::HasReserve<Collection>::value;

        using Tri = Triangle<Idx>;
        using Edge_ = Edge<Idx>;

        using List = detail::FlatLinkedList<Idx, Idx, Collection, Allocator>;

        enum class EdgeType : uint8_t
        {
            NotConstrained = 0, ManuallyConstrained, AutoDetect, Outline, Hole, MAX_EDGE_TYPE
        };

        struct EdgeData
        {
            // an edge can be either:
            // - not constrained
            // - manually constrained
            // - auto detected: will decide if it's an outline or a hole, depending on where it is
            // - part of an outline or a hole; in this case, we need to store its index
            // also store if the edge is delaunay

            struct NotConstrainedEdgeTag {};
            struct ManuallyConstrainedEdgeTag {};

            struct OutlineOrHoleData
            {
                Idx polylineIndex;
                EdgeType type;
            };

            inline bool isConstrained() const
            {
                return !std::holds_alternative<NotConstrainedEdgeTag>(data);
            }

            inline std::optional<Idx> getOutlineOrHoleIndex() const
            {
                if (const OutlineOrHoleData* holeData = std::get_if<OutlineOrHoleData>(&data))
                {
                    return holeData->polylineIndex;
                }
                else
                {
                    return { };
                }
            }

            inline EdgeType getEdgeType() const
            {
                EdgeType result{ };
                std::visit([&](const auto& value)
                {
                    using Ty = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Ty, NotConstrainedEdgeTag>)
                    {
                        result = EdgeType::NotConstrained;
                    }
                    else if constexpr (std::is_same_v<Ty, ManuallyConstrainedEdgeTag>)
                    {
                        result = EdgeType::ManuallyConstrained;
                    }
                    else
                    {
                        result = value.type;
                    }
                }, data);

                return result;
            }

            std::variant<NotConstrainedEdgeTag, ManuallyConstrainedEdgeTag, OutlineOrHoleData> data;
            bool isDelaunay;
        };

        struct TriangleData
        {
            struct UnknownLocationTag {};

            struct KnownLocationData
            {
                // the index of the inner-most outline or hole that contains this triangle
                // nullopt if the triangle is outside of all polylines
                std::optional<Idx> parentPolylineIndex;
            };

            std::variant<UnknownLocationTag, KnownLocationData> data;
        };

        struct PolylineData
        {
            detail::ReadonlySpan<Idx> pointIndices;
            EdgeType type;
        };

        using TMesh = topology::TopologyMesh<Idx, detail::Empty, EdgeData, TriangleData, Collection, Allocator>;
        using TVertex = typename TMesh::Vertex;
        using TEdge = typename TMesh::Edge;
        using TTriangle = typename TMesh::Triangle;

        using TVertexCollection = CollectionWithAllocator<TVertex>;
        using VERelation = typename TMesh::VertexToEdgeRelation;
        using ETRelation = typename TMesh::EdgeToTriangleRelation;
        using VTRelation = typename TMesh::VertexToTriangleRelation;

        // triangulation error types

        struct TE_NoError
        {
            TriangulationError getError() const
            {
                return TriangulationError::NoError;
            }

            std::string getErrorMessage() const
            {
                return "The triangulation was successful";
            }
        };

        struct TE_NotStarted
        {
            TriangulationError getError() const
            {
                return TriangulationError::TriangulationNotStarted;
            }

            std::string getErrorMessage() const
            {
                return "The triangulation was not performed yet";
            }
        };

        struct TE_LessThanThreePoints
        {
            TriangulationError getError() const
            {
                return TriangulationError::LessThanThreePoints;
            }

            std::string getErrorMessage() const
            {
                return "At least 3 input points are required to perform the triangulation";
            }
        };

        struct TE_NonFinitePositionFound
        {
            TriangulationError getError() const
            {
                return TriangulationError::NonFinitePositionFound;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << (std::isnan(value) ? "NaN" : "Infinite") << " value found at point index " << index;
                return ss.str();
            }

            Idx index;
            Scalar value;
        };

        struct TE_DuplicatePointsFound
        {
            TriangulationError getError() const
            {
                return TriangulationError::DuplicatePointsFound;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "Multiple input points had the same position (" << positionX << ", " << positionY
                    << "), at index " << idx0 << " and at index " << idx1;
                return ss.str();
            }

            Scalar positionX;
            Scalar positionY;
            Idx idx0;
            Idx idx1;
        };

        struct TE_AllPointsAreCollinear
        {
            TriangulationError getError() const
            {
                return TriangulationError::AllPointsAreCollinear;
            }

            std::string getErrorMessage() const
            {
                return "All input points were collinear";
            }
        };

        struct TE_PolylineTooShort
        {
            TriangulationError getError() const
            {
                return TriangulationError::PolylineTooShort;
            }

            std::string getErrorMessage() const
            {
                // note: we don't really have an index to return, since the outlines and the holes are stored in the same vector
                return "An input polyline contained less than three points";
            }
        };

        struct TE_PolylineIndexOutOfBounds
        {
            TriangulationError getError() const
            {
                return TriangulationError::PolylineIndexOutOfBounds;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "A polyline referenced a point at index " << pointIndex << ", which is not in range [0, " << (numPointsInPolyline - 1) << "]";
                return ss.str();
            }

            Idx pointIndex;
            Idx numPointsInPolyline;
        };

        struct TE_PolylineDuplicateConsecutivePoints
        {
            TriangulationError getError() const
            {
                return TriangulationError::PolylineDuplicateConsecutivePoints;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "A polyline containes two duplicate consecutive points (index " << pointIndex << ")";
                return ss.str();
            }

            Idx pointIndex;
        };

        struct TE_EdgeWithDifferentConstrainedTypes
        {
            TriangulationError getError() const
            {
                return TriangulationError::EdgeWithDifferentConstrainedTypes;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "The edge between points " << idx0 << " and " << idx1 << " is part of both an outline and a hole";
                return ss.str();
            }

            Idx idx0;
            Idx idx1;
        };

        struct TE_PointOnConstrainedEdge
        {
            TriangulationError getError() const
            {
                return TriangulationError::PointOnConstrainedEdge;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "Point " << pointIndex << " is exactly on a constrained edge (between points "
                    << edgePointIndex0 << " and " << edgePointIndex1 << ")";
                return ss.str();
            }

            Idx pointIndex;
            Idx edgePointIndex0;
            Idx edgePointIndex1;
        };

        struct TE_ConstrainedEdgeIntersection
        {
            TriangulationError getError() const
            {
                return TriangulationError::ConstrainedEdgeIntersection;
            }

            std::string getErrorMessage() const
            {
                std::stringstream ss;
                ss << "Constrained edges are intersecting: first edge between points " << idx0 << " and " << idx1
                    << ", second edge between points " << idx2 << " and " << idx3;
                return ss.str();
            }

            Idx idx0;
            Idx idx1;
            Idx idx2;
            Idx idx3;
        };

        struct TE_HoleNotInsideOutline
        {
            TriangulationError getError() const
            {
                return TriangulationError::HoleNotInsideOutline;
            }

            std::string getErrorMessage() const
            {
                return "Found a hole which was not inside any outlines";
            }
        };

        struct TE_StackedPolylines
        {
            TriangulationError getError() const
            {
                return TriangulationError::StackedPolylines;
            }

            std::string getErrorMessage() const
            {
                return isHole
                    ? "A hole was directly inside another hole"
                    : "An outline was directly inside another outline";
            }

            bool isHole;
        };

        struct TE_AssertionFailed
        {
            TriangulationError getError() const
            {
                return TriangulationError::AssertionFailed;
            }

            std::string getErrorMessage() const
            {
                return "Internal error, this is a bug in the code";
            }
        };

        using TriangulationErrorData = std::variant<
            TE_NoError,
            TE_NotStarted,
            TE_LessThanThreePoints,
            TE_NonFinitePositionFound,
            TE_DuplicatePointsFound,
            TE_AllPointsAreCollinear,
            TE_PolylineTooShort,
            TE_PolylineIndexOutOfBounds,
            TE_PolylineDuplicateConsecutivePoints,
            TE_EdgeWithDifferentConstrainedTypes,
            TE_PointOnConstrainedEdge,
            TE_ConstrainedEdgeIntersection,
            TE_HoleNotInsideOutline,
            TE_StackedPolylines,
            TE_AssertionFailed
        >;

    public:

        Triangulation(Allocator allocator = { }) :
            _allocator(allocator),
            _points(),
            _polylines(allocator.template createStlAllocator<PolylineData>()),
            _manuallyConstrainedEdges(allocator.template createStlAllocator<Vec2<Idx>>()),
            _autoDetectedPolylineTypes(allocator.template createStlAllocator<EdgeType>()),
            _topologyMesh(allocator),
            _initialTriangulation_SortedPoints(allocator.template createStlAllocator<Idx>()),
            _constrainedEdgeVerticesCW(allocator.template createStlAllocator<TVertex>()),
            _constrainedEdgeVerticesCCW(allocator.template createStlAllocator<TVertex>()),
            _constrainedEdgeReTriangulationStack(allocator.template createStlAllocator<TVertex>()),
            _classifyTriangles_CheckedTriangles(allocator.template createStlAllocator<bool>()),
            _classifyTriangles_TrianglesToCheck(allocator.template createStlAllocator<TTriangle>()),
            _convexHullPoints(allocator),
            _parentPolylines(allocator.template createStlAllocator<std::optional<Idx>>()),
            _delaunayCheckStack(allocator.template createStlAllocator<Edge_>())
        {
        }

        // if the triangulation failed, this function returns the type of the error that occured
        TriangulationError getError() const
        {
            TriangulationError err{ };
            std::visit([&](const auto& errorData)
            {
                err = errorData.getError();
            }, _error);
            return err;
        }

        // returns a human-readable message about the last triangulation error (if any)
        std::string getErrorMessage() const
        {
            std::string msg;
            std::visit([&](const auto& errorData)
            {
                msg = std::move(errorData.getErrorMessage());
            }, _error);
            return msg;
        }

        // clears all data of the triangulation, allowing the triangulation object to be reused
        void clear()
        {
            _points.reset();
            _polylines.clear();
            _manuallyConstrainedEdges.clear();

            clearInternalData();
        }

        // set all points which will be used for the triangulation
        // the points are not copied, so they must be valid for the duration of the triangulation
        // the result triangles will be indices of these points
        // if a point should be part of an outline, then use `addOutline`
        // if a point should be part of a hole, then use `addHole`
        // the rest of the points will be steiner points
        void setPoints(detail::ReadonlySpan<Point> points)
        {
            _points = points;
        }

        // `addOutline`, `addHole`, and `addPolylineAutoDetectType` return the polyline's index,
        // which can be used to get its parent, using `getParentPolylineIndex`

        // add an outline - regions surrounded by outlines are "solid", and will be part of the "inside" triangles
        Idx addOutline(detail::ReadonlySpan<Idx> outline)
        {
            Idx id = Idx(_polylines.size());

            _polylines.push_back(PolylineData
            {
                .pointIndices = outline,
                .type = EdgeType::Outline
            });

            return id;
        }

        // add a hole - holes will be subtracted from the final "solid", and will be part of the "outside" triangles
        Idx addHole(detail::ReadonlySpan<Idx> hole)
        {
            Idx id = Idx(_polylines.size());

            _polylines.push_back(PolylineData
            {
                .pointIndices = hole,
                .type = EdgeType::Hole
            });

            return id;
        }

        // add a polyline, and automatically decide if it's an outline or a hole
        Idx addPolylineAutoDetectType(detail::ReadonlySpan<Idx> polyline)
        {
            Idx id = Idx(_polylines.size());

            _polylines.push_back(PolylineData
            {
                .pointIndices = polyline,
                .type = EdgeType::AutoDetect
            });

            return id;
        }

        // set a single constrained edge, which will be part of the final triangulation
        void setConstrainedEdge(const Idx& idxA, const Idx& idxB)
        {
            _manuallyConstrainedEdges.push_back({ .x = idxA, .y = idxB });
        }

        // perform the triangulation
        // returns true if the triangulation succeeded, false otherwise
        [[nodiscard]]
        bool triangulate(bool delaunay)
        {
            clearInternalData();

            if constexpr (collectionHasReserve)
            {
                // guess capacity, 64 should be a good starting point
                constexpr Idx initialCapacity = 64;
                _constrainedEdgeVerticesCW.reserve(initialCapacity);
                _constrainedEdgeVerticesCCW.reserve(initialCapacity);
                _constrainedEdgeReTriangulationStack.reserve(initialCapacity);

                if (delaunay)
                {
                    _delaunayCheckStack.reserve(initialCapacity);
                }
            }

            if (!triangulateInternal(delaunay))
            {
                // if failed, then clear topology mesh and convex hull points, so no invalid triangulation is returned
                _topologyMesh.clear();
                _convexHullPoints.clear();
                return false;
            }

            _error = TE_NoError{ };
            return true;
        }

        // iterate over every interior triangle of the triangulation
        void forEachTriangle(auto&& callback, bool cwTriangles = true) const
        {
            forEachTriangleInternal(callback, cwTriangles, [&](const TTriangle& tri)
            {
                return getTriangleLocation(tri) == TriangleLocation::Interior;
            });
        }

        // iterate over every hole triangle of the triangulation
        void forEachHoleTriangle(auto&& callback, bool cwTriangles = true) const
        {
            forEachTriangleInternal(callback, cwTriangles, [&](const TTriangle& tri)
            {
                return getTriangleLocation(tri) == TriangleLocation::Hole;
            });
        }

        // iterate over every single triangle of the triangulation, even convex hull triangles
        void forEachTriangleOfEveryLocation(auto&& callback, bool cwTriangles = true) const
        {
            forEachTriangleInternal(callback, cwTriangles, [](const TTriangle&) { return true; });
        }

        // iterate over every triangle of a given location
        // locations can be combined, to iterate over e.g. both interior and hole triangles
        void forEachTriangleOfLocation(auto&& callback, const TriangleLocation& locationMask, bool cwTriangles = true) const
        {
            uint8_t mask = static_cast<uint8_t>(locationMask);

            _topologyMesh.forEachTriangle([&](const TTriangle& tri)
            {
                TriangleLocation location = getTriangleLocation(tri);
                bool shouldProcess = (uint8_t(location) & mask) != 0;
                if (shouldProcess)
                {
                    if (cwTriangles)
                    {
                        callback(getTriangleOriginalIndices<false>(tri), location);
                    }
                    else
                    {
                        callback(getTriangleOriginalIndices<true>(tri), location);
                    }
                }
            });
        }

        // iterate over the vertices (vertex indices) in the convex hull
        // the vertices are in clockwise order
        void forEachConvexHullVertex(auto&& callback)
        {
            if (_convexHullPoints.size() == 0 || _convexHullStartIndex == Idx(-1))
            {
                return;
            }

            Idx nodeId = _convexHullStartIndex;
            do
            {
                const typename List::Node& currentNode = _convexHullPoints.getNode(nodeId);
                callback(currentNode.data);
                nodeId = currentNode.nextId;
            } while (nodeId != _convexHullStartIndex);
        }

        // returns the index of the parent of this polyline (the parent directly contains this polyline)
        // returns nullopt for top-level outlines without parent (and for out-of-range index)
        std::optional<Idx> getParentPolylineIndex(Idx polylineIndex)
        {
            if (polylineIndex >= 0 && size_t(polylineIndex) < _parentPolylines.size())
            {
                return _parentPolylines[size_t(polylineIndex)];
            }

            return { };
        }

    private:
        bool fail(const TriangulationErrorData& errorData)
        {
            _error = errorData;
            return false;
        }

        void clearInternalData()
        {
            _topologyMesh.clear();
            _initialTriangulation_SortedPoints.clear();
            _constrainedEdgeVerticesCW.clear();
            _constrainedEdgeVerticesCCW.clear();
            _constrainedEdgeReTriangulationStack.clear();
            _classifyTriangles_CheckedTriangles.clear();
            _classifyTriangles_TrianglesToCheck.clear();
            _delaunayCheckStack.clear();
            _convexHullPoints.clear();
            _convexHullStartIndex = Idx(-1);
            _parentPolylines.clear();
            _autoDetectedPolylineTypes.clear();
        }

        inline static Vector2 adapt(const Point& p)
        {
            return PointAdapter::adapt(p);
        }

        bool triangulateInternal(bool delaunay)
        {
            if (_points.size() < 3)
            {
                // need at least 3 points to triangulate
                return fail(TE_LessThanThreePoints{ });
            }

            if constexpr (Config::NaNChecks)
            {
                // check NaN / inf values
                for (size_t i = 0; i < _points.size(); ++i)
                {
                    Vector2 p = adapt(_points[i]);
                    if (!std::isfinite(p.x) || !std::isfinite(p.y)) DETRIA_UNLIKELY
                    {
                        return fail(TE_NonFinitePositionFound
                        {
                            .index = Idx(i),
                            .value = std::isfinite(p.x) ? p.y : p.x
                        });
                    }
                }
            }

            // reserve topology mesh capacity
            _topologyMesh.reserveVertices(Idx(_points.size()));
            _topologyMesh.reserveTriangles(Idx(_points.size() * 2)); // guess number of triangles
            _topologyMesh.reserveEdges(Idx(_points.size() * 3)); // number of edges should be around number of vertices + number of triangles

            // only create the vertices for now
            for (size_t i = 0; i < _points.size(); ++i)
            {
                _topologyMesh.createVertex();
            }

            TVertex convexHullVertex0{ };
            TVertex convexHullVertex1{ };

            // initial triangulation, will add the triangles
            DETRIA_CHECK(createInitialTriangulation(delaunay, convexHullVertex0, convexHullVertex1));

            DETRIA_CHECK(createConstrainedEdges(delaunay));

            // go through all triangles, and decide if they are inside or outside
            DETRIA_CHECK(classifyTriangles(_topologyMesh.getEdgeBetweenVertices(convexHullVertex0, convexHullVertex1)));

            return true;
        }

        bool createInitialTriangulation(bool delaunay, TVertex& convexHullVertex0, TVertex& convexHullVertex1)
        {
            // initialize point data
            CollectionWithAllocator<Idx>& sortedPoints = _initialTriangulation_SortedPoints;
            sortedPoints.resize(_points.size());
            for (size_t i = 0; i < _points.size(); ++i)
            {
                sortedPoints[i] = Idx(i);
            }

            // initial triangulation - just a valid triangulation that includes all points

            // sort all points by x coordinates; for points with the same x coordinate, sort by y
            // if there are two points (or more) that have the same x and y coordinate, then we can't triangulate it,
            // because it would create degenerate triangles

            bool hasDuplicatePoints = false;
            Idx duplicatePointIndex0{ };
            Idx duplicatePointIndex1{ };
            std::sort(sortedPoints.begin(), sortedPoints.end(), [&](const Idx& idxA, const Idx& idxB)
            {
                Vector2 a = _points[size_t(idxA)];
                Vector2 b = _points[size_t(idxB)];

                if (a.x != b.x)
                {
                    return a.x < b.x;
                }
                else
                {
                    if (a.y != b.y)
                    {
                        return a.y < b.y;
                    }
                    else
                    {
                        // for some reason, when using emscripten, std::sort sometimes compares the same value with itself
                        // so in that case, don't fail, just return false

                        if (idxA != idxB)
                        {
                            hasDuplicatePoints = true;
                            duplicatePointIndex0 = idxA;
                            duplicatePointIndex1 = idxB;
                        }

                        return false;
                    }
                }
            });

            if (hasDuplicatePoints)
            {
                const Idx& idx0 = sortedPoints[duplicatePointIndex0];
                Vector2 point = adapt(_points[size_t(idx0)]);

                return fail(TE_DuplicatePointsFound
                {
                    .positionX = point.x,
                    .positionY = point.y,
                    .idx0 = duplicatePointIndex0,
                    .idx1 = duplicatePointIndex1
                });
            }

            // find an initial triangle
            // since we have no duplicate points, we can always use the first two points as the triangle's points
            // for the third point, we need to find one so that the first three points are not collinear

            const Idx& p0Idx = sortedPoints[0];
            const Idx& p1Idx = sortedPoints[1];
            Vector2 p0Position = adapt(_points[size_t(p0Idx)]);
            Vector2 p1Position = adapt(_points[size_t(p1Idx)]);

            size_t firstNonCollinearIndex = 0; // index to the list of sorted points, not the original points
            math::Orientation triangleOrientation = math::Orientation::Collinear;
            for (size_t i = 2; i < sortedPoints.size(); ++i)
            {
                const Idx& currentIdx = sortedPoints[i];
                Vector2 currentPosition = adapt(_points[size_t(currentIdx)]);

                triangleOrientation = orient2d(p0Position, p1Position, currentPosition);
                if (triangleOrientation != math::Orientation::Collinear)
                {
                    firstNonCollinearIndex = i;
                    break;
                }
            }

            if (triangleOrientation == math::Orientation::Collinear)
            {
                // all points are collinear, and cannot be triangulated
                return fail(TE_AllPointsAreCollinear{ });
            }

            const Idx& p2Idx = sortedPoints[firstNonCollinearIndex];

            // we have a triangle now, so start adding the remaining points to the triangulation
            // also keep track of the convex hull

            using ListNode = typename List::Node;

            TVertex v0(p0Idx);
            TVertex v1(p1Idx);
            TVertex v2(p2Idx);

            // make sure that the triangles are clockwise
            TTriangle firstTriangle{ };
            if (triangleOrientation == math::Orientation::CW)
            {
                // already cw, don't flip
                firstTriangle = _topologyMesh.createTriangle(v0, v1, v2);
            }
            else
            {
                // ccw, flip to cw
                firstTriangle = _topologyMesh.createTriangle(v1, v0, v2);
            }

            Idx vertexIndex0 = _topologyMesh.getVertexOfTriangle(firstTriangle, 0).index;
            Idx vertexIndex1 = _topologyMesh.getVertexOfTriangle(firstTriangle, 1).index;
            Idx vertexIndex2 = _topologyMesh.getVertexOfTriangle(firstTriangle, 2).index;

            if (delaunay)
            {
                for (Idx i = 0; i < 3; ++i)
                {
                    _topologyMesh.getEdgeData(_topologyMesh.getEdgeOfTriangle(firstTriangle, i))->isDelaunay = true;
                }
            }

            // add indices to convex hull
            Idx firstPointId = _convexHullPoints.create(vertexIndex0);
            Idx secondPointId = _convexHullPoints.addAfter(firstPointId, vertexIndex1);
            Idx lastPointId = _convexHullPoints.addAfter(secondPointId, vertexIndex2);

            auto addPoint = [&](size_t sortedPointIdx)
            {
                const Idx& originalIndex = sortedPoints[sortedPointIdx];
                Vector2 position = adapt(_points[size_t(originalIndex)]);

                auto isEdgeVisible = [&](const Idx& nodeId)
                {
                    // decide if the edge (which is given by the current and the next vertex) is visible from the current point

                    const ListNode& node = _convexHullPoints.getNode(nodeId);
                    const ListNode& next = _convexHullPoints.getNode(node.nextId);

                    math::Orientation orientation = orient2d(
                        adapt(_points[size_t(node.data)]),
                        adapt(_points[size_t(next.data)]),
                        position
                    );

                    // don't consider point as visible if exactly on the line
                    return orientation == math::Orientation::CCW;
                };

                // start checking edges, and find the first and last one that is visible from the current point
                // `lastPointId` is guaranteed to be visible, so it's a good starting point

                Idx lastVisibleForwards = lastPointId;
                Idx lastVisibleBackwards = _convexHullPoints.getNode(lastPointId).prevId;

                // check forwards
                while (true)
                {
                    if (isEdgeVisible(lastVisibleForwards))
                    {
                        lastVisibleForwards = _convexHullPoints.getNode(lastVisibleForwards).nextId;
                    }
                    else
                    {
                        break;
                    }
                }

                // check backwards
                while (true)
                {
                    if (isEdgeVisible(lastVisibleBackwards))
                    {
                        lastVisibleBackwards = _convexHullPoints.getNode(lastVisibleBackwards).prevId;
                    }
                    else
                    {
                        lastVisibleBackwards = _convexHullPoints.getNode(lastVisibleBackwards).nextId;
                        break;
                    }
                }

                DETRIA_ASSERT_MSG(lastVisibleForwards != lastVisibleBackwards, "No visible edges found");

                TVertex pVertex(originalIndex);

                // add new triangles
                // if delaunay, then add edges (that we are about to remove) to the list of edges to check
                Idx current = lastVisibleBackwards;
                TEdge lastAddedEdge{ };

                while (current != lastVisibleForwards)
                {
                    const ListNode& currentNode = _convexHullPoints.getNode(current);

                    TVertex currentVertex(currentNode.data);
                    TVertex nextVertex(_convexHullPoints.getNode(currentNode.nextId).data);

                    TEdge edge0 = _topologyMesh.getEdgeBetweenVertices(nextVertex, currentVertex);
                    TEdge edge1 = lastAddedEdge.valid() ? lastAddedEdge : _topologyMesh.createEdge(currentVertex, pVertex);
                    TEdge edge2 = _topologyMesh.createEdge(pVertex, nextVertex);

                    lastAddedEdge = edge2;
                    TTriangle newTriangle = _topologyMesh.createTriangle(pVertex, nextVertex, currentVertex, edge0, edge1, edge2);

                    if (delaunay)
                    {
                        // make sure the newly added triangle meets the delaunay criteria
                        DETRIA_CHECK(delaunayEdgeFlip(pVertex, _topologyMesh.getEdgeOfTriangle(newTriangle, 0)));
                    }

                    current = currentNode.nextId;
                }

                // remove vertices from convex hull if needed
                current = _convexHullPoints.getNode(lastVisibleBackwards).nextId;
                while (current != lastVisibleForwards)
                {
                    Idx next = _convexHullPoints.getNode(current).nextId;
                    _convexHullPoints.remove(current);
                    current = next;
                }

                // add new vertex to convex hull
                lastPointId = _convexHullPoints.addAfter(lastVisibleBackwards, pVertex.index);

                return true;
            };

            Idx rightMostConvexHullPointAtStart = lastPointId;

            // add points up to `firstNonCollinearIndex`
            for (size_t i = 2; i < firstNonCollinearIndex; ++i)
            {
                DETRIA_CHECK(addPoint(i));
            }

            if (firstNonCollinearIndex != 2)
            {
                // if the first three (or more) points were collinear, then `lastPointId` might not be the right-most point
                // so make sure that it is the right-most

                auto getXCoord = [&](const Idx& id)
                {
                    return adapt(_points[size_t(_convexHullPoints.getNode(id).data)]).x;
                };

                if (getXCoord(lastPointId) < getXCoord(rightMostConvexHullPointAtStart))
                {
                    lastPointId = rightMostConvexHullPointAtStart;
                }
            }

            // skip `firstNonCollinearIndex`, add the rest of the points
            for (size_t i = firstNonCollinearIndex + 1; i < sortedPoints.size(); ++i)
            {
                DETRIA_CHECK(addPoint(i));
            }

            // store an edge of the outline for later
            // also, don't store the edge, because it's possible that an edge is deleted and recreated with a different id later
            // storing two vertices guarantees that the edge will be valid later too
            _convexHullStartIndex = lastPointId;
            const ListNode& firstConvexHullNode = _convexHullPoints.getNode(_convexHullStartIndex);
            const ListNode& secondConvexHullNode = _convexHullPoints.getNode(firstConvexHullNode.nextId);
            convexHullVertex0 = TVertex(firstConvexHullNode.data);
            convexHullVertex1 = TVertex(secondConvexHullNode.data);

            return true;
        }

        // use `_delaunayCheckStack` in this function
        // it will always be empty when this function returns
        bool delaunayEdgeFlip(const TVertex& justAddedVertex, const TEdge& oppositeEdge)
        {
            auto addEdgeToCheck = [&](const TEdge& edge)
            {
                _topologyMesh.getEdgeData(edge)->isDelaunay = false;
                _delaunayCheckStack.emplace_back(TopologyEdgeWithVertices
                {
                    .v0 = _topologyMesh.getVertexOfEdge(edge, 0),
                    .v1 = _topologyMesh.getVertexOfEdge(edge, 1),
                    .edge = edge
                });
            };

            addEdgeToCheck(oppositeEdge);

            // flip edges
            while (!_delaunayCheckStack.empty())
            {
                TopologyEdgeWithVertices edgeWithVertices = _delaunayCheckStack.back();
                _delaunayCheckStack.pop_back();

                TEdge edge01{ };
                // check if the edge still has the same vertices
                if (_topologyMesh.getVertexOfEdge(edgeWithVertices.edge, 0).index == edgeWithVertices.v0.index &&
                    _topologyMesh.getVertexOfEdge(edgeWithVertices.edge, 1).index == edgeWithVertices.v1.index) DETRIA_LIKELY
                {
                    // edge still exists
                    edge01 = edgeWithVertices.edge;
                }
                else
                {
                    // edge was flipped
                    continue;
                }

                EdgeData* edgeData = _topologyMesh.getEdgeData(edge01);
                DETRIA_ASSERT(edgeData != nullptr);
                if (edgeData->isDelaunay || edgeData->isConstrained())
                {
                    // don't flip edges that are already delaunay or constrained
                    continue;
                }

                // get first and second triangle of the edge
                // it's possible that we only have one triangle

                ETRelation relation0 = _topologyMesh.getFirstTriangleOfEdge(edge01);
                DETRIA_ASSERT(relation0.valid()); // must have at least one triangle

                ETRelation relation1 = _topologyMesh.getNext(relation0);
                if (!relation1.valid())
                {
                    // no triangle on the other side, so definitely cannot flip
                    edgeData->isDelaunay = true;
                    continue;
                }

                DETRIA_DEBUG_ASSERT(!_topologyMesh.getNext(relation1).valid()); // there should be no more than two triangles

                TTriangle tri0 = relation0.triangle();
                TTriangle tri1 = relation1.triangle();

                Idx indexOfEdgeInTri0 = relation0.indexInTriangle();
                Idx indexOfEdgeInTri1 = relation1.indexInTriangle();

                // get original vertices again, so that they are in the correct orientation
                TVertex vertex0 = _topologyMesh.getVertexOfTriangle(tri0, topology::next3(indexOfEdgeInTri0));
                TVertex vertex1 = _topologyMesh.getVertexOfTriangle(tri0, topology::prev3(indexOfEdgeInTri0));

                TVertex otherVertex0 = _topologyMesh.getVertexOfTriangle(tri0, indexOfEdgeInTri0);
                TVertex otherVertex1 = _topologyMesh.getVertexOfTriangle(tri1, indexOfEdgeInTri1);

                /*
                
         vertex0     otherVertex1
                *---*
                |\  |
                | \ |
                |  \|
                *---*
    otherVertex0     vertex1
                
                */

                // points of the current edge
                Vector2 vertex0Position = adapt(_points[size_t(vertex0.index)]);
                Vector2 vertex1Position = adapt(_points[size_t(vertex1.index)]);

                // points of the edge that we'd get if a flip is needed
                Vector2 otherVertex0Position = adapt(_points[size_t(otherVertex0.index)]);
                Vector2 otherVertex1Position = adapt(_points[size_t(otherVertex1.index)]);

                // TODO?: maybe we could allow user-defined functions to decide if an edge should be flipped
                // that would enable other metrics, e.g. minimize edge length, flip based on the aspect ratio of the triangles, etc.
                // https://people.eecs.berkeley.edu/~jrs/papers/elemj.pdf
                // but we'd need to make sure that every edge is only processed once

                math::CircleLocation loc = incircle(vertex0Position, vertex1Position, otherVertex1Position, otherVertex0Position);
                if (loc == math::CircleLocation::Inside)
                {
                    // flip edge
                    // note that the edge is always flippable if we get here, no need to do orientation checks

                    Idx indexOfPrevEdgeInTri0 = topology::prev3(indexOfEdgeInTri0);
                    Idx indexOfPrevEdgeInTri1 = topology::prev3(indexOfEdgeInTri1);
                    Idx indexOfNextEdgeInTri0 = topology::next3(indexOfEdgeInTri0);
                    Idx indexOfNextEdgeInTri1 = topology::next3(indexOfEdgeInTri1);

                    TEdge edge0 = _topologyMesh.getEdgeOfTriangle(tri0, indexOfPrevEdgeInTri0);
                    TEdge edge1 = _topologyMesh.getEdgeOfTriangle(tri0, indexOfNextEdgeInTri0);
                    TEdge edge2 = _topologyMesh.getEdgeOfTriangle(tri1, indexOfPrevEdgeInTri1);
                    TEdge edge3 = _topologyMesh.getEdgeOfTriangle(tri1, indexOfNextEdgeInTri1);

                    // update relations
                    // edge vertices
                    _topologyMesh.replaceVertexOfEdge(VERelation(edge01, 1), otherVertex0);
                    _topologyMesh.replaceVertexOfEdge(VERelation(edge01, 0), otherVertex1);

                    // triangle0
                    _topologyMesh.replaceVertexOfTriangle(VTRelation(tri0, indexOfPrevEdgeInTri0), otherVertex1);
                    _topologyMesh.replaceEdgeOfTriangle(ETRelation(tri0, indexOfEdgeInTri0), edge3);
                    _topologyMesh.replaceEdgeOfTriangle(ETRelation(tri0, indexOfNextEdgeInTri0), edge01);

                    // triangle1
                    _topologyMesh.replaceVertexOfTriangle(VTRelation(tri1, indexOfPrevEdgeInTri1), otherVertex0);
                    _topologyMesh.replaceEdgeOfTriangle(ETRelation(tri1, indexOfEdgeInTri1), edge1);
                    _topologyMesh.replaceEdgeOfTriangle(ETRelation(tri1, indexOfNextEdgeInTri1), edge01);

                    // only check edges which can be non-delaunay
                    if (justAddedVertex.index == otherVertex0.index)
                    {
                        addEdgeToCheck(edge2);
                        addEdgeToCheck(edge3);
                    }
                    else
                    {
                        DETRIA_DEBUG_ASSERT(justAddedVertex.index == otherVertex1.index);
                        addEdgeToCheck(edge0);
                        addEdgeToCheck(edge1);
                    }
                }

                edgeData->isDelaunay = true;
            }

            return true;
        }

        bool createConstrainedEdges(bool delaunay)
        {
            // ensure that an edge exists between each consecutive vertex for all outlines and holes

            for (size_t i = 0; i < _polylines.size(); ++i)
            {
                const PolylineData& polylineData = _polylines[i];

                detail::ReadonlySpan<Idx> polyline = polylineData.pointIndices;
                EdgeType constrainedEdgeType = polylineData.type;

                if (polyline.size() < 3) DETRIA_UNLIKELY
                {
                    return fail(TE_PolylineTooShort{ });
                }

                Idx prevVertexIdx{ };
                size_t startingIndex{ };

                // we allow polylines to have the same start and end vertex, e.g. [0, 1, 2, 3, 0]

                if (polyline.front() == polyline.back())
                {
                    if (polyline.size() < 4)
                    {
                        // [0, 1, 0] is invalid, even though it has 3 elements
                        return fail(TE_PolylineTooShort{ });
                    }

                    prevVertexIdx = polyline.front();
                    startingIndex = 1;
                }
                else
                {
                    prevVertexIdx = polyline.back();
                    startingIndex = 0;
                }

                for (size_t j = startingIndex; j < polyline.size(); ++j)
                {
                    const Idx& currentIdx = polyline[j];

                    DETRIA_CHECK(constrainSingleEdge(prevVertexIdx, currentIdx, constrainedEdgeType, Idx(i), delaunay));

                    prevVertexIdx = currentIdx;
                }
            }

            for (const Vec2<Idx>& edge : _manuallyConstrainedEdges)
            {
                DETRIA_CHECK(constrainSingleEdge(edge.x, edge.y, EdgeType::ManuallyConstrained, Idx(-1), delaunay));
            }

            return true;
        }

        static constexpr std::array<uint8_t, size_t(EdgeType::MAX_EDGE_TYPE)> getConstrainedEdgeTypePriorities()
        {
            std::array<uint8_t, size_t(EdgeType::MAX_EDGE_TYPE)> priorities{ };
            priorities[size_t(EdgeType::NotConstrained)] = 0;
            priorities[size_t(EdgeType::ManuallyConstrained)] = 1;
            priorities[size_t(EdgeType::AutoDetect)] = 2;
            priorities[size_t(EdgeType::Outline)] = 3;
            priorities[size_t(EdgeType::Hole)] = 3;

            return priorities;
        }

        bool constrainSingleEdge(const Idx& idxA, const Idx& idxB, const EdgeType& constrainedEdgeType, const Idx& polylineIndex, bool delaunay)
        {
            if constexpr (Config::IndexChecks)
            {
                if (idxA < Idx(0) || idxA >= Idx(_points.size()) || idxB < Idx(0) || idxB >= Idx(_points.size())) DETRIA_UNLIKELY
                {
                    return fail(TE_PolylineIndexOutOfBounds
                    {
                        .pointIndex = (idxA < Idx(0) || idxA >= Idx(_points.size())) ? idxA : idxB,
                        .numPointsInPolyline = Idx(_points.size())
                    });
                }

                if (idxA == idxB) DETRIA_UNLIKELY
                {
                    return fail(TE_PolylineDuplicateConsecutivePoints
                    {
                        .pointIndex = idxA
                    });
                }
            }

            TVertex v0(idxA);
            TVertex v1(idxB);

            TEdge currentEdge = _topologyMesh.getEdgeBetweenVertices(v0, v1);
            if (currentEdge.valid())
            {
                // there is already an edge between the vertices, which may or may not be constrained already
                // we have priorities to decide which edge types are overwritten by other types
                // if an edge is not yet constrained, then always overwrite it
                // if an edge is manually constrained, then only overwrite it if the new type is auto-detect, outline, or hole
                // if an edge is auto-detect, then only overwrite it with outline or hole
                // if an edge is already an outline or hole, then check if the new type is the same
                // if they are different, then it's an error

                if (EdgeData* currentEdgeData = _topologyMesh.getEdgeData(currentEdge)) DETRIA_LIKELY
                {
                    constexpr std::array<uint8_t, size_t(EdgeType::MAX_EDGE_TYPE)> edgeTypePriorities = getConstrainedEdgeTypePriorities();

                    EdgeType currentEdgeType = currentEdgeData->getEdgeType();
                    uint8_t currentPriority = edgeTypePriorities[size_t(currentEdgeType)];
                    uint8_t newPriority = edgeTypePriorities[size_t(constrainedEdgeType)];

                    if (newPriority > currentPriority)
                    {
                        // new edge type is higher priority, overwrite

                        switch (constrainedEdgeType)
                        {
                            case EdgeType::ManuallyConstrained:
                                currentEdgeData->data = typename EdgeData::ManuallyConstrainedEdgeTag{ };
                                break;
                            case EdgeType::AutoDetect:
                            case EdgeType::Outline:
                            case EdgeType::Hole:
                                currentEdgeData->data = typename EdgeData::OutlineOrHoleData
                                {
                                    .polylineIndex = polylineIndex,
                                    .type = constrainedEdgeType
                                };

                                break;
                            DETRIA_UNLIKELY default:
                                DETRIA_ASSERT(false);
                                break;
                        }
                    }
                    else if (newPriority == currentPriority)
                    {
                        // same priority, so the types must also be the same

                        if (currentEdgeType != constrainedEdgeType) DETRIA_UNLIKELY
                        {
                            // cannot have an edge that is both an outline and a hole
                            return fail(TE_EdgeWithDifferentConstrainedTypes
                            {
                                .idx0 = idxA,
                                .idx1 = idxB
                            });
                        }
                    }
                    // else lower priority, don't do anything
                }
                else
                {
                    DETRIA_ASSERT(false);
                }

                return true;
            }

            /*
            example:

                 a         b
                 *---------*
                /|        /|\
               / |       / | \
              /  |      /  |  \
             /   |     /   |   \
         v0 *----|----/----|----* v1
             \   |   /     |   /
              \  |  /      |  /
               \ | /       | /
                \|/        |/
                 *---------*
                 c         d

            we want to create a constrained edge between v0 and v1
            start from v0
            since triangles are cw, there must be a triangle which contains v0, and has points pNext and pPrev,
            for which orient2d(v0, v1, pNext) == CCW and orient2d(v0, v1, pPrev) == CW
            in the current example, the starting triangle is "v0 a c"
            the next triangle is the other triangle of edge "a c"
            check the third point of that next triangle (b), whether orient2d(v0, v1, b) is CW or CCW
            if CW, then the next edge is "a b", if CCW, then "c b"
            repeat this until we find v1 in one of the triangles
            also keep track of the outer vertices for both the CW and CCW side
            (in this example, the CW side is "v0 c d v1", CCW side is "v0 a b v1")
            then, remove all in-between triangles, and re-triangulate the CW and CCW sides separately
            this will create an edge between v0 and v1
            */

            const Point& p0 = _points[size_t(v0.index)];
            const Point& p1 = _points[size_t(v1.index)];

            TTriangle initialTriangle{ };
            TVertex vertexCW{ };
            TVertex vertexCCW{ };

            // find initial triangle, which is in the direction of the other point
            DETRIA_CHECK(findInitialTriangleForConstrainedEdge(v0, v1, p0, p1, initialTriangle, vertexCW, vertexCCW));

            _constrainedEdgeVerticesCW.clear();
            _constrainedEdgeVerticesCCW.clear();

            // traverse adjacent triangles, until we reach v1
            // store vertex indices of the triangles along the way
            DETRIA_CHECK(removeInnerTrianglesAndGetOuterVertices(v0, v1, p0, p1, vertexCW, vertexCCW, initialTriangle,
                _constrainedEdgeVerticesCW, _constrainedEdgeVerticesCCW));

            // create new edge, and mark it constrained
            TEdge constrainedEdge = _topologyMesh.createEdge(v0, v1);
            if (constrainedEdgeType == EdgeType::ManuallyConstrained)
            {
                _topologyMesh.setEdgeData(constrainedEdge, EdgeData
                {
                    .data = typename EdgeData::ManuallyConstrainedEdgeTag{ },
                    .isDelaunay = false
                });
            }
            else
            {
                _topologyMesh.setEdgeData(constrainedEdge, EdgeData
                {
                    .data = typename EdgeData::OutlineOrHoleData
                    {
                        .polylineIndex = polylineIndex,
                        .type = constrainedEdgeType
                    },
                    .isDelaunay = false
                });
            }

            // re-triangulate deleted triangles

            DETRIA_CHECK(
                reTriangulateAroundConstrainedEdge<true>(_constrainedEdgeVerticesCW, delaunay) &&
                reTriangulateAroundConstrainedEdge<false>(_constrainedEdgeVerticesCCW, delaunay)
            );

            return true;
        }

        bool findInitialTriangleForConstrainedEdge(const TVertex& v0, const TVertex& v1, const Point& p0, const Point& p1,
            TTriangle& initialTriangle, TVertex& vertexCW, TVertex& vertexCCW)
        {
            bool found = false;
            bool error = false;

            _topologyMesh.forEachTriangleOfVertex(v0, [&](const VTRelation& relation)
            {
                TTriangle tri = relation.triangle();
                Idx indexInTriangle = _topologyMesh.getVertexIndexInTriangle(tri, v0);
                Idx prevIndexInTriangle = topology::prev3(indexInTriangle);
                Idx nextIndexInTriangle = topology::next3(indexInTriangle);
                TVertex prevVertex = _topologyMesh.getVertexOfTriangle(tri, prevIndexInTriangle);
                TVertex nextVertex = _topologyMesh.getVertexOfTriangle(tri, nextIndexInTriangle);

                // we are looking for a triangle where "v0, v1, nextVertex" is ccw, and "v0, v1, prevVertex" is cw

                Vector2 p0Position = adapt(p0);
                Vector2 p1Position = adapt(p1);
                Vector2 prevVertexPosition = adapt(_points[size_t(prevVertex.index)]);
                Vector2 nextVertexPosition = adapt(_points[size_t(nextVertex.index)]);

                math::Orientation orientNext = orient2d(p0Position, p1Position, nextVertexPosition);
                math::Orientation orientPrev = orient2d(p0Position, p1Position, prevVertexPosition);

                if (orientPrev == math::Orientation::Collinear || orientNext == math::Orientation::Collinear) DETRIA_UNLIKELY
                {
                    // we found a point which is exactly on the line
                    // check if it's between v0 and v1
                    // if yes, then this means that an edge would have to go 3 points, which would create degenerate triangles
                    // if this is a steiner point, then maybe we could remove it, but for now, this is an error
                    // if it's not between, then simply skip this triangle

                    auto isBetween = [&](Vector2 point)
                    {
                        Scalar xDiff = predicates::Absolute(p0Position.x - p1Position.x);
                        Scalar yDiff = predicates::Absolute(p0Position.y - p1Position.y);

                        Scalar p0Value{ };
                        Scalar p1Value{ };
                        Scalar pointValue{ };
                        if (xDiff > yDiff)
                        {
                            // compare x coordinate
                            p0Value = p0Position.x;
                            p1Value = p1Position.x;
                            pointValue = point.x;
                        }
                        else
                        {
                            // compare y coordinate
                            p0Value = p0Position.y;
                            p1Value = p1Position.y;
                            pointValue = point.y;
                        }

                        if (p0Value < p1Value)
                        {
                            return p0Value <= pointValue && pointValue <= p1Value;
                        }
                        else
                        {
                            return p1Value <= pointValue && pointValue <= p0Value;
                        }
                    };

                    if (orientPrev == math::Orientation::Collinear)
                    {
                        error = isBetween(prevVertexPosition);
                    }
                    else
                    {
                        error = isBetween(nextVertexPosition);
                    }

                    if (error)
                    {
                        found = true;
                        fail(TE_PointOnConstrainedEdge
                        {
                            .pointIndex = orientPrev == math::Orientation::Collinear ? prevVertex.index : nextVertex.index,
                            .edgePointIndex0 = v0.index,
                            .edgePointIndex1 = v1.index
                        });

                        return false;
                    }

                    return true;
                }
                else if (orientPrev == math::Orientation::CW && orientNext == math::Orientation::CCW)
                {
                    // correct orientation, triangle found
                    found = true;

                    initialTriangle = tri;
                    vertexCW = prevVertex;
                    vertexCCW = nextVertex;

                    return false;
                }

                return true;
            });

            DETRIA_ASSERT(found);

            return !error;
        }

        bool removeInnerTrianglesAndGetOuterVertices(const TVertex& v0, const TVertex& v1, const Point& p0, const Point& p1,
            TVertex vertexCW, TVertex vertexCCW, const TTriangle& initialTriangle,
            CollectionWithAllocator<TVertex>& verticesCW, CollectionWithAllocator<TVertex>& verticesCCW)
        {
            verticesCW.push_back(v0);
            verticesCW.push_back(vertexCW);
            verticesCCW.push_back(v0);
            verticesCCW.push_back(vertexCCW);

            TTriangle prevTriangle = initialTriangle;

            while (true)
            {
                TEdge edge = _topologyMesh.getEdgeBetweenVertices(vertexCW, vertexCCW);
                DETRIA_DEBUG_ASSERT(edge.valid());

                EdgeType edgeType = _topologyMesh.getEdgeData(edge)->getEdgeType();

                if (edgeType != EdgeType::NotConstrained) DETRIA_UNLIKELY
                {
                    // to constrain the current edge, we'd have to remove another edge that is already constrained
                    // this probably means that some outlines or holes are intersecting
                    return fail(TE_ConstrainedEdgeIntersection
                    {
                        .idx0 = v0.index,
                        .idx1 = v1.index,
                        .idx2 = vertexCW.index,
                        .idx3 = vertexCCW.index
                    });
                }

                // remove triangle, since it will be re-triangulated
                _topologyMesh.removeTriangle(prevTriangle);
                _topologyMesh.removeEdge(edge);

                // the edge should only have one triangle now
                TTriangle nextTriangle = _topologyMesh.getFirstTriangleOfEdge(edge).triangle();
                DETRIA_DEBUG_ASSERT(nextTriangle.valid() && nextTriangle.index != prevTriangle.index);

                prevTriangle = nextTriangle;

                Idx nextIndexInTriangleCW = _topologyMesh.getVertexIndexInTriangle(nextTriangle, vertexCW);
                Idx nextIndexInTriangleCCW = _topologyMesh.getVertexIndexInTriangle(nextTriangle, vertexCCW);

                Idx thirdVertexIndexInTriangle = topology::other3(nextIndexInTriangleCW, nextIndexInTriangleCCW);
                TVertex thirdVertex = _topologyMesh.getVertexOfTriangle(nextTriangle, thirdVertexIndexInTriangle);
                if (thirdVertex.index == v1.index)
                {
                    // reached v1
                    break;
                }

                math::Orientation orientation = orient2d(adapt(p0), adapt(p1), adapt(_points[size_t(thirdVertex.index)]));
                if (orientation == math::Orientation::Collinear) DETRIA_UNLIKELY
                {
                    // point on a constrained edge, this is not allowed
                    return fail(TE_PointOnConstrainedEdge
                    {
                        .pointIndex = thirdVertex.index,
                        .edgePointIndex0 = v0.index,
                        .edgePointIndex1 = v1.index
                    });
                }

                if (orientation == math::Orientation::CW)
                {
                    vertexCW = thirdVertex;
                    verticesCW.push_back(thirdVertex);
                }
                else
                {
                    vertexCCW = thirdVertex;
                    verticesCCW.push_back(thirdVertex);
                }
            }

            // remove last triangle (which has v1)
            _topologyMesh.removeTriangle(prevTriangle);

            verticesCW.push_back(v1);
            verticesCCW.push_back(v1);

            DETRIA_ASSERT(verticesCW.size() >= 3 && verticesCCW.size() >= 3);

            return true;
        }

        template <bool IsCW>
        bool reTriangulateAroundConstrainedEdge(const CollectionWithAllocator<TVertex>& vertices, bool delaunay)
        {
            /*
            `requiredOrientation` depends on which side of the line we are on
            for example, let's say we have to re-triangulate this:
            (note that there is no actual edge between v0 and v1 at this point, it will be implicitly added with the re-triangulation)

              v0    a     b
                *---*---*
               / \     /
            c *   \   /
              |    \ /
              *-----*
             d      v1

            we want to triangulate "v0 a b v1" and "v0 c d v1"
            the first polyline is CCW, the second is CW, because all points are on that side of the "v0 v1" line
            when triangulating the first polyline, we can only add a triangle if the vertices are CW
            so for example, we cannot add the "v0 a b" triangle, because the points are not CW, they are collinear; but we can add "a b v1"
            similarly, for the second polyline, we can only add a triangle if the points are CCW, so we can add "v0 c d"
            so:
                CW side of the "v0 v1" line -> `requiredOrientation` == CCW
                CCW side -> `requiredOrientation` == CW
            */

            constexpr math::Orientation requiredOrientation = IsCW ? math::Orientation::CCW : math::Orientation::CW;

            _constrainedEdgeReTriangulationStack.clear();

            _constrainedEdgeReTriangulationStack.push_back(vertices[0]);
            _constrainedEdgeReTriangulationStack.push_back(vertices[1]);

            for (size_t i = 2; i < vertices.size(); ++i)
            {
                const TVertex& currentVertex = vertices[i];

                while (true)
                {
                    TVertex prevPrevVertex = _constrainedEdgeReTriangulationStack[_constrainedEdgeReTriangulationStack.size() - 2];
                    TVertex prevVertex = _constrainedEdgeReTriangulationStack[_constrainedEdgeReTriangulationStack.size() - 1];

                    math::Orientation orientation = orient2d(
                        adapt(_points[size_t(prevPrevVertex.index)]),
                        adapt(_points[size_t(prevVertex.index)]),
                        adapt(_points[size_t(currentVertex.index)])
                    );

                    if (orientation == requiredOrientation)
                    {
                        // add triangle
                        TTriangle tri{ };
                        if constexpr (IsCW)
                        {
                            tri = _topologyMesh.createTriangle(currentVertex, prevVertex, prevPrevVertex);
                        }
                        else
                        {
                            tri = _topologyMesh.createTriangle(currentVertex, prevPrevVertex, prevVertex);
                        }

                        // update stack
                        _constrainedEdgeReTriangulationStack.pop_back();

                        if (delaunay)
                        {
                            // ensure delaunay criteria for the newly inserted triangle
                            // usually, the triangles are already delaunay, but sometimes not, so check it here

                            DETRIA_CHECK(delaunayEdgeFlip(currentVertex, _topologyMesh.getEdgeOfTriangle(tri, 0)));
                        }

                        // if there are more than one items left in the stack, then go back and try to triangulate again
                        // otherwise just break
                        if (_constrainedEdgeReTriangulationStack.size() < 2)
                        {
                            DETRIA_ASSERT(_constrainedEdgeReTriangulationStack.size() == 1);
                            break;
                        }
                    }
                    else
                    {
                        // cannot add triangle, will try again later, after adding a new vertex
                        break;
                    }
                }

                _constrainedEdgeReTriangulationStack.push_back(currentVertex);
            }

            DETRIA_ASSERT(_constrainedEdgeReTriangulationStack.size() == 2);
            return true;
        }

        bool classifyTriangles(const TEdge& startingConvexHullEdge)
        {
            DETRIA_ASSERT(startingConvexHullEdge.valid());

            EdgeData startingEdgeData = *_topologyMesh.getEdgeData(startingConvexHullEdge);
            EdgeType startingEdgeType = startingEdgeData.getEdgeType();
            if (startingEdgeType == EdgeType::Hole)
            {
                // if this happens, then it means that an outer polyline is a hole, which is invalid
                return fail(TE_HoleNotInsideOutline{ });
            }

            // outer edge, only has one triangle
            TTriangle startingTriangle = _topologyMesh.getFirstTriangleOfEdge(startingConvexHullEdge).triangle();
            _topologyMesh.setTriangleData(startingTriangle, TriangleData
            {
                .data = typename TriangleData::KnownLocationData
                {
                    // nullopt if the edge is not part of an outline or a hole
                    .parentPolylineIndex = startingEdgeData.getOutlineOrHoleIndex()
                }
            });

            // for each polyline, store which other polyline contains them
            _parentPolylines.resize(_polylines.size());

            _autoDetectedPolylineTypes.resize(_polylines.size(), EdgeType::AutoDetect);

            CollectionWithAllocator<bool>& checkedTriangles = _classifyTriangles_CheckedTriangles;
            checkedTriangles.resize(size_t(_topologyMesh.getTriangleUsedCount()));
            checkedTriangles[size_t(startingTriangle.index)] = true;

            CollectionWithAllocator<TTriangle>& trianglesToCheck = _classifyTriangles_TrianglesToCheck;
            trianglesToCheck.push_back(startingTriangle);

            if (startingEdgeType == EdgeType::AutoDetect)
            {
                // convex hull edge is auto-detect, so it must be an outline
                if (std::optional<Idx> startingEdgePolylineIndex = startingEdgeData.getOutlineOrHoleIndex())
                {
                    _autoDetectedPolylineTypes[size_t(*startingEdgePolylineIndex)] = EdgeType::Outline;
                }
                else
                {
                    DETRIA_ASSERT(false);
                }
            }

            while (!trianglesToCheck.empty())
            {
                TTriangle currentTriangle = trianglesToCheck.back();
                trianglesToCheck.pop_back();

                const typename TriangleData::KnownLocationData* currentTriangleData =
                    std::get_if<typename TriangleData::KnownLocationData>(&_topologyMesh.getTriangleData(currentTriangle)->data);

                DETRIA_ASSERT(currentTriangleData != nullptr);

                bool currentTriangleIsInterior = getTriangleLocation(*currentTriangleData) == TriangleLocation::Interior;

                for (Idx i = 0; i < 3; ++i)
                {
                    TEdge edge = _topologyMesh.getEdgeOfTriangle(currentTriangle, i);

                    TTriangle neighborTriangle = getOtherTriangleOfEdge(currentTriangle, edge);
                    if (!neighborTriangle.valid())
                    {
                        // the triangle is on the edge of the entire triangulation, so it does not have all 3 neighbors
                        continue;
                    }

                    if (checkedTriangles[size_t(neighborTriangle.index)])
                    {
                        // this triangle was already checked, don't check it again
                        continue;
                    }

                    checkedTriangles[size_t(neighborTriangle.index)] = true;

                    std::optional<Idx> neighborTriangleParentPolylineIndex{ };

                    const EdgeData& edgeData = *_topologyMesh.getEdgeData(edge);
                    if (std::optional<Idx> polylineIndex = edgeData.getOutlineOrHoleIndex())
                    {
                        // check which type of edge we just crossed (if this branch is entered, then it's part of either an outline or a hole)
                        // there are a few possibilites:
                        // if the triangle is inside, and the edge is part of an outline,
                        // then the outline's index must be the same as the triangle's parent polyline index
                        // otherwise it would mean that there is an outline inside another outline, which is not allowed
                        // same for holes: if the triangle is outside, and the edge is part of a hole,
                        // then it must be the same hole as the current triangle's parent
                        // outlines can have any number of holes in them, and holes can have any number of outlines too

                        /*
                        for example:
                        let's say that we have two polylines: "a" and "b"

                          *---*    *---*
                         a|   |   b|   |
                          *---*    *---*

                        let's also say that `startingTriangle` (declared near the start of this function) is not part of any of the polylines
                        so the classification starts from outside
                        "a" and "b" both must be outlines, otherwise we'd have a hole in an area that is already outside, which is invalid
                        to verify this, whenever we reach "a" or "b", check if the edge part of an outline or a hole
                        if it's an outline, then it's fine, because "outside triangle" -> "outline edge" -> "inside triangle" is valid
                        if it's a hole, then check if the hole's index is the same as the triangle's parent polyline index
                        since the current triangle is not inside any polyline, its parent polyline index is nullopt, so the indices are not the same
                        so if either "a" or "b" is a hole, we can detect it, and fail the triangulation
                        */

                        EdgeType currentPolylineType = _polylines[size_t(*polylineIndex)].type;
                        if (currentPolylineType == EdgeType::AutoDetect)
                        {
                            // current type is auto-detect, check if we already already know the actual type
                            EdgeType& autoDetectedType = _autoDetectedPolylineTypes[size_t(*polylineIndex)];
                            if (autoDetectedType == EdgeType::AutoDetect)
                            {
                                // type is not known yet, so just set to the opposite
                                // so if we are outside, then classify as outline
                                // if we are inside, then classify as hole
                                autoDetectedType = currentTriangleIsInterior ? EdgeType::Hole : EdgeType::Outline;
                            }
                            else
                            {
                                // already classified
                            }

                            currentPolylineType = autoDetectedType;
                        }

                        bool currentEdgeIsHole = currentPolylineType == EdgeType::Hole;
                        if (currentTriangleIsInterior == currentEdgeIsHole)
                        {
                            // simple case, we are outside and crossing an outline, or we are inside and crossing a hole (this is always valid)

                            // also update the parent for the current polyline
                            _parentPolylines[size_t(*polylineIndex)] = currentTriangleData->parentPolylineIndex;

                            // just set index, which implicitly sets the opposite location
                            neighborTriangleParentPolylineIndex = polylineIndex;
                        }
                        else
                        {
                            // we are outside and crossing a hole, or we are inside and crossing an outline
                            if (currentTriangleData->parentPolylineIndex == *polylineIndex) DETRIA_LIKELY
                            {
                                // valid case, the neighbor triangle is the opposite location
                                // set its polyline index to the current polyline's parent

                                neighborTriangleParentPolylineIndex = _parentPolylines[size_t(*polylineIndex)];
                            }
                            else
                            {
                                // invalid case, either an outline is inside another outline, or a hole is inside another hole
                                return fail(TE_StackedPolylines{ .isHole = currentEdgeIsHole });
                            }
                        }
                    }
                    else
                    {
                        // simple case, we are not crossing outlines or holes, so just propagate the current location
                        neighborTriangleParentPolylineIndex = currentTriangleData->parentPolylineIndex;
                    }

                    _topologyMesh.setTriangleData(neighborTriangle, TriangleData
                    {
                        .data = typename TriangleData::KnownLocationData
                        {
                            .parentPolylineIndex = neighborTriangleParentPolylineIndex
                        }
                    });

                    trianglesToCheck.push_back(neighborTriangle);
                }
            }

            return true;
        }
        
        inline TTriangle getOtherTriangleOfEdge(const TTriangle& tri, const TEdge& edge)
        {
            // we only have 2 triangles per edge maximum
            // if the edge only has one triangle, then an invalid triangle is returned

            TTriangle otherTriangle{ };
            _topologyMesh.forEachTriangleOfEdge(edge, [&](const ETRelation& relation)
            {
                TTriangle currentTri = relation.triangle();
                if (currentTri.index != tri.index)
                {
                    otherTriangle = currentTri;
                }
            });

            return otherTriangle;
        };

        inline TriangleLocation getTriangleLocation(const typename TriangleData::KnownLocationData& data) const
        {
            if (!data.parentPolylineIndex.has_value())
            {
                return TriangleLocation::ConvexHull;
            }
            else
            {
                DETRIA_DEBUG_ASSERT(size_t(*data.parentPolylineIndex) < _polylines.size());
                EdgeType polylineType = _polylines[size_t(*data.parentPolylineIndex)].type;
                if (polylineType == EdgeType::AutoDetect)
                {
                    polylineType = _autoDetectedPolylineTypes[size_t(*data.parentPolylineIndex)];
                    DETRIA_DEBUG_ASSERT(polylineType == EdgeType::Outline || polylineType == EdgeType::Hole);
                }

                return polylineType == EdgeType::Hole ? TriangleLocation::Hole : TriangleLocation::Interior;
            }
        }

        inline TriangleLocation getTriangleLocation(const TTriangle& tri) const
        {
            const typename TriangleData::KnownLocationData* data =
                std::get_if<typename TriangleData::KnownLocationData>(&_topologyMesh.getTriangleData(tri)->data);

            if (data != nullptr)
            {
                return getTriangleLocation(*data);
            }

            DETRIA_DEBUG_ASSERT(false);
            return TriangleLocation::ConvexHull;
        }

        template <bool Flip = false>
        inline Tri getTriangleOriginalIndices(const TTriangle& tri) const
        {
            Idx x = _topologyMesh.getVertexOfTriangle(tri, 0).index;
            Idx y = _topologyMesh.getVertexOfTriangle(tri, 1).index;
            Idx z = _topologyMesh.getVertexOfTriangle(tri, 2).index;

            if constexpr (Flip)
            {
                return Tri{ .x = x, .y = z, .z = y };
            }
            else
            {
                return Tri{ .x = x, .y = y, .z = z };
            }
        }

        void forEachTriangleInternal(auto&& callback, bool cwTriangles, auto&& shouldProcessTriangle) const
        {
            _topologyMesh.forEachTriangle([&](const TTriangle& tri)
            {
                if (shouldProcessTriangle(tri))
                {
                    if (cwTriangles)
                    {
                        callback(getTriangleOriginalIndices<false>(tri));
                    }
                    else
                    {
                        callback(getTriangleOriginalIndices<true>(tri));
                    }
                }
            });
        }

        inline static math::Orientation orient2d(Vector2 a, Vector2 b, Vector2 c)
        {
            return math::orient2d<Config::UseRobustOrientationTests>(a, b, c);
        }

        inline static math::CircleLocation incircle(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            return math::incircle<Config::UseRobustIncircleTests>(a, b, c, d);
        }

    private:
        Allocator _allocator;

        // inputs
        detail::ReadonlySpan<Point> _points;
        CollectionWithAllocator<PolylineData> _polylines;
        CollectionWithAllocator<Vec2<Idx>> _manuallyConstrainedEdges;
        CollectionWithAllocator<EdgeType> _autoDetectedPolylineTypes;

        TMesh _topologyMesh;

        // reused containers for multiple calculations
        CollectionWithAllocator<Idx> _initialTriangulation_SortedPoints;
        CollectionWithAllocator<TVertex> _constrainedEdgeVerticesCW;
        CollectionWithAllocator<TVertex> _constrainedEdgeVerticesCCW;
        CollectionWithAllocator<TVertex> _constrainedEdgeReTriangulationStack;
        CollectionWithAllocator<bool> _classifyTriangles_CheckedTriangles;
        CollectionWithAllocator<TTriangle> _classifyTriangles_TrianglesToCheck;

        // results which are not related to the triangulation directly
        List _convexHullPoints;
        Idx _convexHullStartIndex = Idx(-1);
        CollectionWithAllocator<std::optional<Idx>> _parentPolylines;

        struct TopologyEdgeWithVertices
        {
            TVertex v0;
            TVertex v1;
            TEdge edge;
        };

        CollectionWithAllocator<TopologyEdgeWithVertices> _delaunayCheckStack;

        TriangulationErrorData _error = TE_NotStarted{ };
    };

#undef DETRIA_CHECK
#undef DETRIA_ASSERT
#undef DETRIA_ASSERT_MSG
#undef DETRIA_DEBUG_ASSERT

#undef DETRIA_LIKELY
#undef DETRIA_UNLIKELY
}

#endif // DETRIA_HPP_INCLUDED

// License
// This library is available under 2 licenses, you can choose whichever you prefer.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Alternative A - Do What The Fuck You Want To Public License
//
// Copyright (c) Kimbatt (https://github.com/Kimbatt)
//
// This work is free. You can redistribute it and/or modify it under the terms of
// the Do What The Fuck You Want To Public License, Version 2, as published by Sam Hocevar.
//
//            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
//                    Version 2, December 2004
//
// Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>
//
// Everyone is permitted to copy and distribute verbatim or modified
// copies of this license document, and changing it is allowed as long
// as the name is changed.
//
//            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
//   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
//
// 0. You just DO WHAT THE FUCK YOU WANT TO.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Alternative B - MIT License
//
// Copyright (c) Kimbatt (https://github.com/Kimbatt)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
