/// detria - a delaunay triangulation library
///
/// Example usage:
/// <example>
/// 
/// // Create a square, and triangulate it
/// 
/// // List of points (positions)
/// var points = new detria.Vec2[]
/// {
///     new detria.Vec2(0.0f, 0.0f),
///     new detria.Vec2(1.0f, 0.0f),
///     new detria.Vec2(1.0f, 1.0f),
///     new detria.Vec2(0.0f, 1.0f),
/// };
/// 
/// // List of point indices
/// var outline = new int[] { 0, 1, 2, 3 };
/// 
/// bool delaunay = true;
/// 
/// var tri = new detria.Triangulation();
/// tri.SetPoints(points);
/// tri.AddOutline(outline);
/// 
/// bool success = tri.Triangulate(delaunay);
/// 
/// if (success)
/// {
///     bool cwTriangles = true;
/// 
///     foreach (detria.Triangle triangle in tri.EnumerateTriangles(cwTriangles))
///     {
///         // `triangle` contains the point indices
/// 
///         var firstPointOfTriangle = points[triangle.x];
///         var secondPointOfTriangle = points[triangle.y];
///         var thirdPointOfTriangle = points[triangle.z];
/// 
///         // Use the results
///         Console.WriteLine($"Triangle: ({firstPointOfTriangle}), ({secondPointOfTriangle}), ({thirdPointOfTriangle})");
///     }
/// }
/// 
/// </example>
///
/// See https://github.com/Kimbatt/detria for more information.
/// License: WTFPL or MIT, at your choice. You can find the license texts at the bottom of this file.

using Scalar = float;
using Idx = int;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

namespace detria
{
    public struct Vec2
    {
        public Scalar x;
        public Scalar y;

        public Vec2(Scalar x, Scalar y)
        {
            this.x = x;
            this.y = y;
        }

        public override string ToString()
        {
            return $"{x}, {y}";
        }
    }

    public struct Triangle
    {
        public Idx x;
        public Idx y;
        public Idx z;

        public override string ToString()
        {
            return $"{x}, {y}, {z}";
        }
    }

    internal struct Edge
    {
        public Idx x;
        public Idx y;

        public override string ToString()
        {
            return $"{x}, {y}";
        }
    }

    public class Predicates
    {
        // http://www.cs.cmu.edu/~quake/robust.html
        // Routines for Arbitrary Precision Floating-point Arithmetic and Fast Robust Geometric Predicates
        // Placed in the public domain by Jonathan Richard Shewchuk

        internal static readonly ErrorBounds errorBounds;

        static Predicates()
        {
            errorBounds = CalculateErrorBounds();
        }

        private readonly Scalar[] _buffer;

        private DataView B;
        private readonly DataView C1;
        private readonly DataView C2;
        private readonly DataView D;
        private DataView u;
        private DataView bc;
        private DataView ca;
        private DataView ab;
        private readonly DataView axbc;
        private readonly DataView axxbc;
        private readonly DataView aybc;
        private readonly DataView ayybc;
        private readonly DataView adet;
        private readonly DataView bxca;
        private readonly DataView bxxca;
        private readonly DataView byca;
        private readonly DataView byyca;
        private readonly DataView bdet;
        private readonly DataView cxab;
        private readonly DataView cxxab;
        private readonly DataView cyab;
        private readonly DataView cyyab;
        private readonly DataView cdet;
        private readonly DataView abdet;
        private readonly DataView fin1;
        private readonly DataView fin2;
        private DataView aa;
        private DataView bb;
        private DataView cc;
        private DataView v;
        private readonly DataView temp8;
        private readonly DataView temp16a;
        private readonly DataView temp16b;
        private readonly DataView temp16c;
        private readonly DataView temp32a;
        private readonly DataView temp32b;
        private readonly DataView temp48;
        private readonly DataView temp64;
        private readonly DataView axtbb;
        private readonly DataView axtcc;
        private readonly DataView aytbb;
        private readonly DataView aytcc;
        private readonly DataView bxtaa;
        private readonly DataView bxtcc;
        private readonly DataView bytaa;
        private readonly DataView bytcc;
        private readonly DataView cxtaa;
        private readonly DataView cxtbb;
        private readonly DataView cytaa;
        private readonly DataView cytbb;
        private readonly DataView axtbc;
        private readonly DataView aytbc;
        private readonly DataView bxtca;
        private readonly DataView bytca;
        private readonly DataView cxtab;
        private readonly DataView cytab;
        private readonly DataView axtbct;
        private readonly DataView aytbct;
        private readonly DataView bxtcat;
        private readonly DataView bytcat;
        private readonly DataView cxtabt;
        private readonly DataView cytabt;
        private readonly DataView axtbctt;
        private readonly DataView aytbctt;
        private readonly DataView bxtcatt;
        private readonly DataView bytcatt;
        private readonly DataView cxtabtt;
        private readonly DataView cytabtt;
        private DataView abt;
        private DataView bct;
        private DataView cat;
        private DataView abtt;
        private DataView bctt;
        private DataView catt;

        public Predicates()
        {
            const int totalCount =
               4 + // B
               8 + // C1
               12 + // C2
               16 + // D
               4 + // u
               4 + // bc
               4 + // ca
               4 + // ab
               8 + // axbc
               16 + // axxbc
               8 + // aybc
               16 + // ayybc
               32 + // adet
               8 + // bxca
               16 + // bxxca
               8 + // byca
               16 + // byyca
               32 + // bdet
               8 + // cxab
               16 + // cxxab
               8 + // cyab
               16 + // cyyab
               32 + // cdet
               64 + // abdet
               1152 + // fin1
               1152 + // fin2
               4 + // aa
               4 + // bb
               4 + // cc
               4 + // v
               8 + // temp8
               16 + // temp16a
               16 + // temp16b
               16 + // temp16c
               32 + // temp32a
               32 + // temp32b
               48 + // temp48
               64 + // temp64
               8 + // axtbb
               8 + // axtcc
               8 + // aytbb
               8 + // aytcc
               8 + // bxtaa
               8 + // bxtcc
               8 + // bytaa
               8 + // bytcc
               8 + // cxtaa
               8 + // cxtbb
               8 + // cytaa
               8 + // cytbb
               8 + // axtbc
               8 + // aytbc
               8 + // bxtca
               8 + // bytca
               8 + // cxtab
               8 + // cytab
               16 + // axtbct
               16 + // aytbct
               16 + // bxtcat
               16 + // bytcat
               16 + // cxtabt
               16 + // cytabt
               8 + // axtbctt
               8 + // aytbctt
               8 + // bxtcatt
               8 + // bytcatt
               8 + // cxtabtt
               8 + // cytabtt
               8 + // abt
               8 + // bct
               8 + // cat
               4 + // abtt
               4 + // bctt
               4; // catt

            _buffer = new Scalar[totalCount];

            int offset = 0;
            DataView CreateDataView(int count)
            {
                DataView view = new DataView(_buffer, offset, count);
                offset += count;
                return view;
            }

            B = CreateDataView(4);
            C1 = CreateDataView(8);
            C2 = CreateDataView(12);
            D = CreateDataView(16);
            u = CreateDataView(4);
            bc = CreateDataView(4);
            ca = CreateDataView(4);
            ab = CreateDataView(4);
            axbc = CreateDataView(8);
            axxbc = CreateDataView(16);
            aybc = CreateDataView(8);
            ayybc = CreateDataView(16);
            adet = CreateDataView(32);
            bxca = CreateDataView(8);
            bxxca = CreateDataView(16);
            byca = CreateDataView(8);
            byyca = CreateDataView(16);
            bdet = CreateDataView(32);
            cxab = CreateDataView(8);
            cxxab = CreateDataView(16);
            cyab = CreateDataView(8);
            cyyab = CreateDataView(16);
            cdet = CreateDataView(32);
            abdet = CreateDataView(64);
            fin1 = CreateDataView(1152);
            fin2 = CreateDataView(1152);
            aa = CreateDataView(4);
            bb = CreateDataView(4);
            cc = CreateDataView(4);
            v = CreateDataView(4);
            temp8 = CreateDataView(8);
            temp16a = CreateDataView(16);
            temp16b = CreateDataView(16);
            temp16c = CreateDataView(16);
            temp32a = CreateDataView(32);
            temp32b = CreateDataView(32);
            temp48 = CreateDataView(48);
            temp64 = CreateDataView(64);
            axtbb = CreateDataView(8);
            axtcc = CreateDataView(8);
            aytbb = CreateDataView(8);
            aytcc = CreateDataView(8);
            bxtaa = CreateDataView(8);
            bxtcc = CreateDataView(8);
            bytaa = CreateDataView(8);
            bytcc = CreateDataView(8);
            cxtaa = CreateDataView(8);
            cxtbb = CreateDataView(8);
            cytaa = CreateDataView(8);
            cytbb = CreateDataView(8);
            axtbc = CreateDataView(8);
            aytbc = CreateDataView(8);
            bxtca = CreateDataView(8);
            bytca = CreateDataView(8);
            cxtab = CreateDataView(8);
            cytab = CreateDataView(8);
            axtbct = CreateDataView(16);
            aytbct = CreateDataView(16);
            bxtcat = CreateDataView(16);
            bytcat = CreateDataView(16);
            cxtabt = CreateDataView(16);
            cytabt = CreateDataView(16);
            axtbctt = CreateDataView(8);
            aytbctt = CreateDataView(8);
            bxtcatt = CreateDataView(8);
            bytcatt = CreateDataView(8);
            cxtabtt = CreateDataView(8);
            cytabtt = CreateDataView(8);
            abt = CreateDataView(8);
            bct = CreateDataView(8);
            cat = CreateDataView(8);
            abtt = CreateDataView(4);
            bctt = CreateDataView(4);
            catt = CreateDataView(4);

#if DEBUG
            if (offset != totalCount)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif
        }

        internal static ErrorBounds CalculateErrorBounds()
        {
            Scalar check = 1;
            bool everyOther = true;

            Scalar epsilon = 1;
            Scalar splitter = 1;

            Scalar lastcheck;
            do
            {
                lastcheck = check;
                epsilon *= 0.5f;
                if (everyOther)
                {
                    splitter *= 2;
                }
                everyOther = !everyOther;
                check = 1 + epsilon;
            }
            while (check != 1 && (check != lastcheck));

            splitter += 1;

            return new ErrorBounds
            {
                splitter = splitter,
                epsilon = epsilon,
                resulterrbound = (3 + 8 * epsilon) * epsilon,
                ccwerrboundA = (3 + 16 * epsilon) * epsilon,
                ccwerrboundB = (2 + 12 * epsilon) * epsilon,
                ccwerrboundC = (9 + 64 * epsilon) * epsilon * epsilon,
                o3derrboundA = (7 + 56 * epsilon) * epsilon,
                o3derrboundB = (3 + 28 * epsilon) * epsilon,
                o3derrboundC = (26 + 288 * epsilon) * epsilon * epsilon,
                iccerrboundA = (10 + 96 * epsilon) * epsilon,
                iccerrboundB = (4 + 48 * epsilon) * epsilon,
                iccerrboundC = (44 + 576 * epsilon) * epsilon * epsilon,
                isperrboundA = (16 + 224 * epsilon) * epsilon,
                isperrboundB = (5 + 72 * epsilon) * epsilon,
                isperrboundC = (71 + 1408 * epsilon) * epsilon * epsilon,
            };
        }

        private static Scalar Estimate(int elen, DataView e)
        {
            Scalar Q = e[0];
            for (int eindex = 1; eindex < elen; eindex++)
            {
                Q += e[eindex];
            }
            return Q;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Scalar Absolute(Scalar a)
        {
            return a >= 0 ? a : -a;
        }

        private static int FastExpansionSumZeroelim(int elen, DataView e, int flen, DataView f, DataView h)
        {
            Scalar Q;
            Scalar Qnew;
            Scalar hh;
            Scalar bvirt;
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
                    Qnew = enow + Q; bvirt = Qnew - enow; hh = Q - bvirt;
                    enow = e[++eindex];
                }
                else
                {
                    Qnew = fnow + Q; bvirt = Qnew - fnow; hh = Q - bvirt;
                    fnow = f[++findex];
                }
                Q = Qnew;
                if (hh != 0)
                {
                    h[hindex++] = hh;
                }
                while ((eindex < elen) && (findex < flen))
                {
                    if ((fnow > enow) == (fnow > -enow))
                    {
                        Qnew = Q + enow; bvirt = Qnew - Q; avirt = Qnew - bvirt; bround = enow - bvirt; around = Q - avirt; hh = around + bround;

                        if (++eindex < elen)
                        {
                            enow = e[eindex];
                        }
                    }
                    else
                    {
                        Qnew = Q + fnow; bvirt = Qnew - Q; avirt = Qnew - bvirt; bround = fnow - bvirt; around = Q - avirt; hh = around + bround;

                        if (++findex < flen)
                        {
                            fnow = f[findex];
                        }
                    }
                    Q = Qnew;
                    if (hh != 0)
                    {
                        h[hindex++] = hh;
                    }
                }
            }
            while (eindex < elen)
            {
                Qnew = Q + enow; bvirt = Qnew - Q; avirt = Qnew - bvirt; bround = enow - bvirt; around = Q - avirt; hh = around + bround;

                if (++eindex < elen)
                {
                    enow = e[eindex];
                }

                Q = Qnew;
                if (hh != 0)
                {
                    h[hindex++] = hh;
                }
            }
            while (findex < flen)
            {
                Qnew = Q + fnow; bvirt = Qnew - Q; avirt = Qnew - bvirt; bround = fnow - bvirt; around = Q - avirt; hh = around + bround;

                if (++findex < flen)
                {
                    fnow = f[findex];
                }

                Q = Qnew;
                if (hh != 0)
                {
                    h[hindex++] = hh;
                }
            }
            if ((Q != 0) || (hindex == 0))
            {
                h[hindex++] = Q;
            }
            return hindex;
        }

        public Scalar Orient2dAdapt(Scalar paX, Scalar paY, Scalar pbX, Scalar pbY, Scalar pcX, Scalar pcY, Scalar detsum)
        {
            Scalar acx, acy, bcx, bcy;
            Scalar acxtail, acytail, bcxtail, bcytail;
            Scalar detleft, detright;
            Scalar detlefttail, detrighttail;
            Scalar det, errbound;
            Scalar B3;
            int C1length, C2length, Dlength;
            Scalar u3;
            Scalar s1, t1;
            Scalar s0, t0;

            Scalar bvirt;
            Scalar avirt, bround, around;
            Scalar c;
            Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;
            Scalar _i, _j;
            Scalar _0;

            acx = paX - pcX;
            bcx = pbX - pcX;
            acy = paY - pcY;
            bcy = pbY - pcY;

            detleft = acx * bcy; c = errorBounds.splitter * acx; abig = c - acx; ahi = c - abig; alo = acx - ahi; c = errorBounds.splitter * bcy; abig = c - bcy; bhi = c - abig; blo = bcy - bhi; err1 = detleft - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); detlefttail = (alo * blo) - err3;
            detright = acy * bcx; c = errorBounds.splitter * acy; abig = c - acy; ahi = c - abig; alo = acy - ahi; c = errorBounds.splitter * bcx; abig = c - bcx; bhi = c - abig; blo = bcx - bhi; err1 = detright - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); detrighttail = (alo * blo) - err3;

            _i = (detlefttail - detrighttail); bvirt = (detlefttail - _i); avirt = _i + bvirt; bround = bvirt - detrighttail; around = detlefttail - avirt; B[0] = around + bround; _j = (detleft + _i); bvirt = (_j - detleft); avirt = _j - bvirt; bround = _i - bvirt; around = detleft - avirt; _0 = around + bround; _i = (_0 - detright); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - detright; around = _0 - avirt; B[1] = around + bround; B3 = (_j + _i); bvirt = (B3 - _j); avirt = B3 - bvirt; bround = _i - bvirt; around = _j - avirt; B[2] = around + bround;
            B[3] = B3;

            det = Estimate(4, B);
            errbound = errorBounds.ccwerrboundB * detsum;
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            bvirt = (paX - acx); avirt = acx + bvirt; bround = bvirt - pcX; around = paX - avirt; acxtail = around + bround;
            bvirt = (pbX - bcx); avirt = bcx + bvirt; bround = bvirt - pcX; around = pbX - avirt; bcxtail = around + bround;
            bvirt = (paY - acy); avirt = acy + bvirt; bround = bvirt - pcY; around = paY - avirt; acytail = around + bround;
            bvirt = (pbY - bcy); avirt = bcy + bvirt; bround = bvirt - pcY; around = pbY - avirt; bcytail = around + bround;

            if ((acxtail == (0)) && (acytail == (0)) && (bcxtail == (0)) && (bcytail == (0)))
            {
                return det;
            }

            errbound = errorBounds.ccwerrboundC * detsum + errorBounds.resulterrbound * Absolute(det);
            det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            s1 = (acxtail * bcy); c = (errorBounds.splitter * acxtail); abig = (c - acxtail); ahi = c - abig; alo = acxtail - ahi; c = (errorBounds.splitter * bcy); abig = (c - bcy); bhi = c - abig; blo = bcy - bhi; err1 = s1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); s0 = (alo * blo) - err3;
            t1 = (acytail * bcx); c = (errorBounds.splitter * acytail); abig = (c - acytail); ahi = c - abig; alo = acytail - ahi; c = (errorBounds.splitter * bcx); abig = (c - bcx); bhi = c - abig; blo = bcx - bhi; err1 = t1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); t0 = (alo * blo) - err3;
            _i = (s0 - t0); bvirt = (s0 - _i); avirt = _i + bvirt; bround = bvirt - t0; around = s0 - avirt; u[0] = around + bround; _j = (s1 + _i); bvirt = (_j - s1); avirt = _j - bvirt; bround = _i - bvirt; around = s1 - avirt; _0 = around + bround; _i = (_0 - t1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - t1; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
            u[3] = u3;
            C1length = FastExpansionSumZeroelim(4, B, 4, u, C1);

            s1 = (acx * bcytail); c = (errorBounds.splitter * acx); abig = (c - acx); ahi = c - abig; alo = acx - ahi; c = (errorBounds.splitter * bcytail); abig = (c - bcytail); bhi = c - abig; blo = bcytail - bhi; err1 = s1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); s0 = (alo * blo) - err3;
            t1 = (acy * bcxtail); c = (errorBounds.splitter * acy); abig = (c - acy); ahi = c - abig; alo = acy - ahi; c = (errorBounds.splitter * bcxtail); abig = (c - bcxtail); bhi = c - abig; blo = bcxtail - bhi; err1 = t1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); t0 = (alo * blo) - err3;
            _i = (s0 - t0); bvirt = (s0 - _i); avirt = _i + bvirt; bround = bvirt - t0; around = s0 - avirt; u[0] = around + bround; _j = (s1 + _i); bvirt = (_j - s1); avirt = _j - bvirt; bround = _i - bvirt; around = s1 - avirt; _0 = around + bround; _i = (_0 - t1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - t1; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
            u[3] = u3;
            C2length = FastExpansionSumZeroelim(C1length, C1, 4, u, C2);

            s1 = (acxtail * bcytail); c = (errorBounds.splitter * acxtail); abig = (c - acxtail); ahi = c - abig; alo = acxtail - ahi; c = (errorBounds.splitter * bcytail); abig = (c - bcytail); bhi = c - abig; blo = bcytail - bhi; err1 = s1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); s0 = (alo * blo) - err3;
            t1 = (acytail * bcxtail); c = (errorBounds.splitter * acytail); abig = (c - acytail); ahi = c - abig; alo = acytail - ahi; c = (errorBounds.splitter * bcxtail); abig = (c - bcxtail); bhi = c - abig; blo = bcxtail - bhi; err1 = t1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); t0 = (alo * blo) - err3;
            _i = (s0 - t0); bvirt = (s0 - _i); avirt = _i + bvirt; bround = bvirt - t0; around = s0 - avirt; u[0] = around + bround; _j = (s1 + _i); bvirt = (_j - s1); avirt = _j - bvirt; bround = _i - bvirt; around = s1 - avirt; _0 = around + bround; _i = (_0 - t1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - t1; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
            u[3] = u3;
            Dlength = FastExpansionSumZeroelim(C2length, C2, 4, u, D);

            return (D[Dlength - 1]);
        }

        private static int ScaleExpansionZeroelim(int elen, DataView e, Scalar b, DataView h)
        {
            Scalar Q, sum;
            Scalar hh;
            Scalar product1;
            Scalar product0;
            int eindex, hindex;
            Scalar enow;
            Scalar bvirt;
            Scalar avirt, bround, around;
            Scalar c;
            Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;

            c = (errorBounds.splitter * b); abig = (c - b); bhi = c - abig; blo = b - bhi;
            Q = (e[0] * b); c = (errorBounds.splitter * e[0]); abig = (c - e[0]); ahi = c - abig; alo = e[0] - ahi; err1 = Q - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); hh = (alo * blo) - err3;
            hindex = 0;
            if (hh != (0))
            {
                h[hindex++] = hh;
            }

            for (eindex = 1; eindex < elen; eindex++)
            {
                enow = e[eindex];
                product1 = (enow * b); c = (errorBounds.splitter * enow); abig = (c - enow); ahi = c - abig; alo = enow - ahi; err1 = product1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); product0 = (alo * blo) - err3;
                sum = (Q + product0); bvirt = (sum - Q); avirt = sum - bvirt; bround = product0 - bvirt; around = Q - avirt; hh = around + bround;
                if (hh != (0))
                {
                    h[hindex++] = hh;
                }

                Q = (product1 + sum); bvirt = Q - product1; hh = sum - bvirt;
                if (hh != (0))
                {
                    h[hindex++] = hh;
                }
            }

            if (Q != (0) || hindex == 0)
            {
                h[hindex++] = Q;
            }

            return hindex;
        }

        public Scalar IncircleAdapt(Scalar paX, Scalar paY, Scalar pbX, Scalar pbY, Scalar pcX, Scalar pcY, Scalar pdX, Scalar pdY, Scalar permanent)
        {
            Scalar adx, bdx, cdx, ady, bdy, cdy;
            Scalar det, errbound;

            Scalar bdxcdy1, cdxbdy1, cdxady1, adxcdy1, adxbdy1, bdxady1;
            Scalar bdxcdy0, cdxbdy0, cdxady0, adxcdy0, adxbdy0, bdxady0;
            Scalar bc3, ca3, ab3;
            int axbclen, axxbclen, aybclen, ayybclen, alen;
            int bxcalen, bxxcalen, bycalen, byycalen, blen;
            int cxablen, cxxablen, cyablen, cyyablen, clen;
            int ablen;
            DataView finnow, finother, finswap;
            int finlength;

            Scalar adxtail, bdxtail, cdxtail, adytail, bdytail, cdytail;
            Scalar adxadx1, adyady1, bdxbdx1, bdybdy1, cdxcdx1, cdycdy1;
            Scalar adxadx0, adyady0, bdxbdx0, bdybdy0, cdxcdx0, cdycdy0;
            Scalar aa3, bb3, cc3;
            Scalar ti1, tj1;
            Scalar ti0, tj0;
            Scalar u3, v3;
            int temp8len, temp16alen, temp16blen, temp16clen;
            int temp32alen, temp32blen, temp48len, temp64len;
            int axtbblen, axtcclen, aytbblen, aytcclen;
            int bxtaalen, bxtcclen, bytaalen, bytcclen;
            int cxtaalen, cxtbblen, cytaalen, cytbblen;
            int axtbclen, aytbclen, bxtcalen, bytcalen, cxtablen, cytablen;
            int axtbctlen, aytbctlen, bxtcatlen, bytcatlen, cxtabtlen, cytabtlen;
            int axtbcttlen, aytbcttlen, bxtcattlen, bytcattlen, cxtabttlen, cytabttlen;
            int abtlen, bctlen, catlen;
            int abttlen, bcttlen, cattlen;
            Scalar abtt3, bctt3, catt3;
            Scalar negate;

            Scalar bvirt;
            Scalar avirt, bround, around;
            Scalar c;
            Scalar abig;
            Scalar ahi, alo, bhi, blo;
            Scalar err1, err2, err3;
            Scalar _i, _j;
            Scalar _0;

            axtbclen = 0;
            aytbclen = 0;
            bxtcalen = 0;
            bytcalen = 0;
            cxtablen = 0;
            cytablen = 0;

            adx = (paX - pdX);
            bdx = (pbX - pdX);
            cdx = (pcX - pdX);
            ady = (paY - pdY);
            bdy = (pbY - pdY);
            cdy = (pcY - pdY);

            bdxcdy1 = (bdx * cdy); c = (errorBounds.splitter * bdx); abig = (c - bdx); ahi = c - abig; alo = bdx - ahi; c = (errorBounds.splitter * cdy); abig = (c - cdy); bhi = c - abig; blo = cdy - bhi; err1 = bdxcdy1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); bdxcdy0 = (alo * blo) - err3;
            cdxbdy1 = (cdx * bdy); c = (errorBounds.splitter * cdx); abig = (c - cdx); ahi = c - abig; alo = cdx - ahi; c = (errorBounds.splitter * bdy); abig = (c - bdy); bhi = c - abig; blo = bdy - bhi; err1 = cdxbdy1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); cdxbdy0 = (alo * blo) - err3;
            _i = (bdxcdy0 - cdxbdy0); bvirt = (bdxcdy0 - _i); avirt = _i + bvirt; bround = bvirt - cdxbdy0; around = bdxcdy0 - avirt; bc[0] = around + bround; _j = (bdxcdy1 + _i); bvirt = (_j - bdxcdy1); avirt = _j - bvirt; bround = _i - bvirt; around = bdxcdy1 - avirt; _0 = around + bround; _i = (_0 - cdxbdy1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - cdxbdy1; around = _0 - avirt; bc[1] = around + bround; bc3 = (_j + _i); bvirt = (bc3 - _j); avirt = bc3 - bvirt; bround = _i - bvirt; around = _j - avirt; bc[2] = around + bround;
            bc[3] = bc3;
            axbclen = ScaleExpansionZeroelim(4, bc, adx, axbc);
            axxbclen = ScaleExpansionZeroelim(axbclen, axbc, adx, axxbc);
            aybclen = ScaleExpansionZeroelim(4, bc, ady, aybc);
            ayybclen = ScaleExpansionZeroelim(aybclen, aybc, ady, ayybc);
            alen = FastExpansionSumZeroelim(axxbclen, axxbc, ayybclen, ayybc, adet);

            cdxady1 = (cdx * ady); c = (errorBounds.splitter * cdx); abig = (c - cdx); ahi = c - abig; alo = cdx - ahi; c = (errorBounds.splitter * ady); abig = (c - ady); bhi = c - abig; blo = ady - bhi; err1 = cdxady1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); cdxady0 = (alo * blo) - err3;
            adxcdy1 = (adx * cdy); c = (errorBounds.splitter * adx); abig = (c - adx); ahi = c - abig; alo = adx - ahi; c = (errorBounds.splitter * cdy); abig = (c - cdy); bhi = c - abig; blo = cdy - bhi; err1 = adxcdy1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); adxcdy0 = (alo * blo) - err3;
            _i = (cdxady0 - adxcdy0); bvirt = (cdxady0 - _i); avirt = _i + bvirt; bround = bvirt - adxcdy0; around = cdxady0 - avirt; ca[0] = around + bround; _j = (cdxady1 + _i); bvirt = (_j - cdxady1); avirt = _j - bvirt; bround = _i - bvirt; around = cdxady1 - avirt; _0 = around + bround; _i = (_0 - adxcdy1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - adxcdy1; around = _0 - avirt; ca[1] = around + bround; ca3 = (_j + _i); bvirt = (ca3 - _j); avirt = ca3 - bvirt; bround = _i - bvirt; around = _j - avirt; ca[2] = around + bround;
            ca[3] = ca3;
            bxcalen = ScaleExpansionZeroelim(4, ca, bdx, bxca);
            bxxcalen = ScaleExpansionZeroelim(bxcalen, bxca, bdx, bxxca);
            bycalen = ScaleExpansionZeroelim(4, ca, bdy, byca);
            byycalen = ScaleExpansionZeroelim(bycalen, byca, bdy, byyca);
            blen = FastExpansionSumZeroelim(bxxcalen, bxxca, byycalen, byyca, bdet);

            adxbdy1 = (adx * bdy); c = (errorBounds.splitter * adx); abig = (c - adx); ahi = c - abig; alo = adx - ahi; c = (errorBounds.splitter * bdy); abig = (c - bdy); bhi = c - abig; blo = bdy - bhi; err1 = adxbdy1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); adxbdy0 = (alo * blo) - err3;
            bdxady1 = (bdx * ady); c = (errorBounds.splitter * bdx); abig = (c - bdx); ahi = c - abig; alo = bdx - ahi; c = (errorBounds.splitter * ady); abig = (c - ady); bhi = c - abig; blo = ady - bhi; err1 = bdxady1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); bdxady0 = (alo * blo) - err3;
            _i = (adxbdy0 - bdxady0); bvirt = (adxbdy0 - _i); avirt = _i + bvirt; bround = bvirt - bdxady0; around = adxbdy0 - avirt; ab[0] = around + bround; _j = (adxbdy1 + _i); bvirt = (_j - adxbdy1); avirt = _j - bvirt; bround = _i - bvirt; around = adxbdy1 - avirt; _0 = around + bround; _i = (_0 - bdxady1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - bdxady1; around = _0 - avirt; ab[1] = around + bround; ab3 = (_j + _i); bvirt = (ab3 - _j); avirt = ab3 - bvirt; bround = _i - bvirt; around = _j - avirt; ab[2] = around + bround;
            ab[3] = ab3;
            cxablen = ScaleExpansionZeroelim(4, ab, cdx, cxab);
            cxxablen = ScaleExpansionZeroelim(cxablen, cxab, cdx, cxxab);
            cyablen = ScaleExpansionZeroelim(4, ab, cdy, cyab);
            cyyablen = ScaleExpansionZeroelim(cyablen, cyab, cdy, cyyab);
            clen = FastExpansionSumZeroelim(cxxablen, cxxab, cyyablen, cyyab, cdet);

            ablen = FastExpansionSumZeroelim(alen, adet, blen, bdet, abdet);
            finlength = FastExpansionSumZeroelim(ablen, abdet, clen, cdet, fin1);

            det = Estimate(finlength, fin1);
            errbound = errorBounds.iccerrboundB * permanent;
            if ((det >= errbound) || (-det >= errbound))
            {
                return det;
            }

            bvirt = (paX - adx); avirt = adx + bvirt; bround = bvirt - pdX; around = paX - avirt; adxtail = around + bround;
            bvirt = (paY - ady); avirt = ady + bvirt; bround = bvirt - pdY; around = paY - avirt; adytail = around + bround;
            bvirt = (pbX - bdx); avirt = bdx + bvirt; bround = bvirt - pdX; around = pbX - avirt; bdxtail = around + bround;
            bvirt = (pbY - bdy); avirt = bdy + bvirt; bround = bvirt - pdY; around = pbY - avirt; bdytail = around + bround;
            bvirt = (pcX - cdx); avirt = cdx + bvirt; bround = bvirt - pdX; around = pcX - avirt; cdxtail = around + bround;
            bvirt = (pcY - cdy); avirt = cdy + bvirt; bround = bvirt - pdY; around = pcY - avirt; cdytail = around + bround;

            if (adxtail == (0) && bdxtail == (0) && cdxtail == (0) &&
                adytail == (0) && bdytail == (0) && cdytail == (0))
            {
                return det;
            }

            errbound = errorBounds.iccerrboundC * permanent + errorBounds.resulterrbound * Absolute(det);
            det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail)
                - (bdy * cdxtail + cdx * bdytail))
                + (2) * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
                + ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail)
                    - (cdy * adxtail + adx * cdytail))
                    + (2) * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
                + ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail)
                    - (ady * bdxtail + bdx * adytail))
                    + (2) * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));

            if (det >= errbound || -det >= errbound)
            {
                return det;
            }

            finnow = fin1;
            finother = fin2;

            if (bdxtail != (0) || bdytail != (0) || cdxtail != (0) || cdytail != (0))
            {
                adxadx1 = (adx * adx); c = (errorBounds.splitter * adx); abig = (c - adx); ahi = c - abig; alo = adx - ahi; err1 = adxadx1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); adxadx0 = (alo * alo) - err3;
                adyady1 = (ady * ady); c = (errorBounds.splitter * ady); abig = (c - ady); ahi = c - abig; alo = ady - ahi; err1 = adyady1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); adyady0 = (alo * alo) - err3;
                _i = (adxadx0 + adyady0); bvirt = (_i - adxadx0); avirt = _i - bvirt; bround = adyady0 - bvirt; around = adxadx0 - avirt; aa[0] = around + bround; _j = (adxadx1 + _i); bvirt = (_j - adxadx1); avirt = _j - bvirt; bround = _i - bvirt; around = adxadx1 - avirt; _0 = around + bround; _i = (_0 + adyady1); bvirt = (_i - _0); avirt = _i - bvirt; bround = adyady1 - bvirt; around = _0 - avirt; aa[1] = around + bround; aa3 = (_j + _i); bvirt = (aa3 - _j); avirt = aa3 - bvirt; bround = _i - bvirt; around = _j - avirt; aa[2] = around + bround;
                aa[3] = aa3;
            }
            if (cdxtail != (0) || cdytail != (0) || adxtail != (0) || adytail != (0))
            {
                bdxbdx1 = (bdx * bdx); c = (errorBounds.splitter * bdx); abig = (c - bdx); ahi = c - abig; alo = bdx - ahi; err1 = bdxbdx1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); bdxbdx0 = (alo * alo) - err3;
                bdybdy1 = (bdy * bdy); c = (errorBounds.splitter * bdy); abig = (c - bdy); ahi = c - abig; alo = bdy - ahi; err1 = bdybdy1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); bdybdy0 = (alo * alo) - err3;
                _i = (bdxbdx0 + bdybdy0); bvirt = (_i - bdxbdx0); avirt = _i - bvirt; bround = bdybdy0 - bvirt; around = bdxbdx0 - avirt; bb[0] = around + bround; _j = (bdxbdx1 + _i); bvirt = (_j - bdxbdx1); avirt = _j - bvirt; bround = _i - bvirt; around = bdxbdx1 - avirt; _0 = around + bround; _i = (_0 + bdybdy1); bvirt = (_i - _0); avirt = _i - bvirt; bround = bdybdy1 - bvirt; around = _0 - avirt; bb[1] = around + bround; bb3 = (_j + _i); bvirt = (bb3 - _j); avirt = bb3 - bvirt; bround = _i - bvirt; around = _j - avirt; bb[2] = around + bround;
                bb[3] = bb3;
            }
            if (adxtail != (0) || adytail != (0) || bdxtail != (0) || bdytail != (0))
            {
                cdxcdx1 = (cdx * cdx); c = (errorBounds.splitter * cdx); abig = (c - cdx); ahi = c - abig; alo = cdx - ahi; err1 = cdxcdx1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); cdxcdx0 = (alo * alo) - err3;
                cdycdy1 = (cdy * cdy); c = (errorBounds.splitter * cdy); abig = (c - cdy); ahi = c - abig; alo = cdy - ahi; err1 = cdycdy1 - (ahi * ahi); err3 = err1 - ((ahi + ahi) * alo); cdycdy0 = (alo * alo) - err3;
                _i = (cdxcdx0 + cdycdy0); bvirt = (_i - cdxcdx0); avirt = _i - bvirt; bround = cdycdy0 - bvirt; around = cdxcdx0 - avirt; cc[0] = around + bround; _j = (cdxcdx1 + _i); bvirt = (_j - cdxcdx1); avirt = _j - bvirt; bround = _i - bvirt; around = cdxcdx1 - avirt; _0 = around + bround; _i = (_0 + cdycdy1); bvirt = (_i - _0); avirt = _i - bvirt; bround = cdycdy1 - bvirt; around = _0 - avirt; cc[1] = around + bround; cc3 = (_j + _i); bvirt = (cc3 - _j); avirt = cc3 - bvirt; bround = _i - bvirt; around = _j - avirt; cc[2] = around + bround;
                cc[3] = cc3;
            }

            if (adxtail != (0))
            {
                axtbclen = ScaleExpansionZeroelim(4, bc, adxtail, axtbc);
                temp16alen = ScaleExpansionZeroelim(axtbclen, axtbc, (2) * adx, temp16a);

                axtcclen = ScaleExpansionZeroelim(4, cc, adxtail, axtcc);
                temp16blen = ScaleExpansionZeroelim(axtcclen, axtcc, bdy, temp16b);

                axtbblen = ScaleExpansionZeroelim(4, bb, adxtail, axtbb);
                temp16clen = ScaleExpansionZeroelim(axtbblen, axtbb, -cdy, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (adytail != (0))
            {
                aytbclen = ScaleExpansionZeroelim(4, bc, adytail, aytbc);
                temp16alen = ScaleExpansionZeroelim(aytbclen, aytbc, (2) * ady, temp16a);

                aytbblen = ScaleExpansionZeroelim(4, bb, adytail, aytbb);
                temp16blen = ScaleExpansionZeroelim(aytbblen, aytbb, cdx, temp16b);

                aytcclen = ScaleExpansionZeroelim(4, cc, adytail, aytcc);
                temp16clen = ScaleExpansionZeroelim(aytcclen, aytcc, -bdx, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (bdxtail != (0))
            {
                bxtcalen = ScaleExpansionZeroelim(4, ca, bdxtail, bxtca);
                temp16alen = ScaleExpansionZeroelim(bxtcalen, bxtca, (2) * bdx, temp16a);

                bxtaalen = ScaleExpansionZeroelim(4, aa, bdxtail, bxtaa);
                temp16blen = ScaleExpansionZeroelim(bxtaalen, bxtaa, cdy, temp16b);

                bxtcclen = ScaleExpansionZeroelim(4, cc, bdxtail, bxtcc);
                temp16clen = ScaleExpansionZeroelim(bxtcclen, bxtcc, -ady, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (bdytail != (0))
            {
                bytcalen = ScaleExpansionZeroelim(4, ca, bdytail, bytca);
                temp16alen = ScaleExpansionZeroelim(bytcalen, bytca, (2) * bdy, temp16a);

                bytcclen = ScaleExpansionZeroelim(4, cc, bdytail, bytcc);
                temp16blen = ScaleExpansionZeroelim(bytcclen, bytcc, adx, temp16b);

                bytaalen = ScaleExpansionZeroelim(4, aa, bdytail, bytaa);
                temp16clen = ScaleExpansionZeroelim(bytaalen, bytaa, -cdx, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (cdxtail != (0))
            {
                cxtablen = ScaleExpansionZeroelim(4, ab, cdxtail, cxtab);
                temp16alen = ScaleExpansionZeroelim(cxtablen, cxtab, (2) * cdx, temp16a);

                cxtbblen = ScaleExpansionZeroelim(4, bb, cdxtail, cxtbb);
                temp16blen = ScaleExpansionZeroelim(cxtbblen, cxtbb, ady, temp16b);

                cxtaalen = ScaleExpansionZeroelim(4, aa, cdxtail, cxtaa);
                temp16clen = ScaleExpansionZeroelim(cxtaalen, cxtaa, -bdy, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (cdytail != (0))
            {
                cytablen = ScaleExpansionZeroelim(4, ab, cdytail, cytab);
                temp16alen = ScaleExpansionZeroelim(cytablen, cytab, (2) * cdy, temp16a);

                cytaalen = ScaleExpansionZeroelim(4, aa, cdytail, cytaa);
                temp16blen = ScaleExpansionZeroelim(cytaalen, cytaa, bdx, temp16b);

                cytbblen = ScaleExpansionZeroelim(4, bb, cdytail, cytbb);
                temp16clen = ScaleExpansionZeroelim(cytbblen, cytbb, -adx, temp16c);

                temp32alen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
                temp48len = FastExpansionSumZeroelim(temp16clen, temp16c, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }

            if (adxtail != (0) || adytail != (0))
            {
                if (bdxtail != (0) || bdytail != (0) || cdxtail != (0) || cdytail != (0))
                {
                    ti1 = (bdxtail * cdy); c = (errorBounds.splitter * bdxtail); abig = (c - bdxtail); ahi = c - abig; alo = bdxtail - ahi; c = (errorBounds.splitter * cdy); abig = (c - cdy); bhi = c - abig; blo = cdy - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (bdx * cdytail); c = (errorBounds.splitter * bdx); abig = (c - bdx); ahi = c - abig; alo = bdx - ahi; c = (errorBounds.splitter * cdytail); abig = (c - cdytail); bhi = c - abig; blo = cdytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; u[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
                    u[3] = u3;
                    negate = -bdy;
                    ti1 = (cdxtail * negate); c = (errorBounds.splitter * cdxtail); abig = (c - cdxtail); ahi = c - abig; alo = cdxtail - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    negate = -bdytail;
                    tj1 = (cdx * negate); c = (errorBounds.splitter * cdx); abig = (c - cdx); ahi = c - abig; alo = cdx - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; v[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; v[1] = around + bround; v3 = (_j + _i); bvirt = (v3 - _j); avirt = v3 - bvirt; bround = _i - bvirt; around = _j - avirt; v[2] = around + bround;
                    v[3] = v3;
                    bctlen = FastExpansionSumZeroelim(4, u, 4, v, bct);

                    ti1 = (bdxtail * cdytail); c = (errorBounds.splitter * bdxtail); abig = (c - bdxtail); ahi = c - abig; alo = bdxtail - ahi; c = (errorBounds.splitter * cdytail); abig = (c - cdytail); bhi = c - abig; blo = cdytail - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (cdxtail * bdytail); c = (errorBounds.splitter * cdxtail); abig = (c - cdxtail); ahi = c - abig; alo = cdxtail - ahi; c = (errorBounds.splitter * bdytail); abig = (c - bdytail); bhi = c - abig; blo = bdytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 - tj0); bvirt = (ti0 - _i); avirt = _i + bvirt; bround = bvirt - tj0; around = ti0 - avirt; bctt[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 - tj1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - tj1; around = _0 - avirt; bctt[1] = around + bround; bctt3 = (_j + _i); bvirt = (bctt3 - _j); avirt = bctt3 - bvirt; bround = _i - bvirt; around = _j - avirt; bctt[2] = around + bround;
                    bctt[3] = bctt3;
                    bcttlen = 4;
                }
                else
                {
                    bct[0] = (0);
                    bctlen = 1;
                    bctt[0] = (0);
                    bcttlen = 1;
                }

                if (adxtail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(axtbclen, axtbc, adxtail, temp16a);
                    axtbctlen = ScaleExpansionZeroelim(bctlen, bct, adxtail, axtbct);
                    temp32alen = ScaleExpansionZeroelim(axtbctlen, axtbct, (2) * adx, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (bdytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, cc, adxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, bdytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (cdytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, bb, -adxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, cdytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = ScaleExpansionZeroelim(axtbctlen, axtbct, adxtail, temp32a);
                    axtbcttlen = ScaleExpansionZeroelim(bcttlen, bctt, adxtail, axtbctt);
                    temp16alen = ScaleExpansionZeroelim(axtbcttlen, axtbctt, (2) * adx, temp16a);
                    temp16blen = ScaleExpansionZeroelim(axtbcttlen, axtbctt, adxtail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (adytail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(aytbclen, aytbc, adytail, temp16a);
                    aytbctlen = ScaleExpansionZeroelim(bctlen, bct, adytail, aytbct);
                    temp32alen = ScaleExpansionZeroelim(aytbctlen, aytbct, (2) * ady, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = ScaleExpansionZeroelim(aytbctlen, aytbct, adytail, temp32a);
                    aytbcttlen = ScaleExpansionZeroelim(bcttlen, bctt, adytail, aytbctt);
                    temp16alen = ScaleExpansionZeroelim(aytbcttlen, aytbctt, (2) * ady, temp16a);
                    temp16blen = ScaleExpansionZeroelim(aytbcttlen, aytbctt, adytail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
            }

            if (bdxtail != (0) || bdytail != (0))
            {
                if (cdxtail != (0) || cdytail != (0) || adxtail != (0) || adytail != (0))
                {
                    ti1 = (cdxtail * ady); c = (errorBounds.splitter * cdxtail); abig = (c - cdxtail); ahi = c - abig; alo = cdxtail - ahi; c = (errorBounds.splitter * ady); abig = (c - ady); bhi = c - abig; blo = ady - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (cdx * adytail); c = (errorBounds.splitter * cdx); abig = (c - cdx); ahi = c - abig; alo = cdx - ahi; c = (errorBounds.splitter * adytail); abig = (c - adytail); bhi = c - abig; blo = adytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; u[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
                    u[3] = u3;
                    negate = -cdy;
                    ti1 = (adxtail * negate); c = (errorBounds.splitter * adxtail); abig = (c - adxtail); ahi = c - abig; alo = adxtail - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    negate = -cdytail;
                    tj1 = (adx * negate); c = (errorBounds.splitter * adx); abig = (c - adx); ahi = c - abig; alo = adx - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; v[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; v[1] = around + bround; v3 = (_j + _i); bvirt = (v3 - _j); avirt = v3 - bvirt; bround = _i - bvirt; around = _j - avirt; v[2] = around + bround;
                    v[3] = v3;
                    catlen = FastExpansionSumZeroelim(4, u, 4, v, cat);

                    ti1 = (cdxtail * adytail); c = (errorBounds.splitter * cdxtail); abig = (c - cdxtail); ahi = c - abig; alo = cdxtail - ahi; c = (errorBounds.splitter * adytail); abig = (c - adytail); bhi = c - abig; blo = adytail - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (adxtail * cdytail); c = (errorBounds.splitter * adxtail); abig = (c - adxtail); ahi = c - abig; alo = adxtail - ahi; c = (errorBounds.splitter * cdytail); abig = (c - cdytail); bhi = c - abig; blo = cdytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 - tj0); bvirt = (ti0 - _i); avirt = _i + bvirt; bround = bvirt - tj0; around = ti0 - avirt; catt[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 - tj1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - tj1; around = _0 - avirt; catt[1] = around + bround; catt3 = (_j + _i); bvirt = (catt3 - _j); avirt = catt3 - bvirt; bround = _i - bvirt; around = _j - avirt; catt[2] = around + bround;
                    catt[3] = catt3;
                    cattlen = 4;
                }
                else
                {
                    cat[0] = (0);
                    catlen = 1;
                    catt[0] = (0);
                    cattlen = 1;
                }

                if (bdxtail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(bxtcalen, bxtca, bdxtail, temp16a);
                    bxtcatlen = ScaleExpansionZeroelim(catlen, cat, bdxtail, bxtcat);
                    temp32alen = ScaleExpansionZeroelim(bxtcatlen, bxtcat, (2) * bdx, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (cdytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, aa, bdxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, cdytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (adytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, cc, -bdxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, adytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = ScaleExpansionZeroelim(bxtcatlen, bxtcat, bdxtail, temp32a);
                    bxtcattlen = ScaleExpansionZeroelim(cattlen, catt, bdxtail, bxtcatt);
                    temp16alen = ScaleExpansionZeroelim(bxtcattlen, bxtcatt, (2) * bdx, temp16a);
                    temp16blen = ScaleExpansionZeroelim(bxtcattlen, bxtcatt, bdxtail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (bdytail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(bytcalen, bytca, bdytail, temp16a);
                    bytcatlen = ScaleExpansionZeroelim(catlen, cat, bdytail, bytcat);
                    temp32alen = ScaleExpansionZeroelim(bytcatlen, bytcat, (2) * bdy, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = ScaleExpansionZeroelim(bytcatlen, bytcat, bdytail, temp32a);
                    bytcattlen = ScaleExpansionZeroelim(cattlen, catt, bdytail, bytcatt);
                    temp16alen = ScaleExpansionZeroelim(bytcattlen, bytcatt, (2) * bdy, temp16a);
                    temp16blen = ScaleExpansionZeroelim(bytcattlen, bytcatt, bdytail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
            }

            if (cdxtail != (0) || cdytail != (0))
            {
                if (adxtail != (0) || adytail != (0) || bdxtail != (0) || bdytail != (0))
                {
                    ti1 = (adxtail * bdy); c = (errorBounds.splitter * adxtail); abig = (c - adxtail); ahi = c - abig; alo = adxtail - ahi; c = (errorBounds.splitter * bdy); abig = (c - bdy); bhi = c - abig; blo = bdy - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (adx * bdytail); c = (errorBounds.splitter * adx); abig = (c - adx); ahi = c - abig; alo = adx - ahi; c = (errorBounds.splitter * bdytail); abig = (c - bdytail); bhi = c - abig; blo = bdytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; u[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; u[1] = around + bround; u3 = (_j + _i); bvirt = (u3 - _j); avirt = u3 - bvirt; bround = _i - bvirt; around = _j - avirt; u[2] = around + bround;
                    u[3] = u3;
                    negate = -ady;
                    ti1 = (bdxtail * negate); c = (errorBounds.splitter * bdxtail); abig = (c - bdxtail); ahi = c - abig; alo = bdxtail - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    negate = -adytail;
                    tj1 = (bdx * negate); c = (errorBounds.splitter * bdx); abig = (c - bdx); ahi = c - abig; alo = bdx - ahi; c = (errorBounds.splitter * negate); abig = (c - negate); bhi = c - abig; blo = negate - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 + tj0); bvirt = (_i - ti0); avirt = _i - bvirt; bround = tj0 - bvirt; around = ti0 - avirt; v[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 + tj1); bvirt = (_i - _0); avirt = _i - bvirt; bround = tj1 - bvirt; around = _0 - avirt; v[1] = around + bround; v3 = (_j + _i); bvirt = (v3 - _j); avirt = v3 - bvirt; bround = _i - bvirt; around = _j - avirt; v[2] = around + bround;
                    v[3] = v3;
                    abtlen = FastExpansionSumZeroelim(4, u, 4, v, abt);

                    ti1 = (adxtail * bdytail); c = (errorBounds.splitter * adxtail); abig = (c - adxtail); ahi = c - abig; alo = adxtail - ahi; c = (errorBounds.splitter * bdytail); abig = (c - bdytail); bhi = c - abig; blo = bdytail - bhi; err1 = ti1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); ti0 = (alo * blo) - err3;
                    tj1 = (bdxtail * adytail); c = (errorBounds.splitter * bdxtail); abig = (c - bdxtail); ahi = c - abig; alo = bdxtail - ahi; c = (errorBounds.splitter * adytail); abig = (c - adytail); bhi = c - abig; blo = adytail - bhi; err1 = tj1 - (ahi * bhi); err2 = err1 - (alo * bhi); err3 = err2 - (ahi * blo); tj0 = (alo * blo) - err3;
                    _i = (ti0 - tj0); bvirt = (ti0 - _i); avirt = _i + bvirt; bround = bvirt - tj0; around = ti0 - avirt; abtt[0] = around + bround; _j = (ti1 + _i); bvirt = (_j - ti1); avirt = _j - bvirt; bround = _i - bvirt; around = ti1 - avirt; _0 = around + bround; _i = (_0 - tj1); bvirt = (_0 - _i); avirt = _i + bvirt; bround = bvirt - tj1; around = _0 - avirt; abtt[1] = around + bround; abtt3 = (_j + _i); bvirt = (abtt3 - _j); avirt = abtt3 - bvirt; bround = _i - bvirt; around = _j - avirt; abtt[2] = around + bround;
                    abtt[3] = abtt3;
                    abttlen = 4;
                }
                else
                {
                    abt[0] = (0);
                    abtlen = 1;
                    abtt[0] = (0);
                    abttlen = 1;
                }

                if (cdxtail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(cxtablen, cxtab, cdxtail, temp16a);
                    cxtabtlen = ScaleExpansionZeroelim(abtlen, abt, cdxtail, cxtabt);
                    temp32alen = ScaleExpansionZeroelim(cxtabtlen, cxtabt, (2) * cdx, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    if (adytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, bb, cdxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, adytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    if (bdytail != (0))
                    {
                        temp8len = ScaleExpansionZeroelim(4, aa, -cdxtail, temp8);
                        temp16alen = ScaleExpansionZeroelim(temp8len, temp8, bdytail, temp16a);
                        finlength = FastExpansionSumZeroelim(finlength, finnow, temp16alen, temp16a, finother);
                        finswap = finnow; finnow = finother; finother = finswap;
                    }

                    temp32alen = ScaleExpansionZeroelim(cxtabtlen, cxtabt, cdxtail, temp32a);
                    cxtabttlen = ScaleExpansionZeroelim(abttlen, abtt, cdxtail, cxtabtt);
                    temp16alen = ScaleExpansionZeroelim(cxtabttlen, cxtabtt, (2) * cdx, temp16a);
                    temp16blen = ScaleExpansionZeroelim(cxtabttlen, cxtabtt, cdxtail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                if (cdytail != (0))
                {
                    temp16alen = ScaleExpansionZeroelim(cytablen, cytab, cdytail, temp16a);
                    cytabtlen = ScaleExpansionZeroelim(abtlen, abt, cdytail, cytabt);
                    temp32alen = ScaleExpansionZeroelim(cytabtlen, cytabt, (2) * cdy, temp32a);
                    temp48len = FastExpansionSumZeroelim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp48len, temp48, finother);
                    finswap = finnow; finnow = finother; finother = finswap;

                    temp32alen = ScaleExpansionZeroelim(cytabtlen, cytabt, cdytail, temp32a);
                    cytabttlen = ScaleExpansionZeroelim(abttlen, abtt, cdytail, cytabtt);
                    temp16alen = ScaleExpansionZeroelim(cytabttlen, cytabtt, (2) * cdy, temp16a);
                    temp16blen = ScaleExpansionZeroelim(cytabttlen, cytabtt, cdytail, temp16b);
                    temp32blen = FastExpansionSumZeroelim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                    temp64len = FastExpansionSumZeroelim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                    finlength = FastExpansionSumZeroelim(finlength, finnow, temp64len, temp64, finother);
                    /* finswap = finnow; */
                    finnow = finother; /* finother = finswap; */
                }
            }

            return finnow[finlength - 1];
        }

        internal struct ErrorBounds
        {
            public Scalar splitter;
            public Scalar epsilon;
            public Scalar resulterrbound;
            public Scalar ccwerrboundA, ccwerrboundB, ccwerrboundC;
            public Scalar o3derrboundA, o3derrboundB, o3derrboundC;
            public Scalar iccerrboundA, iccerrboundB, iccerrboundC;
            public Scalar isperrboundA, isperrboundB, isperrboundC;
        }

        private struct DataView
        {
            private readonly Scalar[] _array;
            private readonly int _offset;

#if DEBUG
            private readonly int _count;
#endif

            public DataView(Scalar[] array, int offset, int count)
            {
                _array = array;
                _offset = offset;
#if DEBUG
                _count = count;
#endif
            }

            public Scalar this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
#if DEBUG
                    if (index < 0 || index >= _count)
                    {
                        throw new IndexOutOfRangeException();
                    }
#endif
                    return _array[_offset + index];
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set
                {
#if DEBUG
                    if (index < 0 || index >= _count)
                    {
                        throw new IndexOutOfRangeException();
                    }
#endif
                    _array[_offset + index] = value;
                }
            }
        }
    }

    public static class Math
    {
        public enum Orientation : byte
        {
            CW = 0, CCW = 1, Collinear = 2
        }

        public static Orientation Orient2d(Predicates pred, Vec2 a, Vec2 b, Vec2 c)
        {
            Scalar pa0 = a.x;
            Scalar pa1 = a.y;
            Scalar pb0 = b.x;
            Scalar pb1 = b.y;
            Scalar pc0 = c.x;
            Scalar pc1 = c.y;

            Scalar detleft = (pa0 - pc0) * (pb1 - pc1);
            Scalar detright = (pa1 - pc1) * (pb0 - pc0);
            Scalar det = detleft - detright;

            do
            {
                Scalar detsum;

                if (detleft > 0)
                {
                    if (detright <= 0)
                    {
                        break;
                    }
                    else
                    {
                        detsum = detleft + detright;
                    }
                }
                else if (detleft < 0)
                {
                    if (detright >= 0)
                    {
                        break;
                    }
                    else
                    {
                        detsum = -detleft - detright;
                    }
                }
                else
                {
                    break;
                }

                Scalar errbound = Predicates.errorBounds.ccwerrboundA * detsum;
                if (Predicates.Absolute(det) < errbound)
                {
                    det = pred.Orient2dAdapt(a.x, a.y, b.x, b.y, c.x, c.y, detsum);
                }
            }
            while (false);

            if (det < 0)
            {
                return Orientation.CW;
            }
            else if (det > 0)
            {
                return Orientation.CCW;
            }
            else
            {
                return Orientation.Collinear;
            }
        }

        public enum CircleLocation : byte
        {
            Inside = 0, Outside = 1, Cocircular = 2
        }

        public static CircleLocation Incircle(Predicates pred, Vec2 a, Vec2 b, Vec2 c, Vec2 d)
        {
#if false
//# ifndef NDEBUG
//            // The points pa, pb, and pc must be in counterclockwise order, or the sign of the result will be reversed
//            detail::detriaAssert(math::orient2d<Robust, Vec2>(a, b, c) == math::Orientation::CCW);
//#endif
#endif

            Scalar pa0 = a.x;
            Scalar pa1 = a.y;
            Scalar pb0 = b.x;
            Scalar pb1 = b.y;
            Scalar pc0 = c.x;
            Scalar pc1 = c.y;
            Scalar pd0 = d.x;
            Scalar pd1 = d.y;

            Scalar adx = pa0 - pd0;
            Scalar bdx = pb0 - pd0;
            Scalar cdx = pc0 - pd0;
            Scalar ady = pa1 - pd1;
            Scalar bdy = pb1 - pd1;
            Scalar cdy = pc1 - pd1;

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

            Scalar permanent =
                (Predicates.Absolute(bdxcdy) + Predicates.Absolute(cdxbdy)) * alift +
                (Predicates.Absolute(cdxady) + Predicates.Absolute(adxcdy)) * blift +
                (Predicates.Absolute(adxbdy) + Predicates.Absolute(bdxady)) * clift;

            Scalar errbound = Predicates.errorBounds.iccerrboundA * permanent;
            if (Predicates.Absolute(det) <= errbound)
            {
                det = pred.IncircleAdapt(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y, permanent);
            }

            if (det > 0)
            {
                return CircleLocation.Inside;
            }
            else if (det < 0)
            {
                return CircleLocation.Outside;
            }
            else
            {
                return CircleLocation.Cocircular;
            }
        }
    }

    internal static class Detail
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void ThrowAssertionFailedError()
        {
            throw new Exception("Assertion failed");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Reserve<T>(List<T> collection, int capacity)
        {
            if (collection.Capacity < capacity)
            {
                collection.Capacity = capacity;
            }
        }

        public static void ClearAndResize<T>(List<T> collection, int size, T value)
        {
            collection.Clear();

            for (int i = 0; i < size; ++i)
            {
                collection.Add(value);
            }
        }
    }

    internal static class Constants
    {
        public const Idx NullIndex = unchecked((Idx)(-1));
    }

    public struct NullableIdx : IEquatable<NullableIdx>
    {
        private readonly Idx value;

        public bool HasValue
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return value != Constants.NullIndex;
            }
        }

        public Idx Value
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
#if DEBUG
                if (!HasValue)
                {
                    Detail.ThrowAssertionFailedError();
                }
#endif

                return value;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public NullableIdx(Idx value)
        {
            this.value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator NullableIdx(Idx idx)
        {
            return new NullableIdx(idx);
        }

        public static NullableIdx Null
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return new NullableIdx(Constants.NullIndex);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(NullableIdx a, NullableIdx b)
        {
            return a.Equals(b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(NullableIdx a, NullableIdx b)
        {
            return !a.Equals(b);
        }

        public override bool Equals(object obj)
        {
            if (!(obj is NullableIdx))
            {
                return false;
            }

            NullableIdx other = (NullableIdx)obj;
            return Equals(other);
        }

        public override int GetHashCode()
        {
            return value.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(NullableIdx other)
        {
            return value == other.value;
        }

        public override string ToString()
        {
            return HasValue ? Value.ToString() : "Empty";
        }
    }

    public struct VertexIndex : IEquatable<VertexIndex>
    {
        public NullableIdx index;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VertexIndex(NullableIdx index)
        {
            this.index = index;
        }

        public static VertexIndex Null
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return new VertexIndex(NullableIdx.Null);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(VertexIndex a, VertexIndex b)
        {
            return a.Equals(b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(VertexIndex a, VertexIndex b)
        {
            return !a.Equals(b);
        }

        public bool IsValid
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return index.HasValue;
            }
        }

        public override bool Equals(object obj)
        {
            if (!(obj is VertexIndex))
            {
                return false;
            }

            VertexIndex other = (VertexIndex)obj;
            return Equals(other);
        }

        public override int GetHashCode()
        {
            return index.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(VertexIndex other)
        {
            return index == other.index;
        }

        public override string ToString()
        {
            return index.ToString();
        }
    }

    public struct HalfEdgeIndex : IEquatable<HalfEdgeIndex>
    {
        public NullableIdx index;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HalfEdgeIndex(NullableIdx index)
        {
            this.index = index;
        }

        public static HalfEdgeIndex Null
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return new HalfEdgeIndex(NullableIdx.Null);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(HalfEdgeIndex a, HalfEdgeIndex b)
        {
            return a.Equals(b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(HalfEdgeIndex a, HalfEdgeIndex b)
        {
            return !a.Equals(b);
        }

        public bool IsValid
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return index.HasValue;
            }
        }

        public override bool Equals(object obj)
        {
            if (!(obj is HalfEdgeIndex))
            {
                return false;
            }

            HalfEdgeIndex other = (HalfEdgeIndex)obj;
            return Equals(other);
        }

        public override int GetHashCode()
        {
            return index.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(HalfEdgeIndex other)
        {
            return index == other.index;
        }

        public override string ToString()
        {
            return index.ToString();
        }
    }

    public enum EdgeType : byte
    {
        NotConstrained = 0, ManuallyConstrained, AutoDetect, Outline, Hole, MAX_EDGE_TYPE
    }

    public struct EdgeData
    {
        // An edge can be either:
        // - Not constrained
        // - Manually constrained
        // - Auto detected: will decide if it's an outline or a hole, depending on where it is
        // - Part of an outline or a hole; in this case, we need to store its index
        // Also store if the edge is delaunay and if it's boundary

        public DataType dataType;
        public Flags flags;

        // Only if dataType == OutlineOrHole
        public EdgeType type;
        public Idx polylineIndex;

        public bool IsDelaunay
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return HasFlag(Flags.Delaunay);
            }
        }

        public bool IsBoundary
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return HasFlag(Flags.Boundary);
            }
        }

        public bool IsConstrained
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return dataType != DataType.NotConstrained;
            }
        }

        public NullableIdx OutlineOrHoleIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return dataType == DataType.OutlineOrHole ? polylineIndex : NullableIdx.Null;
            }
        }

        public EdgeType EdgeType
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                switch (dataType)
                {
                    case DataType.NotConstrained:
                        return EdgeType.NotConstrained;

                    case DataType.ManuallyConstrained:
                        return EdgeType.ManuallyConstrained;

                    case DataType.OutlineOrHole:
                        return type;

                    default:
                        // Shouldn't happen
                        return EdgeType.NotConstrained;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasFlag(Flags flag)
        {
            return (flags & flag) != Flags.None;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetFlag(Flags flag, bool value)
        {
            if (value)
            {
                flags |= flag;
            }
            else
            {
                flags &= ~flag;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetDelaunay(bool value)
        {
            SetFlag(Flags.Delaunay, value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetBoundary(bool value)
        {
            SetFlag(Flags.Boundary, value);
        }

        public enum DataType : byte
        {
            NotConstrained = 0,
            ManuallyConstrained = 1,
            OutlineOrHole = 2,
        }

        [Flags]
        public enum Flags : byte
        {
            None = 0x0,
            Delaunay = 0x1,
            Boundary = 0x2,
        }
    }

    public class Topology
    {
        // Using half-edges for storing vertex relations
        // Each vertex has outgoing half-edges (only the first edge is stored)
        // The rest of the edges going around a given vertex are stored in a doubly-linked list
        // The half-edges are also paired with an opposite edge, which references the other vertex
        // The triangles are not stored, since the topology is constructed in a way that the edges are not intersecting,
        // and the triangles can always be calculated from the stored data (the triangles are calculated at the final step of the triangulation)
        // For example:
        //
        //            v1  e13     e31  v3
        //             *------   ------*
        //            / \             /
        //       e10 /   \ e12       / e32
        //          /     \         /
        //
        //        /         \     /
        //   e01 /       e21 \   / e23
        //      /             \ /
        //  v0 *------   ------* v2
        //        e02     e20
        //
        // v0.firstEdge == e01
        // getOpposite(e01) == e10
        // getOpposite(e01).vertex == v1
        // e01.nextEdge == e02
        // e01.prevEdge == e02
        // e20.nextEdge == e21
        // e21.prevEdge == e23

        private readonly List<Vertex> _vertices = new List<Vertex>();
        private readonly List<HalfEdge> _edges = new List<HalfEdge>();

        public struct Vertex
        {
            public HalfEdgeIndex firstEdge;

            public static Vertex Null
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return new Vertex
                    {
                        firstEdge = HalfEdgeIndex.Null,
                    };
                }
            }
        }

        public struct HalfEdge
        {
            public VertexIndex vertex;
            public HalfEdgeIndex prevEdge;
            public HalfEdgeIndex nextEdge;

            public EdgeData data;

            public static HalfEdge Null
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return new HalfEdge
                    {
                        vertex = VertexIndex.Null,
                        prevEdge = HalfEdgeIndex.Null,
                        nextEdge = HalfEdgeIndex.Null,
                    };
                }
            }
        }

        public int VertexCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return _vertices.Count;
            }
        }

        public int HalfEdgeCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return _edges.Count;
            }
        }

        public void Clear()
        {
            _vertices.Clear();
            _edges.Clear();
        }

        public void ReserveVertices(int capacity)
        {
            Detail.Reserve(_vertices, capacity);
        }

        public void ReserveHalfEdges(int capacity)
        {
            Detail.Reserve(_edges, capacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VertexIndex CreateVertex()
        {
            Idx idx = (Idx)_vertices.Count;
            _vertices.Add(Vertex.Null);
            return new VertexIndex(idx);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vertex GetVertex(VertexIndex idx)
        {
            return _vertices[(int)idx.index.Value];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetVertex(VertexIndex idx, Vertex vertex)
        {
            _vertices[(int)idx.index.Value] = vertex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HalfEdge GetEdge(HalfEdgeIndex idx)
        {
            return _edges[(int)idx.index.Value];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetEdge(HalfEdgeIndex idx, HalfEdge edge)
        {
            _edges[(int)idx.index.Value] = edge;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public EdgeData GetEdgeData(HalfEdgeIndex idx)
        {
            return GetEdge(idx).data;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetEdgeData(HalfEdgeIndex idx, EdgeData data)
        {
            HalfEdge edge = GetEdge(idx);
            edge.data = data;
            SetEdge(idx, edge);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static HalfEdgeIndex GetOpposite(HalfEdgeIndex idx)
        {
            return new HalfEdgeIndex(idx.index.Value ^ 1);
        }

        public HalfEdgeIndex GetEdgeBetween(VertexIndex a, VertexIndex b)
        {
            HalfEdgeIndex existingEdge = HalfEdgeIndex.Null;

            ForEachEdgeOfVertex(a, (HalfEdgeIndex edge) =>
            {
                HalfEdgeIndex oppositeEdgeIndex = GetOpposite(edge);
                if (GetEdge(oppositeEdgeIndex).vertex == b)
                {
                    existingEdge = edge;
                    return false;
                }

                return true;
            });

            return existingEdge;
        }

        public HalfEdgeIndex CreateNewEdge(VertexIndex a, VertexIndex b, HalfEdgeIndex addAAfterThisEdge, HalfEdgeIndex addBAfterThisEdge, HalfEdgeIndex reusedEdgeIndex)
        {
            // If the given edges are valid, the edge will be inserted after the given edges
            // Otherwise the new edge will be added as the first edge of the given vertex
            // This function should only be called if there is no edge between the given vertices

            // Half edges are always created in pairs, which means that the opposite index can always be retrieved by xor-ing the edge index with 1

            HalfEdge AddHalfEdge(HalfEdgeIndex newEdgeIndex, VertexIndex vertex, HalfEdgeIndex afterEdge)
            {
                HalfEdge newEdge;

                newEdge.data = new EdgeData
                {
                    polylineIndex = Constants.NullIndex,
                };
                newEdge.vertex = vertex;

                if (afterEdge.IsValid)
                {
                    // Valid index, insert between the edges

                    HalfEdge originalEdge = GetEdge(afterEdge);
                    HalfEdgeIndex beforeEdge = originalEdge.nextEdge;
                    HalfEdge originalNextEdge = GetEdge(beforeEdge);

                    newEdge.prevEdge = afterEdge;
                    newEdge.nextEdge = beforeEdge;

                    if (afterEdge == beforeEdge)
                    {
                        originalEdge.nextEdge = newEdgeIndex;
                        originalEdge.prevEdge = newEdgeIndex;
                        SetEdge(afterEdge, originalEdge);
                    }
                    else
                    {
                        originalEdge.nextEdge = newEdgeIndex;
                        originalNextEdge.prevEdge = newEdgeIndex;

                        SetEdge(afterEdge, originalEdge);
                        SetEdge(beforeEdge, originalNextEdge);
                    }
                }
                else
                {
                    // Null index, set as first edge
                    Vertex v = GetVertex(vertex);
                    v.firstEdge = newEdgeIndex;
                    SetVertex(vertex, v);

                    // Set references to self
                    newEdge.nextEdge = newEdgeIndex;
                    newEdge.prevEdge = newEdgeIndex;
                }

                return newEdge;
            }

            bool reusedEdge = reusedEdgeIndex.IsValid;

            HalfEdgeIndex newEdgeIndexA;
            HalfEdgeIndex newEdgeIndexB;

            if (reusedEdge)
            {
                newEdgeIndexA = reusedEdgeIndex;
                newEdgeIndexB = GetOpposite(reusedEdgeIndex);
            }
            else
            {
                newEdgeIndexA = new HalfEdgeIndex((Idx)_edges.Count);
                newEdgeIndexB = new HalfEdgeIndex((Idx)(_edges.Count + 1));
            }

            HalfEdge newEdgeA = AddHalfEdge(newEdgeIndexA, a, addAAfterThisEdge);
            HalfEdge newEdgeB = AddHalfEdge(newEdgeIndexB, b, addBAfterThisEdge);

            if (reusedEdge)
            {
                _edges[(int)newEdgeIndexA.index.Value] = newEdgeA;
                _edges[(int)newEdgeIndexB.index.Value] = newEdgeB;
            }
            else
            {
                _edges.Add(newEdgeA);
                _edges.Add(newEdgeB);
            }

            return newEdgeIndexA;
        }

        public void ForEachEdgeOfVertex(VertexIndex idx, Func<HalfEdgeIndex, bool> callback)
        {
            Vertex v = GetVertex(idx);
            if (!v.firstEdge.IsValid)
            {
                return;
            }

            HalfEdgeIndex firstEdge = v.firstEdge;
            HalfEdgeIndex currentEdgeIndex = firstEdge;

            do
            {
                HalfEdge edge = GetEdge(currentEdgeIndex);

                if (!callback(currentEdgeIndex))
                {
                    return;
                }

                currentEdgeIndex = edge.nextEdge;
            }
            while (currentEdgeIndex != firstEdge);
        }

        public void UnlinkEdge(HalfEdgeIndex edgeIndex)
        {
            // Assuming the vertices of this edge have at least one other edge
            // The first edge of the vertices will not be set correctly otherwise

            HalfEdge edge = GetEdge(edgeIndex);
            HalfEdge opposite = GetEdge(GetOpposite(edgeIndex));

            Vertex v0 = GetVertex(edge.vertex);
            Vertex v1 = GetVertex(opposite.vertex);
            v0.firstEdge = edge.nextEdge;
            v1.firstEdge = opposite.nextEdge;
            SetVertex(edge.vertex, v0);
            SetVertex(opposite.vertex, v1);

            HalfEdge tmpEdge;

            tmpEdge = GetEdge(edge.nextEdge);
            tmpEdge.prevEdge = edge.prevEdge;
            SetEdge(edge.nextEdge, tmpEdge);

            tmpEdge = GetEdge(edge.prevEdge);
            tmpEdge.nextEdge = edge.nextEdge;
            SetEdge(edge.prevEdge, tmpEdge);


            tmpEdge = GetEdge(opposite.nextEdge);
            tmpEdge.prevEdge = opposite.prevEdge;
            SetEdge(opposite.nextEdge, tmpEdge);

            tmpEdge = GetEdge(opposite.prevEdge);
            tmpEdge.nextEdge = opposite.nextEdge;
            SetEdge(opposite.prevEdge, tmpEdge);
        }

        public void FlipEdge(HalfEdgeIndex edgeIndex)
        {
            /*        0                 0
                *        *                 *
                *       /|\               / \
                *      / | \             /   \
                *   3 *  |  * 1  -->  3 *-----* 1
                *      \ | /             \   /
                *       \|/               \ /
                *        *                 *
                *        2                 2
                */

            // Setup all references
            HalfEdge edge = GetEdge(edgeIndex);
            HalfEdgeIndex oppositeIndex = GetOpposite(edgeIndex);
            HalfEdge opposite = GetEdge(oppositeIndex);

            HalfEdgeIndex e01i = edge.prevEdge;
            HalfEdgeIndex e03i = edge.nextEdge;
            HalfEdgeIndex e21i = opposite.nextEdge;
            HalfEdgeIndex e23i = opposite.prevEdge;
            HalfEdge e01 = GetEdge(e01i);
            HalfEdge e03 = GetEdge(e03i);
            HalfEdge e21 = GetEdge(e21i);
            HalfEdge e23 = GetEdge(e23i);

            HalfEdgeIndex e10i = GetOpposite(e01i);
            HalfEdgeIndex e30i = GetOpposite(e03i);
            HalfEdgeIndex e12i = GetOpposite(e21i);
            HalfEdgeIndex e32i = GetOpposite(e23i);
            HalfEdge e10 = GetEdge(e10i);
            HalfEdge e30 = GetEdge(e30i);
            HalfEdge e12 = GetEdge(e12i);
            HalfEdge e32 = GetEdge(e32i);

            VertexIndex originalEdgeVertexIndex = edge.vertex;
            VertexIndex originalOppositeVertexIndex = opposite.vertex;
            Vertex v0 = GetVertex(edge.vertex);
            Vertex v2 = GetVertex(opposite.vertex);

            // Unlink the edge
            e01.nextEdge = e03i;
            e03.prevEdge = e01i;
            e23.nextEdge = e21i;
            e21.prevEdge = e23i;

            // Technically we only need to change firstEdge if it's the flipped edge, but we can avoid a branch if we always set it
            v0.firstEdge = e01i;
            v2.firstEdge = e23i;

            // Re-link edge as flipped
            edge.prevEdge = e12i;
            edge.nextEdge = e10i;
            e10.prevEdge = edgeIndex;
            e12.nextEdge = edgeIndex;

            opposite.prevEdge = e30i;
            opposite.nextEdge = e32i;
            e32.prevEdge = oppositeIndex;
            e30.nextEdge = oppositeIndex;

            edge.vertex = e10.vertex;
            opposite.vertex = e32.vertex;

            // Write values back
            SetEdge(edgeIndex, edge);
            SetEdge(oppositeIndex, opposite);

            SetEdge(e01i, e01);
            SetEdge(e03i, e03);
            SetEdge(e21i, e21);
            SetEdge(e23i, e23);

            SetEdge(e10i, e10);
            SetEdge(e30i, e30);
            SetEdge(e12i, e12);
            SetEdge(e32i, e32);

            SetVertex(originalEdgeVertexIndex, v0);
            SetVertex(originalOppositeVertexIndex, v2);
        }
    }

    public enum TriangleLocation : byte
    {
        // Inside an outline
        Interior = 0,

        // Part of a hole
        Hole = 1,

        // Not part of any outlines or holes, only part of the initial triangulation
        ConvexHull = 2,
    }

    [Flags]
    public enum TriangleLocationMask : byte
    {
        Interior = 1 << TriangleLocation.Interior,
        Hole = 1 << TriangleLocation.Hole,
        ConvexHull = 1 << TriangleLocation.ConvexHull,
        All = Interior | Hole | ConvexHull,
    }

    public enum TriangulationErrorType
    {
        // Triangulation was successful
        NoError,

        // The triangulation object was created, but no triangulation was performed yet
        TriangulationNotStarted,

        // Less than three points were added to the triangulation
        LessThanThreePoints,


        // Errors of the initial triangulation phase

        // The input points contained a NaN or infinite value
        NonFinitePositionFound,

        // The list of input points contained duplicates
        DuplicatePointsFound,

        // All of the input points were collinear, so no valid triangles could be created
        AllPointsAreCollinear,


        // Errors of constrained edge creation

        // A polyline (outline or hole) contained less than 3 points
        PolylineTooShort,

        // An index in a polyline was out-of-bounds (idx < 0 or idx >= number of point)
        PolylineIndexOutOfBounds,

        // Two consecutive points in a polyline were the same
        PolylineDuplicateConsecutivePoints,

        // An edge was part of both an outline and a hole
        EdgeWithDifferentConstrainedTypes,

        // A point was exactly on a constrained edge
        PointOnConstrainedEdge,

        // Two constrained edges were intersecting
        ConstrainedEdgeIntersection,


        // Errors of triangle classification

        // Found a hole which was not inside any outlines
        HoleNotInsideOutline,

        // An outline was directly inside another outline, or a hole was directly inside another hole
        StackedPolylines,


        // A condition that should always be true was false, this error indicates a bug in the code
        AssertionFailed
    }

    public class Triangulation
    {
        #region Public API

        /// <summary>
        /// Enables checking for invalid (NaN, infinity) values in the list of input points.
        /// Can be disabled if you can ensure that all of the point values are valid.
        /// </summary>
        public bool EnableNaNChecks { get; set; } = true;

        /// <summary>
        /// Returns information about why the triangulation failed.
        /// </summary>
        public ITriangulationError Error
        {
            get
            {
                return _error;
            }
        }

        /// <summary>
        /// Returns the number of all triangles created during the triangulation, can be used e.g. to reserve memory for the results.
        /// Note that this number is usually greater than the number of "interesting" triangles,
        /// since it contains the interior, hole, and convex hull triangles as well.
        /// </summary>
        public int MaxNumTriangles
        {
            get
            {
                return _resultTriangles.Count;
            }
        }

        /// <summary>
        /// Returns the half-edge data structure created during the triangulation.
        /// There is usually no need to access this data, this is mostly used for testing.
        /// </summary>
        public Topology Topology
        {
            get
            {
                return _topology;
            }
        }

        /// <summary>
        /// Set all points which will be used for the triangulation.
        /// The result triangles will be indices of these points.
        /// If a point should be part of an outline, then use the <see cref="AddOutline"/> method.
        /// If a point should be part of a hole, then use <see cref="AddHole"/>.
        /// The rest of the points will be steiner points.
        /// </summary>
        /// <param name="points">List of the points to be used for the triangulation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetPoints(IReadOnlyList<Vec2> points)
        {
            _points = points;
        }

        /// <see cref="AddOutline"/>, <see cref="AddHole"/>, and <see cref="AddPolylineAutoDetectType"/> return the polyline's index,
        /// which can be used to get its parent, using the <see cref="TryGetParentPolylineIndex"/> method.

        /// <summary>
        /// Add an outline - regions surrounded by outlines are "solid", and will be part of the "inside" triangles.
        /// </summary>
        /// <param name="outline">List of point indices for this outline.</param>
        public Idx AddOutline(IReadOnlyList<Idx> outline)
        {
            Idx id = (Idx)_polylines.Count;

            _polylines.Add(new PolylineData
            {
                pointIndices = outline,
                type = EdgeType.Outline,
            });

            return id;
        }

        /// <summary>
        /// Add a hole - holes will be subtracted from the final "solid", and will be part of the "outside" triangles.
        /// </summary>
        /// <param name="hole">List of point indices for this hole.</param>
        public Idx AddHole(IReadOnlyList<Idx> hole)
        {
            Idx id = (Idx)_polylines.Count;

            _polylines.Add(new PolylineData
            {
                pointIndices = hole,
                type = EdgeType.Hole,
            });

            return id;
        }

        /// <summary>
        /// Add a polyline, and automatically decide if it's an outline or a hole.
        /// </summary>
        /// <param name="hole">List of point indices for this polyline.</param>
        public Idx AddPolylineAutoDetectType(IReadOnlyList<Idx> polyline)
        {
            Idx id = (Idx)_polylines.Count;

            _polylines.Add(new PolylineData
            {
                pointIndices = polyline,
                type = EdgeType.AutoDetect,
            });

            return id;
        }


        /// <summary>
        /// Set a single constrained edge, which will be part of the final triangulation.
        /// </summary>
        /// <param name="vertexIndexA">First vertex of the edge.</param>
        /// <param name="vertexIndexB">Second vertex of the edge.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetConstrainedEdge(Idx vertexIndexA, Idx vertexIndexB)
        {
            _manuallyConstrainedEdges.Add(new Edge { x = vertexIndexA, y = vertexIndexB });
        }

        /// <summary>
        /// Perform the triangulation.
        /// </summary>
        /// <param name="delaunay">If true, then the result triangles will satisfy the delaunay condition.</param>
        /// <returns>True if the triangulation succeeded, false if failed.</returns>
        /// <remarks>
        /// If the triangulation failed, the <see cref="Error"/> property can be used to retrieve more information about the error.
        /// No exception is thrown on error, make sure to check the return value of this function.
        /// </remarks>
        public bool Triangulate(bool delaunay)
        {
            ClearInternalData();

            Detail.Reserve(_constrainedEdgeVerticesCW, InitialCapacity);
            Detail.Reserve(_constrainedEdgeVerticesCCW, InitialCapacity);

            if (!TriangulateInternal(delaunay))
            {
                // If failed, then clear topology and convex hull points, so no invalid triangulation is returned
                _topology.Clear();
                _convexHullInitialVertex = NullVertex;
                _resultTriangles.Clear();
                return false;
            }

            _error = new TE_NoError();
            return true;
        }

        /// <summary>
        /// Enumerate every interior triangle of the triangulation.
        /// </summary>
        /// <param name="cwTriangles">If true, then the triangles will be in clockwise order.</param>
        public IEnumerable<Triangle> EnumerateTriangles(bool cwTriangles = true)
        {
            return EnumerateAllTrianglesWithData(cwTriangles,
                data => GetTriangleLocation(data.data) == TriangleLocation.Interior)
                .Select(data => data.triangle);
        }

        /// <summary>
        /// Enumerate every hole triangle of the triangulation.
        /// </summary>
        /// <param name="cwTriangles">If true, then the triangles will be in clockwise order.</param>
        public IEnumerable<Triangle> EnumerateHoleTriangles(bool cwTriangles = true)
        {
            return EnumerateAllTrianglesWithData(cwTriangles,
                data => GetTriangleLocation(data.data) == TriangleLocation.Hole)
                .Select(data => data.triangle);
        }

        /// <summary>
        /// Enumerate every single triangle of the triangulation, even convex hull triangles.
        /// </summary>
        /// <param name="cwTriangles">If true, then the triangles will be in clockwise order.</param>
        public IEnumerable<Triangle> EnumerateAllTriangles(bool cwTriangles)
        {
            return EnumerateAllTrianglesWithData(cwTriangles, null)
                .Select(data => data.triangle);
        }

        /// <summary>
        /// Enumerate every triangle of a given location/locations.
        /// Locations can be combined, to include e.g. both interior and hole triangles.
        /// </summary>
        /// <param name="locationMask">A bitmask value, which indicates the locations of the triangles that should be included in the result.</param>
        /// <param name="cwTriangles">If true, then the triangles will be in clockwise order.</param>
        public IEnumerable<Triangle> EnumerateTrianglesOfLocation(TriangleLocationMask locationMask, bool cwTriangles = true)
        {
            return EnumerateAllTrianglesWithData(cwTriangles,
                data =>
                {
                    TriangleLocationMask currentMask = (TriangleLocationMask)(1 << (int)GetTriangleLocation(data.data));
                    return (currentMask & locationMask) != 0;
                })
                .Select(data => data.triangle);
        }

        /// <summary>
        /// Enumerate the vertices (vertex indices) in the convex hull.
        /// The vertices are in clockwise order.
        /// </summary>
        public IEnumerable<Idx> EnumerateConvexHullVertices()
        {
            if (!_convexHullInitialVertex.IsValid)
            {
                yield break;
            }

            // Find boundary edge of the convex hull vertex
            HalfEdgeIndex startingEdge = Topology.GetOpposite(GetBoundaryEdgeOfVertex(_convexHullInitialVertex));
            if (!startingEdge.IsValid)
            {
                yield break;
            }

            HalfEdgeIndex edge = startingEdge;
            do
            {
                Topology.HalfEdge halfEdge = _topology.GetEdge(edge);
                yield return halfEdge.vertex.index.Value;
                HalfEdgeIndex opposite = Topology.GetOpposite(edge);
                edge = _topology.GetEdge(opposite).nextEdge;
            }
            while (edge != startingEdge);
        }

        /// <summary>
        /// Attempts to retrieve the parent index of the given polyline (that directly contains this polyline).
        /// </summary>
        /// <param name="polylineIndex">Index of the polyline for which the parent should be retrieved.</param>
        /// <param name="parentPolylineIndex">The result parent polyline index.</param>
        /// <returns>True if the given polyline has a parent, false if the polyline is top-level (or the polyline with the given index doesn't exist)</returns>
        public bool TryGetParentPolylineIndex(Idx polylineIndex, out Idx parentPolylineIndex)
        {
            if ((int)polylineIndex >= 0 && (int)polylineIndex < _parentPolylines.Count)
            {
                NullableIdx parentIndexNullable = _parentPolylines[(int)polylineIndex];
                if (parentIndexNullable.HasValue)
                {
                    parentPolylineIndex = parentIndexNullable.Value;
                    return true;
                }
            }

            parentPolylineIndex = Constants.NullIndex;
            return false;
        }

        /// <summary>
        /// Clears all data of the triangulation, allowing the triangulation object to be reused.
        /// </summary>
        public void Clear()
        {
            _points = null;
            _polylines.Clear();
            _manuallyConstrainedEdges.Clear();

            ClearInternalData();
        }

        /// <summary>
        /// Generates a text representation of the triangulation input, which can be used for debugging, or for reporting an issue.
        /// </summary>
        public string GenerateDebugFile()
        {
            // File structure:
            // numVertices numPolylines numManuallyConstrainedEdges
            // for numPolylines: numVerticesInPolyline
            // for numPolylines: polylineType (0 - outline, 1 - hole, 2 - auto detect)
            // list of vertices (x, y)
            // list of vertices in polylines (index)
            // list of manually constrained edges (index, index)

            if (_points == null)
            {
                return null;
            }

            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            int numPoints = _points.Count;
            int numPolylines = _polylines.Count;
            int numManuallyConstrainedEdges = _manuallyConstrainedEdges.Count;

            sb.Append(numPoints);
            sb.Append(' ');
            sb.Append(numPolylines);
            sb.Append(' ');
            sb.Append(numManuallyConstrainedEdges);
            sb.AppendLine();

            for (int i = 0; i < _polylines.Count; ++i)
            {
                sb.Append(_polylines[i].pointIndices.Count);
                sb.Append(' ');
            }
            sb.AppendLine();

            for (int i = 0; i < _polylines.Count; ++i)
            {
                switch (_polylines[i].type)
                {
                    case EdgeType.Outline:
                        sb.Append('0');
                        break;
                    case EdgeType.Hole:
                        sb.Append('1');
                        break;
                    case EdgeType.AutoDetect:
                        sb.Append('2');
                        break;

                    default:
                        break;
                }

                sb.Append(' ');
            }
            sb.AppendLine();

            for (int i = 0; i < _points.Count; ++i)
            {
                sb.Append(_points[i].x);
                sb.Append(' ');
                sb.Append(_points[i].y);
                sb.AppendLine();
            }

            for (int i = 0; i < _polylines.Count; ++i)
            {
                PolylineData data = _polylines[i];
                int numPointIndices = data.pointIndices.Count;
                for (int j = 0; j < numPointIndices; ++j)
                {
                    sb.Append(data.pointIndices[j]);
                    sb.AppendLine();
                }
            }

            for (int i = 0; i < _manuallyConstrainedEdges.Count; ++i)
            {
                sb.Append(_manuallyConstrainedEdges[i].x);
                sb.Append(' ');
                sb.Append(_manuallyConstrainedEdges[i].y);
                sb.AppendLine();
            }

            return sb.ToString();
        }

        #endregion

        #region Triangulation error types

        public interface ITriangulationError
        {
            TriangulationErrorType ErrorType { get; }
            string ErrorMessage { get; }
        }

        private struct TE_NoError : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.NoError;

            public string ErrorMessage => "The triangulation was successful";
        }

        private struct TE_NotStarted : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.TriangulationNotStarted;

            public string ErrorMessage => "The triangulation was not performed yet";
        }

        private struct TE_LessThanThreePoints : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.LessThanThreePoints;

            public string ErrorMessage => "At least 3 input points are required to perform the triangulation";
        }

        private struct TE_NonFinitePositionFound : ITriangulationError
        {
            private readonly Idx index;
            private readonly Scalar value;

            public TE_NonFinitePositionFound(Idx index, Scalar value)
            {
                this.index = index;
                this.value = value;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.NonFinitePositionFound;

            public string ErrorMessage => $"{(Scalar.IsNaN(value) ? "NaN" : "Infinite")} value found at point index {index}";
        }

        private struct TE_DuplicatePointsFound : ITriangulationError
        {
            private readonly Idx idx0;
            private readonly Idx idx1;
            private readonly Scalar positionX;
            private readonly Scalar positionY;

            public TE_DuplicatePointsFound(Idx idx0, Idx idx1, Scalar positionX, Scalar positionY)
            {
                this.idx0 = idx0;
                this.idx1 = idx1;
                this.positionX = positionX;
                this.positionY = positionY;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.DuplicatePointsFound;

            public string ErrorMessage => $"Multiple input points had the same position ({positionX}, {positionY}), at index {idx0} and at index {idx1}";
        }

        private struct TE_AllPointsAreCollinear : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.AllPointsAreCollinear;

            public string ErrorMessage => "All input points were collinear";
        }

        private struct TE_PolylineTooShort : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.PolylineTooShort;

            public string ErrorMessage =>
                // Note: we don't really have an index to return, since the outlines and the holes are stored in the same list
                "An input polyline contained less than three points";
        }

        private struct TE_PolylineIndexOutOfBounds : ITriangulationError
        {
            private readonly Idx pointIndex;
            private readonly Idx numPointsInPolyline;

            public TE_PolylineIndexOutOfBounds(Idx pointIndex, Idx numPointsInPolyline)
            {
                this.pointIndex = pointIndex;
                this.numPointsInPolyline = numPointsInPolyline;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.PolylineIndexOutOfBounds;

            public string ErrorMessage => $"A polyline referenced a point at index {pointIndex}, which is not in range [0, {numPointsInPolyline - 1}]";
        }

        private struct TE_PolylineDuplicateConsecutivePoints : ITriangulationError
        {
            private readonly Idx pointIndex;

            public TE_PolylineDuplicateConsecutivePoints(Idx pointIndex)
            {
                this.pointIndex = pointIndex;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.PolylineDuplicateConsecutivePoints;

            public string ErrorMessage => $"A polyline contains two duplicate consecutive points (index {pointIndex})";
        }

        private struct TE_EdgeWithDifferentConstrainedTypes : ITriangulationError
        {
            private readonly Idx idx0;
            private readonly Idx idx1;

            public TE_EdgeWithDifferentConstrainedTypes(Idx idx0, Idx idx1)
            {
                this.idx0 = idx0;
                this.idx1 = idx1;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.EdgeWithDifferentConstrainedTypes;

            public string ErrorMessage => $"The edge between points {idx0} and {idx1} is part of both an outline and a hole";
        }

        private struct TE_PointOnConstrainedEdge : ITriangulationError
        {
            private readonly Idx pointIndex;
            private readonly Idx edgePointIndex0;
            private readonly Idx edgePointIndex1;

            public TE_PointOnConstrainedEdge(Idx pointIndex, Idx edgePointIndex0, Idx edgePointIndex1)
            {
                this.pointIndex = pointIndex;
                this.edgePointIndex0 = edgePointIndex0;
                this.edgePointIndex1 = edgePointIndex1;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.PointOnConstrainedEdge;

            public string ErrorMessage => $"Point {pointIndex} is exactly on a constrained edge (between points {edgePointIndex0} and {edgePointIndex1})";
        }

        private struct TE_ConstrainedEdgeIntersection : ITriangulationError
        {
            private readonly Idx idx0;
            private readonly Idx idx1;
            private readonly Idx idx2;
            private readonly Idx idx3;

            public TE_ConstrainedEdgeIntersection(Idx idx0, Idx idx1, Idx idx2, Idx idx3)
            {
                this.idx0 = idx0;
                this.idx1 = idx1;
                this.idx2 = idx2;
                this.idx3 = idx3;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.ConstrainedEdgeIntersection;

            public string ErrorMessage => $"Constrained edges are intersecting: first edge between points {idx0} and {idx1}, second edge between points {idx2} and {idx3}";
        }

        private struct TE_HoleNotInsideOutline : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.HoleNotInsideOutline;

            public string ErrorMessage => "Found a hole which was not inside any outlines";
        }

        private struct TE_StackedPolylines : ITriangulationError
        {
            private readonly bool isHole;

            public TE_StackedPolylines(bool isHole)
            {
                this.isHole = isHole;
            }

            public TriangulationErrorType ErrorType => TriangulationErrorType.StackedPolylines;

            public string ErrorMessage => isHole
                    ? "A hole was directly inside another hole"
                    : "An outline was directly inside another outline";
        }

        private struct TE_AssertionFailed : ITriangulationError
        {
            public TriangulationErrorType ErrorType => TriangulationErrorType.AssertionFailed;

            public string ErrorMessage => "Internal error, this indicates a bug in detria's code";
        }

        #endregion


        // Inputs
        private IReadOnlyList<Vec2> _points = null;
        private readonly List<PolylineData> _polylines = new List<PolylineData>();
        private readonly List<Edge> _manuallyConstrainedEdges = new List<Edge>();
        private readonly List<EdgeType> _autoDetectedPolylineTypes = new List<EdgeType>();

        private readonly Topology _topology = new Topology();

        // Reused containers for multiple calculations
        private readonly List<Idx> _initialTriangulation_SortedPoints = new List<Idx>();
        private readonly List<VertexIndex> _constrainedEdgeVerticesCW = new List<VertexIndex>();
        private readonly List<VertexIndex> _constrainedEdgeVerticesCCW = new List<VertexIndex>();
        private readonly Stack<HalfEdgeIndex> _deletedConstrainedEdges = new Stack<HalfEdgeIndex>(InitialCapacity);
        private readonly List<VertexIndex> _constrainedEdgeReTriangulationStack = new List<VertexIndex>();
        private readonly List<TriangleIndex> _classifyTriangles_TrianglesByHalfEdgeIndex = new List<TriangleIndex>();
        private readonly List<bool> _classifyTriangles_CheckedHalfEdges = new List<bool>();
        private readonly List<bool> _classifyTriangles_CheckedTriangles = new List<bool>();
        private readonly Stack<TriangleIndex> _classifyTriangles_TrianglesToCheck = new Stack<TriangleIndex>(256);
        private readonly HalfEdgeIndex[] _classifyTriangles_TmpTriangleEdges = new HalfEdgeIndex[3];

        private readonly Stack<TopologyEdgeWithVertices> _delaunayCheckStack = new Stack<TopologyEdgeWithVertices>(InitialCapacity);

        private readonly List<TriangleWithData> _resultTriangles = new List<TriangleWithData>();
        private ITriangulationError _error = new TE_NotStarted();

        // Results which are not related to the triangulation directly
        private VertexIndex _convexHullInitialVertex = NullVertex;
        private readonly List<NullableIdx> _parentPolylines = new List<NullableIdx>();

        // Misc
        private static readonly VertexIndex NullVertex = VertexIndex.Null;
        private static readonly HalfEdgeIndex NullEdge = HalfEdgeIndex.Null;
        private static readonly byte[] _constrainedEdgeTypePriorities = GetConstrainedEdgeTypePriorities();

        private readonly Predicates _pred = new Predicates();

        // Guess capacity, 64 should be a good starting point
        private const int InitialCapacity = 64;

        private struct TriangleWithData
        {
            public Triangle triangle;
            public TriangleData data;
        }

        private struct TopologyEdgeWithVertices
        {
            public VertexIndex v0;
            public VertexIndex v1;
            public HalfEdgeIndex edge;
            public VertexIndex oppositeVertex;
        }

        private struct TriangleData
        {
            // Only valid if locationDataType == LocationDataType.Known
            // The index of the inner-most outline or hole that contains this triangle
            // Null id if the triangle is outside of all polylines
            public NullableIdx parentPolylineIndex;

            public HalfEdgeIndex firstEdge;

            public LocationDataType locationDataType;

            public enum LocationDataType
            {
                Unknown,
                Known,
            }
        }

        private struct TriangleIndex : IEquatable<TriangleIndex>
        {
            public NullableIdx index;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public TriangleIndex(NullableIdx index)
            {
                this.index = index;
            }

            public static TriangleIndex Null
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return new TriangleIndex(NullableIdx.Null);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator ==(TriangleIndex a, TriangleIndex b)
            {
                return a.Equals(b);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool operator !=(TriangleIndex a, TriangleIndex b)
            {
                return !a.Equals(b);
            }

            public bool IsValid
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return index != Constants.NullIndex;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override bool Equals(object obj)
            {
                if (!(obj is TriangleIndex))
                {
                    return false;
                }

                TriangleIndex other = (TriangleIndex)obj;
                return Equals(other);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override int GetHashCode()
            {
                return index.GetHashCode();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(TriangleIndex other)
            {
                return index == other.index;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public override string ToString()
            {
                return index.ToString();
            }
        }

        private struct PolylineData
        {
            public IReadOnlyList<Idx> pointIndices;
            public EdgeType type;
        }

        private bool Fail(ITriangulationError errorData)
        {
            _error = errorData;
            return false;
        }

        private void ClearInternalData()
        {
            _topology.Clear();
            _resultTriangles.Clear();
            _initialTriangulation_SortedPoints.Clear();
            _constrainedEdgeVerticesCW.Clear();
            _constrainedEdgeVerticesCCW.Clear();
            _deletedConstrainedEdges.Clear();
            _constrainedEdgeReTriangulationStack.Clear();
            _classifyTriangles_TrianglesByHalfEdgeIndex.Clear();
            _classifyTriangles_CheckedHalfEdges.Clear();
            _classifyTriangles_CheckedTriangles.Clear();
            _classifyTriangles_TrianglesToCheck.Clear();
            _delaunayCheckStack.Clear();
            _convexHullInitialVertex = NullVertex;
            _parentPolylines.Clear();
            _autoDetectedPolylineTypes.Clear();
        }

        private bool TriangulateInternal(bool delaunay)
        {
            if (_points == null || _points.Count < 3)
            {
                // Need at least 3 points to triangulate
                return Fail(new TE_LessThanThreePoints());
            }

            if (EnableNaNChecks)
            {
                // Check NaN / inf values
                for (int i = 0; i < _points.Count; ++i)
                {
                    Vec2 p = _points[i];
                    bool invalidX = Scalar.IsNaN(p.x) || Scalar.IsInfinity(p.x);
                    bool invalidY = Scalar.IsNaN(p.y) || Scalar.IsInfinity(p.y);

                    if (invalidX || invalidY)
                    {
                        return Fail(new TE_NonFinitePositionFound((Idx)i, invalidX ? p.x : p.y));
                    }
                }
            }

            // Reserve topology capacity
            _topology.ReserveVertices(_points.Count);
            int numTriangles = _points.Count * 2; // Guess number of triangles

            // Number of edges should be around number of vertices + number of triangles
            _topology.ReserveHalfEdges((_points.Count + numTriangles) * 2);

            Detail.Reserve(_resultTriangles, numTriangles);

            // Initialize point data
            List<Idx> sortedPoints = _initialTriangulation_SortedPoints;
            sortedPoints.Clear();
            Detail.Reserve(sortedPoints, _points.Count);

            for (int i = 0; i < _points.Count; ++i)
            {
                sortedPoints.Add((Idx)i);
                _topology.CreateVertex();
            }

            // Sort all points by x coordinates; for points with the same x coordinate, sort by y
            // If there are two points (or more) that have the same x and y coordinate, then we can't triangulate it,
            // because it would create degenerate triangles

            // Unlike the C++ implementation, use the built-in sort algorithm, since it seems to be slightly faster
            sortedPoints.Sort((idxA, idxB) =>
            {
                Vec2 a = _points[(int)idxA];
                Vec2 b = _points[(int)idxB];
                bool differentX = a.x != b.x;
                return differentX ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y);
            });

            // Since the points are sorted, duplicates must be next to each other (if any)
            for (int i = 1; i < _points.Count; ++i)
            {
                Idx prevIndex = sortedPoints[i - 1];
                Idx currentIndex = sortedPoints[i];

                Vec2 prev = _points[(int)prevIndex];
                Vec2 current = _points[(int)currentIndex];

                if (prev.x == current.x && prev.y == current.y)
                {
                    return Fail(new TE_DuplicatePointsFound(prevIndex, currentIndex, current.x, current.y));
                }
            }

            // Set initial convex hull vertex - the first sorted point is guaranteed to be part of the convex hull
            _convexHullInitialVertex = new VertexIndex(sortedPoints[0]);

            bool allPointsAreCollinear;

            // Initial triangulation, will add the edges
            if (!CreateInitialTriangulation(delaunay, sortedPoints, 0, sortedPoints.Count, out allPointsAreCollinear))
            {
                return false;
            }

            if (allPointsAreCollinear)
            {
                return Fail(new TE_AllPointsAreCollinear());
            }

            // Add constrained edges, which ensures that all required vertices have edges between them
            if (!CreateConstrainedEdges(delaunay))
            {
                return false;
            }

            // Go through all triangles, and decide if they are inside or outside
            if (!ClassifyTriangles())
            {
                return false;
            }

            return true;
        }

        private bool CreateInitialTriangulation(bool delaunay, List<Idx> sortedPoints, int startIndex, int endIndex, out bool allCollinear)
        {
            // Initial triangulation - just a valid triangulation that includes all points

            allCollinear = true; // Start with all collinear, set to false as soon as a non-collinear point is found

            // Find an initial edge
            // Since we have no duplicate points, we can always use the first two points

            Idx p0Idx = sortedPoints[startIndex];
            Idx p1Idx = sortedPoints[startIndex + 1];

            VertexIndex v0 = new VertexIndex(p0Idx);
            VertexIndex v1 = new VertexIndex(p1Idx);

            HalfEdgeIndex lastEdge;

            void MarkInitialBoundaryEdge(HalfEdgeIndex edge)
            {
                HalfEdgeIndex opposite = Topology.GetOpposite(edge);

                EdgeData edgeData = _topology.GetEdgeData(edge);
                EdgeData oppositeData = _topology.GetEdgeData(opposite);

                edgeData.SetBoundary(true);
                oppositeData.SetBoundary(true);

                if (delaunay)
                {
                    edgeData.SetDelaunay(true);
                    oppositeData.SetDelaunay(true);
                }

                _topology.SetEdgeData(edge, edgeData);
                _topology.SetEdgeData(opposite, oppositeData);
            }

            // Create initial edge
            {
                HalfEdgeIndex e10 = _topology.CreateNewEdge(v1, v0, NullEdge, NullEdge, NullEdge);
                MarkInitialBoundaryEdge(e10);
                lastEdge = e10;
            }

            // Add points
            for (int i = startIndex + 2; i < endIndex; ++i)
            {
                Idx originalIndex = sortedPoints[i];
                VertexIndex vertex = new VertexIndex(originalIndex);

                Vec2 position = _points[(int)originalIndex];

                bool IsEdgeVisible(HalfEdgeIndex edge)
                {
                    // Decide if the edge is visible from the current point

                    VertexIndex vertex0 = _topology.GetEdge(edge).vertex;
                    VertexIndex vertex1 = _topology.GetEdge(Topology.GetOpposite(edge)).vertex;

                    Math.Orientation orientation = Orient2d(
                        _points[(int)vertex0.index.Value],
                        _points[(int)vertex1.index.Value],
                        position
                    );

                    // Don't consider point as visible if exactly on the line
                    return orientation == Math.Orientation.CCW;
                }

                // Start checking edges, and find the first and last one that is visible from the current point
                // The vertex of `lastEdge` is guaranteed to be visible, so it's a good starting point

                HalfEdgeIndex lastVisibleForwards = lastEdge;
                HalfEdgeIndex lastVisibleBackwards = GetPrevEdgeAlongConvexHull(lastEdge);
                HalfEdgeIndex prevVisibleBackwards = lastEdge;

                // Check forwards
                while (true)
                {
                    if (IsEdgeVisible(lastVisibleForwards))
                    {
                        lastVisibleForwards = GetNextEdgeAlongConvexHull(lastVisibleForwards);
                    }
                    else
                    {
                        break;
                    }
                }

                // Check backwards
                while (true)
                {
                    if (IsEdgeVisible(lastVisibleBackwards))
                    {
                        prevVisibleBackwards = lastVisibleBackwards;
                        lastVisibleBackwards = GetPrevEdgeAlongConvexHull(lastVisibleBackwards);
                    }
                    else
                    {
                        break;
                    }
                }

                if (lastVisibleForwards == prevVisibleBackwards)
                {
                    // All points are collinear so far, so only add an edge

                    VertexIndex prevVertex = new VertexIndex(sortedPoints[i - 1]);
                    lastEdge = _topology.CreateNewEdge(vertex, prevVertex, NullEdge, lastEdge, NullEdge);
                    MarkInitialBoundaryEdge(lastEdge);
                    continue;
                }

                allCollinear = false;

                // Add new edges
                // If delaunay, then add edges that we are about to remove to the list of edges to check
                HalfEdgeIndex current = prevVisibleBackwards;
                HalfEdgeIndex lastAddedEdge = NullEdge;

                while (current != lastVisibleForwards)
                {
                    HalfEdgeIndex edge0i = current;
                    Topology.HalfEdge edge0 = _topology.GetEdge(current);
                    VertexIndex currenVertexIndex = edge0.vertex;
                    Topology.HalfEdge oppositeEdge0 = _topology.GetEdge(Topology.GetOpposite(current));
                    VertexIndex nexVertexIndex = oppositeEdge0.vertex;
                    HalfEdgeIndex nextEdge = oppositeEdge0.nextEdge;

                    HalfEdgeIndex edge1i = lastAddedEdge.IsValid
                        ? lastAddedEdge
                        : _topology.CreateNewEdge(vertex, currenVertexIndex, NullEdge, edge0.prevEdge, NullEdge);
                    Topology.HalfEdge edge1 = _topology.GetEdge(edge1i);

                    HalfEdgeIndex oppositeEdge0i = Topology.GetOpposite(current);
                    HalfEdgeIndex edge2i = _topology.CreateNewEdge(vertex, nexVertexIndex, edge1.prevEdge, oppositeEdge0i, NullEdge);

                    // Update boundary status
                    EdgeData oppositeEdgeData = _topology.GetEdgeData(oppositeEdge0i);
                    oppositeEdgeData.SetBoundary(false); // Previously boundary edge, but it became interior now
                    _topology.SetEdgeData(oppositeEdge0i, oppositeEdgeData);

                    if (!lastAddedEdge.IsValid)
                    {
                        // Mark first edge of the newly added vertex as boundary
                        edge1 = _topology.GetEdge(edge1i);
                        edge1.data.SetBoundary(true);
                        _topology.SetEdge(edge1i, edge1);
                    }

                    lastAddedEdge = edge2i;

                    if (delaunay)
                    {
                        // Make sure the newly added triangle meets the delaunay criteria
                        AddEdgeToFlipCheck(edge0i, vertex);
                    }

                    current = nextEdge;
                }

                // Finally, mark lastAddedEdge as boundary
                HalfEdgeIndex lastEdgeOpposite = Topology.GetOpposite(lastAddedEdge);
                EdgeData lastEdgeData = _topology.GetEdgeData(lastEdgeOpposite);
                lastEdgeData.SetBoundary(true);
                _topology.SetEdgeData(lastEdgeOpposite, lastEdgeData);

                lastEdge = lastAddedEdge;

                if (delaunay)
                {
                    DelaunayEdgeFlip();
                }
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private HalfEdgeIndex GetNextEdgeAlongConvexHull(HalfEdgeIndex halfEdge)
        {
            Topology.HalfEdge opposite = _topology.GetEdge(Topology.GetOpposite(halfEdge));
            return opposite.nextEdge;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private HalfEdgeIndex GetPrevEdgeAlongConvexHull(HalfEdgeIndex halfEdge)
        {
            HalfEdgeIndex prevEdge = _topology.GetEdge(halfEdge).prevEdge;
            return Topology.GetOpposite(prevEdge);
        }

        // Add an edge which should be checked for delaunay edge flip
        private void AddEdgeToFlipCheck(HalfEdgeIndex e, VertexIndex oppositeVertex)
        {
            Topology.HalfEdge edge = _topology.GetEdge(e);
            HalfEdgeIndex oppositeIndex = Topology.GetOpposite(e);
            Topology.HalfEdge oppositeEdge = _topology.GetEdge(oppositeIndex);

            edge.data.SetDelaunay(false);
            oppositeEdge.data.SetDelaunay(false);

            _topology.SetEdge(e, edge);
            _topology.SetEdge(oppositeIndex, oppositeEdge);

            _delaunayCheckStack.Push(new TopologyEdgeWithVertices
            {
                v0 = edge.vertex,
                v1 = oppositeEdge.vertex,
                edge = e,
                oppositeVertex = oppositeVertex,
            });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool DelaunayEdgeFlip(VertexIndex justAddedVertex, HalfEdgeIndex oppositeEdge)
        {
            AddEdgeToFlipCheck(oppositeEdge, justAddedVertex);
            return DelaunayEdgeFlip();
        }

        // Use `_delaunayCheckStack` in this function
        // It will always be empty when this function returns
        private bool DelaunayEdgeFlip()
        {
            TopologyEdgeWithVertices edgeWithVertices;

            // Flip edges if needed
            while (_delaunayCheckStack.TryPop(out edgeWithVertices))
            {
                HalfEdgeIndex e01 = edgeWithVertices.edge;
                HalfEdgeIndex e10 = Topology.GetOpposite(e01);

                Topology.HalfEdge edge01 = _topology.GetEdge(e01);
                Topology.HalfEdge edge10 = _topology.GetEdge(e10);

                // Check if the edge still has the same vertices
                if (edge01.vertex != edgeWithVertices.v0 || edge10.vertex != edgeWithVertices.v1)
                {
                    // Edge was flipped
                    continue;
                }

                EdgeData edgeData = edge01.data;
                EdgeData oppositeEdgeData = edge10.data;
                if (edgeData.IsDelaunay || edgeData.IsConstrained || oppositeEdgeData.IsDelaunay || oppositeEdgeData.IsConstrained)
                {
                    // Don't flip edges that are already delaunay or constrained
                    continue;
                }

                if (edgeData.IsBoundary || oppositeEdgeData.IsBoundary)
                {
                    // Also don't flip boundary edges
                    edge01.data.SetDelaunay(true);
                    edge10.data.SetDelaunay(true);

                    _topology.SetEdge(e01, edge01);
                    _topology.SetEdge(e10, edge10);

                    continue;
                }

                VertexIndex vertex0 = edge01.vertex;
                VertexIndex vertex1 = edge10.vertex;
                VertexIndex otherVertex0 = _topology.GetEdge(Topology.GetOpposite(edge01.nextEdge)).vertex;
                VertexIndex otherVertex1 = _topology.GetEdge(Topology.GetOpposite(edge10.nextEdge)).vertex;

                if (otherVertex0 == otherVertex1)
                {
                    // This can happen during re-triangulation around a constrained edge
                    // This just means that the edge is boundary, so skip it
                    continue;
                }

                /*

         vertex0     otherVertex1
                *---*
                |\  |
                | \ |
                |  \|
                *---*
    otherVertex0     vertex1

                */

                // Points of the current edge
                Vec2 vertex0Position = _points[(int)vertex0.index.Value];
                Vec2 vertex1Position = _points[(int)vertex1.index.Value];

                // Points of the edge that we'd get if a flip is needed
                Vec2 otherVertex0Position = _points[(int)otherVertex0.index.Value];
                Vec2 otherVertex1Position = _points[(int)otherVertex1.index.Value];

                // TODO?: maybe we could allow user-defined functions to decide if an edge should be flipped
                // That would enable other metrics, e.g. minimize edge length, flip based on the aspect ratio of the triangles, etc.
                // https://people.eecs.berkeley.edu/~jrs/papers/elemj.pdf
                // But we'd need to make sure that every edge is only processed once

                Math.CircleLocation loc = Incircle(vertex0Position, vertex1Position, otherVertex1Position, otherVertex0Position);
                if (loc == Math.CircleLocation.Inside)
                {
                    // Flip edge
                    // The edge is always flippable if we get here, no need to do orientation checks

                    VertexIndex oppositeVertex = edgeWithVertices.oppositeVertex;
                    _topology.FlipEdge(e01);
                    edge01 = _topology.GetEdge(e01);
                    edge10 = _topology.GetEdge(e10);

                    // Flipping an edge might require other edges to be flipped too
                    // Only check edges which can be non-delaunay
                    if (oppositeVertex.index == otherVertex0.index)
                    {
                        AddEdgeToFlipCheck(edge01.prevEdge, oppositeVertex);
                        AddEdgeToFlipCheck(edge01.nextEdge, oppositeVertex);
                    }
                    else
                    {
#if DEBUG
                        if (oppositeVertex.index != otherVertex1.index)
                        {
                            Detail.ThrowAssertionFailedError();
                        }
#endif

                        AddEdgeToFlipCheck(edge10.nextEdge, oppositeVertex);
                        AddEdgeToFlipCheck(edge10.prevEdge, oppositeVertex);
                    }
                }

                edge01.data.SetDelaunay(true);
                edge10.data.SetDelaunay(true);

                _topology.SetEdge(e01, edge01);
                _topology.SetEdge(e10, edge10);
            }

            return true;
        }

        private bool CreateConstrainedEdges(bool delaunay)
        {
            // Ensure that an edge exists between each consecutive vertex for all outlines and holes

            for (int i = 0; i < _polylines.Count; ++i)
            {
                PolylineData polylineData = _polylines[i];

                IReadOnlyList<Idx> polyline = polylineData.pointIndices;
                EdgeType constrainedEdgeType = polylineData.type;

                if (polyline.Count < 3)
                {
                    return Fail(new TE_PolylineTooShort());
                }

                Idx prevVertexIdx;
                int startingIndex;

                // We allow polylines to have the same start and end vertex, e.g. [0, 1, 2, 3, 0]

                if (polyline[0] == polyline[polyline.Count - 1])
                {
                    if (polyline.Count < 4)
                    {
                        // [0, 1, 0] is invalid, even though it has 3 elements
                        return Fail(new TE_PolylineTooShort());
                    }

                    prevVertexIdx = polyline[0];
                    startingIndex = 1;
                }
                else
                {
                    prevVertexIdx = polyline[polyline.Count - 1];
                    startingIndex = 0;
                }

                for (int j = startingIndex; j < polyline.Count; ++j)
                {
                    Idx currentIdx = polyline[j];

                    if (!ConstrainSingleEdge(prevVertexIdx, currentIdx, constrainedEdgeType, (Idx)i, delaunay))
                    {
                        return false;
                    }

                    prevVertexIdx = currentIdx;
                }
            }

            for (int i = 0; i < _manuallyConstrainedEdges.Count; ++i)
            {
                Edge edge = _manuallyConstrainedEdges[i];
                if (!ConstrainSingleEdge(edge.x, edge.y, EdgeType.ManuallyConstrained, NullableIdx.Null, delaunay))
                {
                    return false;
                }
            }

            return true;
        }

        private static byte[] GetConstrainedEdgeTypePriorities()
        {
            byte[] priorities = new byte[(int)EdgeType.MAX_EDGE_TYPE];
            priorities[(int)EdgeType.NotConstrained] = 0;
            priorities[(int)EdgeType.ManuallyConstrained] = 1;
            priorities[(int)EdgeType.AutoDetect] = 2;
            priorities[(int)EdgeType.Outline] = 3;
            priorities[(int)EdgeType.Hole] = 3;

            return priorities;
        }

        private bool ConstrainSingleEdge(Idx idxA, Idx idxB, EdgeType constrainedEdgeType, NullableIdx polylineIndex, bool delaunay)
        {
            if (idxA < 0 || idxA >= (Idx)_points.Count || idxB < 0 || idxB >= (Idx)_points.Count)
            {
                return Fail(new TE_PolylineIndexOutOfBounds((idxA < 0 || idxA >= (Idx)_points.Count) ? idxA : idxB, (Idx)_points.Count));
            }

            if (idxA == idxB)
            {
                return Fail(new TE_PolylineDuplicateConsecutivePoints(idxA));
            }

            VertexIndex v0 = new VertexIndex(idxA);
            VertexIndex v1 = new VertexIndex(idxB);

            HalfEdgeIndex currentEdgeIndex = _topology.GetEdgeBetween(v0, v1);
            if (currentEdgeIndex.IsValid)
            {
                // There is already an edge between the vertices, which may or may not be constrained already
                // We have priorities to decide which edge types are overwritten by other types
                // If an edge is not yet constrained, then always overwrite it
                // If an edge is manually constrained, then only overwrite it if the new type is auto-detect, outline, or hole
                // If an edge is auto-detect, then only overwrite it with outline or hole
                // If an edge is already an outline or hole, then check if the new type is the same
                // If they are different, then it's an error

                EdgeData currentEdgeData = _topology.GetEdgeData(currentEdgeIndex);

                EdgeType currentEdgeType = currentEdgeData.EdgeType;
                byte currentPriority = _constrainedEdgeTypePriorities[(int)currentEdgeType];
                byte newPriority = _constrainedEdgeTypePriorities[(int)constrainedEdgeType];

                if (newPriority > currentPriority)
                {
                    // New edge type is higher priority, overwrite

                    HalfEdgeIndex oppositeEdgeIndex = Topology.GetOpposite(currentEdgeIndex);
                    EdgeData oppositeEdgeData = _topology.GetEdgeData(oppositeEdgeIndex);

                    switch (constrainedEdgeType)
                    {
                        case EdgeType.ManuallyConstrained:
                            currentEdgeData.dataType = oppositeEdgeData.dataType = EdgeData.DataType.ManuallyConstrained;
                            _topology.SetEdgeData(currentEdgeIndex, currentEdgeData);
                            _topology.SetEdgeData(oppositeEdgeIndex, oppositeEdgeData);
                            break;
                        case EdgeType.AutoDetect:
                        case EdgeType.Outline:
                        case EdgeType.Hole:
                            currentEdgeData.dataType = oppositeEdgeData.dataType = EdgeData.DataType.OutlineOrHole;
                            currentEdgeData.polylineIndex = oppositeEdgeData.polylineIndex = polylineIndex.Value;
                            currentEdgeData.type = oppositeEdgeData.type = constrainedEdgeType;

                            _topology.SetEdgeData(currentEdgeIndex, currentEdgeData);
                            _topology.SetEdgeData(oppositeEdgeIndex, oppositeEdgeData);
                            break;

                        default:
                            Detail.ThrowAssertionFailedError();
                            break;
                    }
                }
                else if (newPriority == currentPriority)
                {
                    // Same priority, so the types must also be the same

                    if (currentEdgeType != constrainedEdgeType)
                    {
                        // Cannot have an edge that is both an outline and a hole
                        return Fail(new TE_EdgeWithDifferentConstrainedTypes(idxA, idxB));
                    }
                }
                // Else lower priority, don't do anything

                return true;
            }

            /*
            Example:

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

            We want to create a constrained edge between v0 and v1
            Start from v0
            Since triangles are cw, there must be a triangle which contains v0, and has points pNext and pPrev,
            for which orient2d(v0, v1, pNext) == CCW and orient2d(v0, v1, pPrev) == CW
            In the current example, the starting triangle is "v0 a c"
            The next triangle is the other triangle of edge "a c"
            Check the third point of that next triangle (b), whether orient2d(v0, v1, b) is CW or CCW
            If CW, then the next edge is "a b", if CCW, then "c b"
            Repeat this until we find v1 in one of the triangles
            Also keep track of the outer vertices for both the CW and CCW side
            (in this example, the CW side is "v0 c d v1", CCW side is "v0 a b v1")
            Then re-triangulate the CW and CCW sides separately
            This will ensure that an edge exists between v0 and v1
            */

            Idx p0 = v0.index.Value;
            Idx p1 = v1.index.Value;

            HalfEdgeIndex initialTriangleEdge;
            VertexIndex vertexCW;
            VertexIndex vertexCCW;

            // Find initial triangle, which is in the direction of the other point
            if (!FindInitialTriangleForConstrainedEdge(v0, v1, p0, p1, out initialTriangleEdge, out vertexCW, out vertexCCW))
            {
                return false;
            }

            _constrainedEdgeVerticesCW.Clear();
            _constrainedEdgeVerticesCCW.Clear();

            // Traverse adjacent triangles, until we reach v1
            // Store vertex indices of the triangles along the way
            // Also store the deleted edges, those will be reused later
            // Since the total number of edges doesn't change here, we will always exactly have the right amount of edges
            _deletedConstrainedEdges.Clear();
            if (!RemoveInnerTrianglesAndGetOuterVertices(v0, v1, p0, p1, vertexCW, vertexCCW, initialTriangleEdge,
                _constrainedEdgeVerticesCW, _constrainedEdgeVerticesCCW))
            {
                return false;
            }

            // Create new edge, and mark it constrained
            HalfEdgeIndex beforeV0 = _topology.GetEdgeBetween(v0, _constrainedEdgeVerticesCCW[1]); // First is v0
            HalfEdgeIndex beforeV1 = _topology.GetEdgeBetween(v1, _constrainedEdgeVerticesCW[_constrainedEdgeVerticesCW.Count - 2]); // Last is v1

            HalfEdgeIndex constrainedEdgeIndex = _topology.CreateNewEdge(v0, v1, beforeV0, beforeV1, _deletedConstrainedEdges.Pop());

            Topology.HalfEdge constrainedEdge = _topology.GetEdge(constrainedEdgeIndex);
            HalfEdgeIndex oppositeConstrainedEdgeIndex = Topology.GetOpposite(constrainedEdgeIndex);
            Topology.HalfEdge oppositeConstrainedEdge = _topology.GetEdge(oppositeConstrainedEdgeIndex);

            if (constrainedEdgeType == EdgeType.ManuallyConstrained)
            {
                constrainedEdge.data = oppositeConstrainedEdge.data = new EdgeData
                {
                    dataType = EdgeData.DataType.ManuallyConstrained,
                    flags = EdgeData.Flags.None,
                };
            }
            else
            {
                constrainedEdge.data = oppositeConstrainedEdge.data = new EdgeData
                {
                    dataType = EdgeData.DataType.OutlineOrHole,
                    flags = EdgeData.Flags.None,

                    type = constrainedEdgeType,
                    polylineIndex = polylineIndex.Value,
                };
            }

            _topology.SetEdge(constrainedEdgeIndex, constrainedEdge);
            _topology.SetEdge(oppositeConstrainedEdgeIndex, oppositeConstrainedEdge);

            // Re-triangulate deleted triangles

            bool reTriangulateSuccess =
                ReTriangulateAroundConstrainedEdge(_constrainedEdgeVerticesCW, delaunay, true) &&
                ReTriangulateAroundConstrainedEdge(_constrainedEdgeVerticesCCW, delaunay, false);

            if (!reTriangulateSuccess)
            {
                return false;
            }

#if DEBUG
            if (_deletedConstrainedEdges.Count != 0)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif
            return true;
        }

        private bool FindInitialTriangleForConstrainedEdge(VertexIndex v0, VertexIndex v1, Idx p0, Idx p1,
            out HalfEdgeIndex initialTriangleEdge, out VertexIndex vertexCW, out VertexIndex vertexCCW)
        {
            bool found = false;
            bool error = false;

            Vec2 p0Position = _points[(int)p0];
            Vec2 p1Position = _points[(int)p1];

            HalfEdgeIndex initialTriangleEdge_ = NullEdge;
            VertexIndex vertexCW_ = NullVertex;
            VertexIndex vertexCCW_ = NullVertex;

            _topology.ForEachEdgeOfVertex(v0, (HalfEdgeIndex e) =>
            {
                Topology.HalfEdge edge = _topology.GetEdge(e);

                VertexIndex prevVertex = _topology.GetEdge(Topology.GetOpposite(edge.nextEdge)).vertex;
                VertexIndex nexVertexIndex = _topology.GetEdge(Topology.GetOpposite(e)).vertex;

                // We are looking for a triangle where "v0, v1, nexVertexIndex" is ccw, and "v0, v1, prevVertex" is cw

                Vec2 prevVertexPosition = _points[(int)prevVertex.index.Value];
                Vec2 nexVertexIndexPosition = _points[(int)nexVertexIndex.index.Value];

                Math.Orientation orientNext = Orient2d(p0Position, p1Position, nexVertexIndexPosition);
                Math.Orientation orientPrev = Orient2d(p0Position, p1Position, prevVertexPosition);

                if (orientPrev == Math.Orientation.Collinear || orientNext == Math.Orientation.Collinear)
                {
                    // We found a point which is exactly on the line
                    // Check if it's between v0 and v1
                    // If yes, then this means that an edge would have to go 3 points, which would create degenerate triangles
                    // If this is a steiner point, then maybe we could remove it, but for now, this is an error
                    // If it's not between, then simply skip this triangle

                    bool IsBetween(Vec2 point)
                    {
                        Scalar xDiff = Predicates.Absolute(p0Position.x - p1Position.x);
                        Scalar yDiff = Predicates.Absolute(p0Position.y - p1Position.y);

                        Scalar p0Value;
                        Scalar p1Value;
                        Scalar pointValue;
                        if (xDiff > yDiff)
                        {
                            // Compare x coordinate
                            p0Value = p0Position.x;
                            p1Value = p1Position.x;
                            pointValue = point.x;
                        }
                        else
                        {
                            // Compare y coordinate
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
                    }

                    if (orientPrev == Math.Orientation.Collinear)
                    {
                        error = IsBetween(prevVertexPosition);
                    }
                    else
                    {
                        error = IsBetween(nexVertexIndexPosition);
                    }

                    if (error)
                    {
                        found = true;
                        Fail(new TE_PointOnConstrainedEdge(
                            (orientPrev == Math.Orientation.Collinear ? prevVertex.index : nexVertexIndex.index).Value,
                            v0.index.Value,
                            v1.index.Value
                        ));
                        return false;
                    }

                    return true;
                }
                else if (orientPrev == Math.Orientation.CW && orientNext == Math.Orientation.CCW)
                {
                    // Correct orientation, triangle found
                    found = true;

                    initialTriangleEdge_ = e;
                    vertexCW_ = prevVertex;
                    vertexCCW_ = nexVertexIndex;

                    return false;
                }

                return true;
            });

            if (!found)
            {
#if DEBUG
                Detail.ThrowAssertionFailedError();
#endif
            }

            initialTriangleEdge = initialTriangleEdge_;
            vertexCW = vertexCW_;
            vertexCCW = vertexCCW_;

            return !error;
        }

        private bool RemoveInnerTrianglesAndGetOuterVertices(VertexIndex v0, VertexIndex v1, Idx p0, Idx p1,
            VertexIndex vertexCW, VertexIndex vertexCCW, HalfEdgeIndex initialTriangleEdge,
            List<VertexIndex> verticesCW, List<VertexIndex> verticesCCW)
        {
            verticesCW.Add(v0);
            verticesCW.Add(vertexCW);
            verticesCCW.Add(v0);
            verticesCCW.Add(vertexCCW);

            HalfEdgeIndex edgeAcross = _topology.GetEdge(Topology.GetOpposite(initialTriangleEdge)).prevEdge;

            while (true)
            {
                Topology.HalfEdge edge = _topology.GetEdge(edgeAcross);
                if (edge.data.EdgeType != EdgeType.NotConstrained)
                {
                    // To constrain the current edge, we'd have to remove another edge that is already constrained
                    // This means that some outlines or holes are intersecting
                    return Fail(new TE_ConstrainedEdgeIntersection(v0.index.Value, v1.index.Value, vertexCW.index.Value, vertexCCW.index.Value));
                }

                HalfEdgeIndex indexOfEdgeOfThirdVertex = Topology.GetOpposite(edge.prevEdge);
                Topology.HalfEdge edgeOfThirdVertex = _topology.GetEdge(indexOfEdgeOfThirdVertex);
                VertexIndex thirdVertex = edgeOfThirdVertex.vertex;

                HalfEdgeIndex nextEdgeAcross = NullEdge;

                bool reachedOtherEnd = thirdVertex == v1;
                if (!reachedOtherEnd)
                {
                    // Find next direction

                    Math.Orientation orientation = Orient2d(_points[(int)p0], _points[(int)p1], _points[(int)thirdVertex.index.Value]);
                    if (orientation == Math.Orientation.Collinear)
                    {
                        // Point on a constrained edge, this is not allowed
                        return Fail(new TE_PointOnConstrainedEdge(thirdVertex.index.Value, v0.index.Value, v1.index.Value));
                    }

                    if (orientation == Math.Orientation.CW)
                    {
                        vertexCW = thirdVertex;
                        verticesCW.Add(thirdVertex);
                        nextEdgeAcross = Topology.GetOpposite(indexOfEdgeOfThirdVertex);
                    }
                    else
                    {
                        vertexCCW = thirdVertex;
                        verticesCCW.Add(thirdVertex);
                        nextEdgeAcross = edgeOfThirdVertex.prevEdge;
                    }
                }

                // Remove edge
                _topology.UnlinkEdge(edgeAcross);
                _deletedConstrainedEdges.Push(edgeAcross);

                if (reachedOtherEnd)
                {
                    break;
                }

                edgeAcross = nextEdgeAcross;
            }

            verticesCW.Add(v1);
            verticesCCW.Add(v1);


#if DEBUG
            if (verticesCW.Count < 3 || verticesCCW.Count < 3)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif

            return true;
        }

        private bool ReTriangulateAroundConstrainedEdge(List<VertexIndex> vertices, bool delaunay, bool isCW)
        {
            /*
            `requiredOrientation` depends on which side of the line we are on
            For example, let's say we have to re-triangulate this:

              v0    a     b
                *---*---*
               / \     /
            c *   \   /
              |    \ /
              *-----*
             d      v1

            We want to triangulate "v0 a b v1" and "v0 c d v1"
            The first polyline is CCW, the second is CW, because all points are on that side of the "v0 v1" line
            When triangulating the first polyline, we can only add a triangle if the vertices are CW
            So for example, we cannot add the "v0 a b" triangle, because the points are not CW, they are collinear; but we can add "a b v1"
            Similarly, for the second polyline, we can only add a triangle if the points are CCW, so we can add "v0 c d"
            So:
                CW side of the "v0 v1" line -> `requiredOrientation` == CCW
                CCW side -> `requiredOrientation` == CW
            */

            Math.Orientation requiredOrientation = isCW ? Math.Orientation.CCW : Math.Orientation.CW;

            _constrainedEdgeReTriangulationStack.Clear();

            _constrainedEdgeReTriangulationStack.Add(vertices[0]);
            _constrainedEdgeReTriangulationStack.Add(vertices[1]);

            HalfEdgeIndex GetReusedEdge()
            {
#if DEBUG
                if (_deletedConstrainedEdges.Count == 0)
                {
                    Detail.ThrowAssertionFailedError();
                }
#endif
                return _deletedConstrainedEdges.Pop();
            }

            for (int i = 2; i < vertices.Count; ++i)
            {
                VertexIndex currenVertexIndex = vertices[i];

                while (true)
                {
                    VertexIndex prevPrevVertex = _constrainedEdgeReTriangulationStack[_constrainedEdgeReTriangulationStack.Count - 2];
                    VertexIndex prevVertex = _constrainedEdgeReTriangulationStack[_constrainedEdgeReTriangulationStack.Count - 1];

                    Math.Orientation orientation = Orient2d(
                        _points[(int)prevPrevVertex.index.Value],
                        _points[(int)prevVertex.index.Value],
                        _points[(int)currenVertexIndex.index.Value]
                    );

                    if (orientation == requiredOrientation)
                    {
                        // Add edge if needed

                        HalfEdgeIndex oppositeEdge = NullEdge;
                        if (isCW)
                        {
                            if (!_topology.GetEdgeBetween(prevPrevVertex, currenVertexIndex).IsValid)
                            {
                                oppositeEdge = _topology.GetEdgeBetween(prevPrevVertex, prevVertex);

                                _topology.CreateNewEdge(prevPrevVertex, currenVertexIndex,
                                    _topology.GetEdge(oppositeEdge).prevEdge,
                                    _topology.GetEdgeBetween(currenVertexIndex, prevVertex),
                                    GetReusedEdge()
                                );
                            }
                        }
                        else
                        {
                            if (!_topology.GetEdgeBetween(currenVertexIndex, prevPrevVertex).IsValid)
                            {
                                oppositeEdge = _topology.GetEdgeBetween(prevPrevVertex, prevVertex);

                                _topology.CreateNewEdge(currenVertexIndex, prevPrevVertex,
                                    _topology.GetEdge(_topology.GetEdgeBetween(currenVertexIndex, prevVertex)).prevEdge,
                                    oppositeEdge,
                                    GetReusedEdge()
                                );
                            }
                        }

                        // Update stack
                        _constrainedEdgeReTriangulationStack.RemoveAt(_constrainedEdgeReTriangulationStack.Count - 1);

                        if (delaunay)
                        {
                            if (i == vertices.Count - 1 && !oppositeEdge.IsValid)
                            {
                                // For the last vertex, the edge might already be created, so oppositeEdge is not always set
                                // But it is needed for delaunay edge flip, so make sure it is set properly
                                oppositeEdge = _topology.GetEdgeBetween(prevPrevVertex, prevVertex);
#if DEBUG
                                if (!oppositeEdge.IsValid)
                                {
                                    Detail.ThrowAssertionFailedError();
                                }
#endif
                            }

                            if (oppositeEdge.IsValid)
                            {
                                // Ensure delaunay criteria for the newly inserted triangle
                                // Usually, the triangles are already delaunay, but sometimes not, so check it here

                                if (!DelaunayEdgeFlip(currenVertexIndex, oppositeEdge))
                                {
                                    return false;
                                }
                            }
                        }

                        // If there are more than one items left in the stack, then go back and try to triangulate again
                        // Otherwise just break
                        if (_constrainedEdgeReTriangulationStack.Count < 2)
                        {
#if DEBUG
                            if (_constrainedEdgeReTriangulationStack.Count != 1)
                            {
                                Detail.ThrowAssertionFailedError();
                            }
#endif
                            break;
                        }
                    }
                    else
                    {
                        // Cannot add triangle, will try again later, after adding a new vertex
                        break;
                    }
                }

                _constrainedEdgeReTriangulationStack.Add(currenVertexIndex);
            }

#if DEBUG
            if (_constrainedEdgeReTriangulationStack.Count != 2)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif
            return true;
        }

        private bool ClassifyTriangles()
        {
            HalfEdgeIndex startingConvexHullEdge = Topology.GetOpposite(GetBoundaryEdgeOfVertex(_convexHullInitialVertex));

#if DEBUG
            if (!startingConvexHullEdge.IsValid)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif

            EdgeData startingEdgeData = _topology.GetEdgeData(startingConvexHullEdge);
            EdgeType startingEdgeType = startingEdgeData.EdgeType;
            if (startingEdgeType == EdgeType.Hole)
            {
                // If this happens, then it means that an outer polyline is a hole, which is invalid
                return Fail(new TE_HoleNotInsideOutline());
            }

            _resultTriangles.Clear();

            // Euler characteristic: number of triangles == number of edges - number of vertices + 1
            // Since the outer "face" doesn't exist, only add 1 instead of 2
            int expectedTriangleCount = 1 + _topology.HalfEdgeCount / 2 - _topology.VertexCount;

            Detail.Reserve(_resultTriangles, expectedTriangleCount);

            // Create all triangles

            List<TriangleIndex> trianglesByHalfEdgeIndex = _classifyTriangles_TrianglesByHalfEdgeIndex;
            Detail.ClearAndResize(trianglesByHalfEdgeIndex, _topology.HalfEdgeCount, TriangleIndex.Null);

            List<bool> checkedHalfEdges = _classifyTriangles_CheckedHalfEdges;
            Detail.ClearAndResize(checkedHalfEdges, _topology.HalfEdgeCount, false);

            for (int i = 0; i < _topology.HalfEdgeCount; ++i)
            {
                if (checkedHalfEdges[i])
                {
                    // Edge already checked
                    continue;
                }

                HalfEdgeIndex edge = new HalfEdgeIndex((Idx)i);

                if (_topology.GetEdgeData(edge).IsBoundary)
                {
                    // Half-edge is boundary, don't add its triangle
                    checkedHalfEdges[i] = true;
                    continue;
                }

                TriangleIndex triangleIndex = new TriangleIndex((Idx)_resultTriangles.Count);

                Topology.HalfEdge e0 = _topology.GetEdge(edge);

                HalfEdgeIndex e1i = _topology.GetEdge(Topology.GetOpposite(edge)).prevEdge;
                Topology.HalfEdge e1 = _topology.GetEdge(e1i);

                HalfEdgeIndex e2i = _topology.GetEdge(Topology.GetOpposite(e1i)).prevEdge;
                Topology.HalfEdge e2 = _topology.GetEdge(e2i);

                checkedHalfEdges[i] = true;
                checkedHalfEdges[(int)e1i.index.Value] = true;
                checkedHalfEdges[(int)e2i.index.Value] = true;

                trianglesByHalfEdgeIndex[i] = triangleIndex;
                trianglesByHalfEdgeIndex[(int)e1i.index.Value] = triangleIndex;
                trianglesByHalfEdgeIndex[(int)e2i.index.Value] = triangleIndex;

                TriangleWithData tri = new TriangleWithData
                {
                    triangle = new Triangle
                    {
                        x = e0.vertex.index.Value,
                        y = e1.vertex.index.Value,
                        z = e2.vertex.index.Value,
                    },
                    data = new TriangleData
                    {
                        parentPolylineIndex = NullableIdx.Null,
                        firstEdge = edge,
                        locationDataType = TriangleData.LocationDataType.Unknown,
                    },
                };

                _resultTriangles.Add(tri);
            }

            // Outer edge, only has one triangle
            TriangleIndex startingTriangle = trianglesByHalfEdgeIndex[(int)startingConvexHullEdge.index.Value];

#if DEBUG
            if (!startingTriangle.IsValid)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif

            TriangleWithData startingTriangleData = _resultTriangles[(int)startingTriangle.index.Value];
            startingTriangleData.data.locationDataType = TriangleData.LocationDataType.Known;

            // Null idx if the edge is not part of an outline or a hole
            startingTriangleData.data.parentPolylineIndex = startingEdgeData.OutlineOrHoleIndex;
            _resultTriangles[(int)startingTriangle.index.Value] = startingTriangleData;

            // For each polyline, store which other polyline contains them
            Detail.ClearAndResize(_parentPolylines, _polylines.Count, NullableIdx.Null);

            Detail.ClearAndResize(_autoDetectedPolylineTypes, _polylines.Count, EdgeType.AutoDetect);

            List<bool> checkedTriangles = _classifyTriangles_CheckedTriangles;
            Detail.ClearAndResize(checkedTriangles, _resultTriangles.Count, false);
            checkedTriangles[(int)startingTriangle.index.Value] = true;

            Stack<TriangleIndex> trianglesToCheck = _classifyTriangles_TrianglesToCheck;
            trianglesToCheck.Clear();
            trianglesToCheck.Push(startingTriangle);

            if (startingEdgeType == EdgeType.AutoDetect)
            {
                // Convex hull edge is auto-detect, so it must be an outline
                NullableIdx startingEdgePolylineIndex = startingEdgeData.OutlineOrHoleIndex;
                if (startingEdgePolylineIndex.HasValue)
                {
                    _autoDetectedPolylineTypes[(int)startingEdgePolylineIndex.Value] = EdgeType.Outline;
                }
                else
                {
                    Detail.ThrowAssertionFailedError();
                }
            }

            TriangleIndex currentTriangle;
            while (trianglesToCheck.TryPop(out currentTriangle))
            {
                TriangleData currentTriangleData = _resultTriangles[(int)currentTriangle.index.Value].data;
                NullableIdx currentTriangleParentPolylineIndex = currentTriangleData.parentPolylineIndex;
#if DEBUG
                if (currentTriangleData.locationDataType != TriangleData.LocationDataType.Known)
                {
                    Detail.ThrowAssertionFailedError();
                }
#endif
                bool currentTriangleIsInterior = GetTriangleLocation(currentTriangleData) == TriangleLocation.Interior;

                HalfEdgeIndex[] edgesOfCurrentTriangle = _classifyTriangles_TmpTriangleEdges;
                edgesOfCurrentTriangle[0] = currentTriangleData.firstEdge;
                edgesOfCurrentTriangle[1] = _topology.GetEdge(Topology.GetOpposite(edgesOfCurrentTriangle[0])).prevEdge;
                edgesOfCurrentTriangle[2] = _topology.GetEdge(Topology.GetOpposite(edgesOfCurrentTriangle[1])).prevEdge;

                for (Idx i = 0; i < 3; ++i)
                {
                    HalfEdgeIndex edgeIndex = edgesOfCurrentTriangle[i];
                    EdgeData edgeData = _topology.GetEdgeData(edgeIndex);

                    TriangleIndex neighborTriangle = trianglesByHalfEdgeIndex[(int)Topology.GetOpposite(edgeIndex).index.Value];
                    if (!neighborTriangle.IsValid)
                    {
                        // The triangle is on the edge of the entire triangulation, so it does not have all 3 neighbors
                        continue;
                    }

                    if (checkedTriangles[(int)neighborTriangle.index.Value])
                    {
                        // This triangle was already checked, don't check it again
                        continue;
                    }

                    checkedTriangles[(int)neighborTriangle.index.Value] = true;

                    NullableIdx neighborTriangleParentPolylineIndex;
                    NullableIdx polylineIndex = edgeData.OutlineOrHoleIndex;
                    if (polylineIndex.HasValue)
                    {
                        // Check which type of edge we just crossed (if this branch is entered, then it's part of either an outline or a hole)
                        // There are a few possibilities:
                        // If the triangle is inside, and the edge is part of an outline,
                        // then the outline's index must be the same as the triangle's parent polyline index
                        // Otherwise it would mean that there is an outline inside another outline, which is not allowed
                        // Same for holes: if the triangle is outside, and the edge is part of a hole,
                        // then it must be the same hole as the current triangle's parent
                        // Outlines can have any number of holes in them, and holes can have any number of outlines too

                        /*
                        For example:
                        Let's say that we have two polylines: "a" and "b"

                          *---*    *---*
                         a|   |   b|   |
                          *---*    *---*

                        Let's also say that `startingTriangle` (declared near the start of this function) is not part of any of the polylines
                        So the classification starts from outside
                        "a" and "b" both must be outlines, otherwise we'd have a hole in an area that is already outside, which is invalid
                        To verify this, whenever we reach "a" or "b", check if the edge part of an outline or a hole
                        If it's an outline, then it's fine, because "outside triangle" -> "outline edge" -> "inside triangle" is valid
                        If it's a hole, then check if the hole's index is the same as the triangle's parent polyline index
                        Since the current triangle is not inside any polyline, its parent polyline index is nullopt, so the indices are not the same
                        So if either "a" or "b" is a hole, we can detect it, and fail the triangulation
                        */

                        EdgeType currentPolylineType = _polylines[(int)polylineIndex.Value].type;
                        if (currentPolylineType == EdgeType.AutoDetect)
                        {
                            // Current type is auto-detect, check if we already already know the actual type
                            EdgeType autoDetectedType = _autoDetectedPolylineTypes[(int)polylineIndex.Value];
                            if (autoDetectedType == EdgeType.AutoDetect)
                            {
                                // Type is not known yet, so just set to the opposite
                                // So if we are outside, then classify as outline
                                // If we are inside, then classify as hole
                                autoDetectedType = currentTriangleIsInterior ? EdgeType.Hole : EdgeType.Outline;
                                _autoDetectedPolylineTypes[(int)polylineIndex.Value] = autoDetectedType;
                            }
                            // Else already classified

                            currentPolylineType = autoDetectedType;
                        }

                        bool currentEdgeIsHole = currentPolylineType == EdgeType.Hole;
                        if (currentTriangleIsInterior == currentEdgeIsHole)
                        {
                            // Simple case, we are outside and crossing an outline, or we are inside and crossing a hole (this is always valid)

                            // Also update the parent for the current polyline
                            _parentPolylines[(int)polylineIndex.Value] = currentTriangleParentPolylineIndex;

                            // Just set index, which implicitly sets the opposite location
                            neighborTriangleParentPolylineIndex = polylineIndex;
                        }
                        else
                        {
                            // We are outside and crossing a hole, or we are inside and crossing an outline
                            if (currentTriangleParentPolylineIndex.HasValue && currentTriangleParentPolylineIndex.Value == polylineIndex.Value)
                            {
                                // Valid case, the neighbor triangle is the opposite location
                                // Set its polyline index to the current polyline's parent

                                neighborTriangleParentPolylineIndex = _parentPolylines[(int)polylineIndex.Value];
                            }
                            else
                            {
                                // Invalid case, either an outline is inside another outline, or a hole is inside another hole
                                return Fail(new TE_StackedPolylines(currentEdgeIsHole));
                            }
                        }
                    }
                    else
                    {
                        // Simple case, we are not crossing outlines or holes, so just propagate the current location
                        neighborTriangleParentPolylineIndex = currentTriangleParentPolylineIndex;
                    }

                    TriangleWithData resultTriangle = _resultTriangles[(int)neighborTriangle.index.Value];
                    resultTriangle.data.locationDataType = TriangleData.LocationDataType.Known;
                    resultTriangle.data.parentPolylineIndex = neighborTriangleParentPolylineIndex;
                    _resultTriangles[(int)neighborTriangle.index.Value] = resultTriangle;

                    trianglesToCheck.Push(neighborTriangle);
                }
            }

            return true;
        }

        private TriangleLocation GetTriangleLocation(TriangleData data)
        {
#if DEBUG
            if (data.locationDataType != TriangleData.LocationDataType.Known)
            {
                Detail.ThrowAssertionFailedError();
            }
#endif

            if (!data.parentPolylineIndex.HasValue)
            {
                return TriangleLocation.ConvexHull;
            }
            else
            {
                NullableIdx parentPolylineIndex = data.parentPolylineIndex;
#if DEBUG
                if ((int)parentPolylineIndex.Value >= _polylines.Count)
                {
                    Detail.ThrowAssertionFailedError();
                }
#endif
                EdgeType polylineType = _polylines[(int)parentPolylineIndex.Value].type;
                if (polylineType == EdgeType.AutoDetect)
                {
                    polylineType = _autoDetectedPolylineTypes[(int)parentPolylineIndex.Value];
#if DEBUG
                    if (polylineType != EdgeType.Outline && polylineType != EdgeType.Hole)
                    {
                        Detail.ThrowAssertionFailedError();
                    }
#endif
                }

                return polylineType == EdgeType.Hole ? TriangleLocation.Hole : TriangleLocation.Interior;
            }
        }

        private IEnumerable<TriangleWithData> EnumerateAllTrianglesWithData(bool cwTriangles, Func<TriangleWithData, bool> filter)
        {
            bool flipped = !cwTriangles;

            TriangleWithData FlipIfNeeded(TriangleWithData data)
            {
                if (flipped)
                {
                    Triangle tri = data.triangle;
                    Idx tmp = tri.y;
                    tri.y = tri.z;
                    tri.z = tmp;
                    data.triangle = tri;
                }

                return data;
            }

            if (filter != null)
            {
                return _resultTriangles.Where(filter).Select(FlipIfNeeded);
            }
            else
            {
                return _resultTriangles.Select(FlipIfNeeded);
            }
        }

        private HalfEdgeIndex GetBoundaryEdgeOfVertex(VertexIndex vertex)
        {
            // Since only one half of an edge is boundary, there should be only one boundary edge here

            HalfEdgeIndex boundaryEdge = NullEdge;

            _topology.ForEachEdgeOfVertex(vertex, (HalfEdgeIndex halfEdge) =>
            {
                if (_topology.GetEdgeData(halfEdge).IsBoundary)
                {
                    boundaryEdge = halfEdge;
                    return false;
                }

                return true;
            });

            return boundaryEdge;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Math.Orientation Orient2d(Vec2 a, Vec2 b, Vec2 c)
        {
            return Math.Orient2d(_pred, a, b, c);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Math.CircleLocation Incircle(Vec2 a, Vec2 b, Vec2 c, Vec2 d)
        {
            return Math.Incircle(_pred, a, b, c, d);
        }
    }
}

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
