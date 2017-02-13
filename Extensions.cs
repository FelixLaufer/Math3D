using System;

namespace Math3D
{
    public static class Math3DExtensions
    {
        public static readonly double Deg2Rad = Math.PI / 180d;
        public static readonly double Rad2Deg = 180d / Math.PI;

        public static double ToDeg(this double radians)
        {
            return radians * Rad2Deg;
        }
        public static double ToRad(this double degrees)
        {
            return degrees * Deg2Rad;
        }
    }
}
