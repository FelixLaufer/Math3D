using System;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;


namespace Math3D
{
    public class Vector3D : IEquatable<Vector3D>
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public Vector3D(Vector3D v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }
        public Vector3D(Vector<double> v) : this (v.ToArray())
        { }
        public Vector3D(double[] data)
        {
            if (data.Length == 3)
            {
                X = data[0];
                Y = data[1];
                Z = data[2];
            }
            else
                throw new InvalidParameterException(1);
        }
        public Vector3D(double x = 0, double y = 0, double z = 0)
        {
            X = x;
            Y = y;
            Z = z;
        }

        #region Operators
        public static Vector3D operator *(double s, Vector3D v)
        {
            return new Vector3D(s * v.ToVector());
        }
        public static Vector3D operator *(Vector3D v, double s)
        {
            return new Vector3D(v.ToVector() * s);
        }
        public static Vector3D operator +(Vector3D v1, Vector3D v2)
        {
            return new Vector3D(v1.ToVector() + v2.ToVector());
        }
        public static Vector3D operator -(Vector3D v)
        {
            return -1 * v;
        }
        public static Vector3D operator -(Vector3D v1, Vector3D v2)
        {
            return new Vector3D(v1.ToVector() - v2.ToVector());
        }
        public static bool operator ==(Vector3D v1, Vector3D v2)
        {
            if (ReferenceEquals(v1, null))
                return ReferenceEquals(v2, null);
            if (ReferenceEquals(v2, null))
                return ReferenceEquals(v1, null);

            return v1.ToVector() == v2.ToVector();
        }
        public static bool operator !=(Vector3D v1, Vector3D v2)
        {
            return v1.ToVector() != v2.ToVector();
        }
        public bool Equals(Vector3D v)
        {
            return ToVector().Equals(v.ToVector());
        }
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj))
                return false;

            if (ReferenceEquals(this, obj))
                return true;

            return obj.GetType() == GetType() && Equals((Vector3D)obj);
        }
        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = ToVector().GetHashCode();
                hashCode = (hashCode * 397) ^ X.GetHashCode();
                return hashCode;
            }
        }
        #endregion

        #region Public methods
        public double Abs()
        {
            return Norm();
        }
        public Vector3D Add(double s)
        {
            return new Vector3D(ToVector().Add(s)); 
        }
        public Vector3D Add(Vector3D v)
        {
            return new Vector3D(ToVector().Add(v.ToVector()));
        }
        public Vector3D CrossProduct(Vector3D v)
        {
            return CrossProduct(this, v);
        }
        public static Vector3D CrossProduct(Vector3D v1, Vector3D v2)
        {
            return new Vector3D(v1.Y * v2.Z - v1.Z * v2.Y, -v1.X * v2.Z + v1.Z * v2.X, v1.X * v2.Y - v1.Y * v2.X);
        }
        public double Distance(Vector3D v)
        {
            return (this - v).Norm();
        }
        public static double Distance (Vector3D v1, Vector3D v2)
        {
            return v1.Distance(v2);
        }
        public Vector3D Divide(double s)
        {
            return new Vector3D(ToVector().Divide(s));
        }
        public Vector3D DivideByThis(double s)
        {
            return new Vector3D(ToVector().DivideByThis(s));
        }
        public double DotProduct(Vector3D v)
        {
            return ToVector().DotProduct(v.ToVector());
        }
        public static double DotProduct(Vector3D v1, Vector3D v2)
        {
            return v1.DotProduct(v2);
        }
        public double Length()
        {
            return Norm();
        }
        public Vector3D Multiply(double s)
        {
            return new Vector3D(ToVector().Multiply(s));
        }
        public double Norm()
        {
            return Math.Sqrt(X * X + Y * Y + Z * Z);
        }
        public Vector3D Normalize()
        {
            var norm = Norm();
            return (norm != 0d) ? 1d / norm * this : Null();
        }
        public static Vector3D Null()
        {
            return new Vector3D(0, 0, 0);
        }
        public Vector3D RotateByThis(Quaternion q)
        {
            return q.RotateThis(this);
        }
        public Vector3D ProjectOnto(Vector3D v)
        {
            return Projection(this, v);
        }
        public Vector3D RejectOnto(Vector3D v)
        {
            return Rejection(this, v);
        }
        public static Vector3D Projection(Vector3D v1, Vector3D v2)
        {
            return DotProduct(v1, v2) / DotProduct(v2, v2) * v2;
        }
        public static Vector3D Rejection(Vector3D v1, Vector3D v2)
        {
            return v1 - Projection(v1, v2);
        }
        public Vector3D Subtract(double s)
        {
            return new Vector3D(ToVector().Subtract(s));
        }
        public Vector3D Subtract(Vector3D v)
        {
            return new Vector3D(ToVector().Subtract(v.ToVector()));
        }
        public Quaternion ToQuaternion()
        {
            return new Quaternion(0, X, Y, Z);
        }
        public override string ToString()
        {
            return ToVector().ToString();
        }
        public Vector<double> ToVector()
        {
            return Vector<double>.Build.DenseOfArray(new[] { X, Y, Z });
        }
        public static Vector3D UnitX()
        {
            return new Vector3D(1, 0, 0);
        }
        public static Vector3D UnitY()
        {
            return new Vector3D(0, 1, 0);
        }
        public static Vector3D UnitZ()
        {
            return new Vector3D(0, 0, 1);
        }
        #endregion
    }
    public class Matrix3D : IEquatable<Matrix3D>
    {
        public double[,] Storage { get; private set; } = new double[3, 3];
        public Matrix3D(Matrix3D m)
            : this (m.Storage)
        { }
        public Matrix3D(Matrix<double> m)
        {
            if (m.RowCount == 3 && m.ColumnCount == 3)
                Storage = m.Storage.ToArray();
            else
                throw new InvalidParameterException(1);
        }
        public Matrix3D(Vector3D[] columnVectors3D)
        {
            if (columnVectors3D.Length == 3)
                Storage = new[,] {
                    { columnVectors3D[0].X, columnVectors3D[1].X, columnVectors3D[2].X },
                    { columnVectors3D[0].Y, columnVectors3D[1].Y, columnVectors3D[2].Y },
                    { columnVectors3D[0].Z, columnVectors3D[1].Z, columnVectors3D[2].Z }
                };
            else
                throw new InvalidParameterException(1);
        }
        public Matrix3D(double[,] data)
        {
            if (data.GetLength(0) == 3 && data.GetLength(1) == 3)
                Storage = data;
            else
                throw new InvalidParameterException(1);
        }
        public Matrix3D(double m11 = 0, double m12 = 0, double m13 = 0, double m21 = 0, double m22 = 0, double m23 = 0, double m31 = 0, double m32 = 0, double m33 = 0)
        {
            Storage = new[,] { { m11, m12, m13 }, { m21, m22, m23 }, { m31, m32, m33 } };
        }

        #region Operators
        public static Matrix3D operator *(double s, Matrix3D m)
        {
            return new Matrix3D(s * m.ToMatrix());
        }
        public static Matrix3D operator *(Matrix3D m, double s)
        {
            return new Matrix3D(m.ToMatrix() * s);
        }
        public static Vector3D operator *(Matrix3D m, Vector3D v)
        {
            return new Vector3D(m.ToMatrix() * v.ToVector());
        }
        public static Vector3D operator *(Vector3D v, Matrix3D m)
        {
            return new Vector3D(v.ToVector() * m.ToMatrix());
        }
        public static Matrix3D operator *(Matrix3D m1, Matrix3D m2)
        {
            return new Matrix3D(m1.ToMatrix() * m2.ToMatrix());
        }
        public static Matrix3D operator +(double s, Matrix3D m)
        {
            return new Matrix3D(s + m.ToMatrix());
        }
        public static Matrix3D operator +(Matrix3D m, double s)
        {
            return new Matrix3D(m.ToMatrix() + s);
        }
        public static Matrix3D operator +(Matrix3D m1, Matrix3D m2)
        {
            return new Matrix3D(m1.ToMatrix() + m2.ToMatrix());

        }
        public static Matrix3D operator -(Matrix3D m)
        {
            return -1 * m;
        }
        public static Matrix3D operator -(double s, Matrix3D m)
        {
            return new Matrix3D(s - m.ToMatrix());
        }
        public static Matrix3D operator -(Matrix3D m, double s)
        {
            return new Matrix3D(m.ToMatrix() - s);
        }
        public static Matrix3D operator -(Matrix3D m1, Matrix3D m2)
        {
            return new Matrix3D(m1.ToMatrix() - m2.ToMatrix());
        }
        public static bool operator ==(Matrix3D m1, Matrix3D m2)
        {
            return m1.ToMatrix() == m2.ToMatrix();
        }
        public static bool operator !=(Matrix3D m1, Matrix3D m2)
        {
            return m1.ToMatrix() != m2.ToMatrix();
        }
        public bool Equals(Matrix3D m)
        {
            return ToMatrix().Equals(m.ToMatrix());
        }
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj))
                return false;

            if (ReferenceEquals(this, obj))
                return true;

            return obj.GetType() == GetType() && Equals((Matrix3D)obj);
        }
        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = ToMatrix().GetHashCode();
                hashCode = (hashCode * 397) ^ Storage[0, 0].GetHashCode();
                return hashCode;
            }
        }
        #endregion

        #region Public methods
        public double this[int row, int column]
        {
            get
            {
                return Storage[row, column];
            }
        }
        public Matrix3D Add(double s)
        {
            return new Matrix3D(ToMatrix().Add((s)));
        }
        public Matrix3D Add(Matrix3D m)
        {
            return new Matrix3D(ToMatrix().Add(m.ToMatrix()));
        }
        public void ApplyPointwise(Func<double, double> func)
        {
            for (var i = 0; i < Storage.GetLength(0); ++i)
                for (var j = 0; j < Storage.GetLength(1); ++j)
                    Storage[i, j] = func(Storage[i, j]);
        }
        public static Matrix3D Identity()
        {
            return new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1);
        }
        public Matrix3D Inverse()
        {
            return new Matrix3D(ToMatrix().Inverse());
        }
        public double Determinant()
        {
            return ToMatrix().Determinant();
        }
        public Vector3D Diagonal()
        {
            return new Vector3D(ToMatrix().Diagonal().ToArray());
        }
        public Matrix3D Divide(double s)
        {
            return new Matrix3D(ToMatrix().Divide(s));
        }
        public Matrix3D DivideByThis(double s)
        {
            return new Matrix3D(ToMatrix().DivideByThis(s));
        }
        public Vector3D LeftMultiply(Vector3D v)
        {
            return new Vector3D(ToMatrix().LeftMultiply(v.ToVector()));
        }
        public Matrix3D Multiply(double s)
        {
            return new Matrix3D(ToMatrix().Multiply(s));
        }
        public Vector3D Multiply(Vector3D v)
        {
            return new Vector3D(ToMatrix().Multiply(v.ToVector()));
        }
        public Matrix3D Multiply(Matrix3D m)
        {
            return new Matrix3D(ToMatrix().Multiply(m.ToMatrix()));
        }
        public Matrix3D PointwiseMultiply(Matrix3D m)
        {
            return new Matrix3D(ToMatrix().PointwiseMultiply(m.ToMatrix()));
        }
        public Matrix3D Subtract(double s)
        {
            return new Matrix3D(ToMatrix().Subtract(s));
        }
        public Matrix3D Subtract(Matrix3D m)
        {
            return new Matrix3D(ToMatrix().Subtract(m.ToMatrix()));
        }
        public static Matrix3D RotX(double alpha)
        {
            var rotX = new Matrix3D(1, 0, 0, 0, Math.Cos(alpha), -Math.Sin(alpha), 0, Math.Sin(alpha), Math.Cos(alpha));
            rotX.ApplyPointwise(i => Math.Abs(i) < Config.Epsilon ? 0 : Math.Abs(i - 1d) < Config.Epsilon ? 1d : i);
            return rotX;
        }
        public static Matrix3D RotY(double alpha)
        {
            var rotY = new Matrix3D(Math.Cos(alpha), 0, Math.Sin(alpha), 0, 1, 0, -Math.Sin(alpha), 0, Math.Cos(alpha));
            rotY.ApplyPointwise(i => Math.Abs(i) < Config.Epsilon ? 0 : Math.Abs(i - 1d) < Config.Epsilon ? 1d : i);
            return rotY;
        }
        public static Matrix3D RotZ(double alpha)
        {
            var rotZ = new Matrix3D(Math.Cos(alpha), -Math.Sin(alpha), 0, Math.Sin(alpha), Math.Cos(alpha), 0, 0, 0, 1);
            rotZ.ApplyPointwise(i => Math.Abs(i) < Config.Epsilon ? 0 : Math.Abs(i - 1d) < Config.Epsilon ? 1d : i);
            return rotZ;
        }
        public Matrix<double> ToMatrix()
        {
            return Matrix<double>.Build.DenseOfArray(Storage);
        }
        public override string ToString()
        {
            return ToMatrix().ToString();
        }
        public Quaternion ToQuaternion()
        {
            var r11 = Storage[0, 0];
            var r12 = Storage[0, 1];
            var r13 = Storage[0, 2];
            var r21 = Storage[1, 0];
            var r22 = Storage[1, 1];
            var r23 = Storage[1, 2];
            var r31 = Storage[2, 0];
            var r32 = Storage[2, 1];
            var r33 = Storage[2, 2];

            var w = (r11 + r22 + r33 + 1.0d) / 4.0d;
            var x = (r11 - r22 - r33 + 1.0d) / 4.0d;
            var y = (-r11 + r22 - r33 + 1.0d) / 4.0d;
            var z = (-r11 - r22 + r33 + 1.0d) / 4.0d;

            if (w < 0) w = 0.0d;
            if (x < 0) x = 0.0d;
            if (y < 0) y = 0.0d;
            if (z < 0) z = 0.0d;

            w = Math.Sqrt(w);
            x = Math.Sqrt(x);
            y = Math.Sqrt(y);
            z = Math.Sqrt(z);

            if (w >= x && w >= y && w >= z)
            {
                w *= 1.0d;
                x *= Math.Sign(r32 - r23);
                y *= Math.Sign(r13 - r31);
                z *= Math.Sign(r21 - r12);
            }
            else if (x >= w && x >= y && x >= z)
            {
                w *= Math.Sign(r32 - r23);
                x *= 1.0d;
                y *= Math.Sign(r21 + r12);
                z *= Math.Sign(r13 + r31);
            }
            else if (y >= w && y >= x && y >= z)
            {
                w *= Math.Sign(r13 - r31);
                x *= Math.Sign(r21 + r12);
                y *= 1.0d;
                z *= Math.Sign(r32 + r23);
            }
            else if (z >= w && z >= x && z >= y)
            {
                w *= Math.Sign(r21 - r12);
                x *= Math.Sign(r31 + r13);
                y *= Math.Sign(r32 + r23);
                z *= 1.0d;
            }
            else
                throw new InvalidOperationException("Cannot convert matrix to quaternion");

            var quaternion = new Quaternion(w, x, y, z);
            quaternion.Normalize();
            return quaternion;
        }
        public Matrix3D Transpose()
        {
            return new Matrix3D(ToMatrix().Transpose());
        }
        #endregion
    }
    public class Quaternion
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double W { get; set; }
        public Quaternion(Quaternion q)
        {
            W = q.W;
            X = q.X;
            Y = q.Y;
            Z = q.Z;
        }
        public Quaternion(Vector3D v, double w = 0)
        {
            W = w;
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }
        public Quaternion(AxisAngle axisAngle)
            : this(axisAngle.ToQuaternion())
        { }
        public Quaternion(Vector<double> v) 
            : this(v.ToArray())
        { }
        public Quaternion(double[] data)
        {
            if (data.Length == 4)
            {
                W = data[0];
                X = data[1];
                Y = data[2];
                Z = data[3];
            }
            else
                throw new InvalidParameterException(1);
        }
        public Quaternion(double w = 0, double x = 0, double y = 0, double z = 0)
        {
            W = w;
            X = x;
            Y = y;
            Z = z;
        }

        #region Operators
        public static Quaternion operator *(double s, Quaternion q)
        {
            return q.Multiply(s);
        }
        public static Quaternion operator *(Quaternion q, double s)
        {
            return s * q;
        }
        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            return q1.Multiply(q2);
        }
        public static Quaternion operator +(Quaternion q1, Quaternion q2)
        {
            return q1.Add(q2);
        }
        public static Quaternion operator -(Quaternion q)
        {
            return -1 * q;
        }
        public static Quaternion operator -(Quaternion q1, Quaternion q2)
        {
            return q1.Subtract(q2);
        }
        public static bool operator ==(Quaternion q1, Quaternion q2)
        {
            return q1.W == q2.W && q1.X == q2.X && q1.Y == q2.Y && q1.Z == q2.Z;
        }
        public static bool operator !=(Quaternion q1, Quaternion q2)
        {
            return !(q1 == q2);
        }
        public bool Equals(Quaternion q)
        {
            return W == q.W && X == q.X && Y == q.Y && Z == q.Z;
        }
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj))
                return false;

            if (ReferenceEquals(this, obj))
                return true;

            return obj.GetType() == GetType() && Equals((Quaternion)obj);
        }
        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = W.GetHashCode();
                hashCode = (hashCode * 397) ^ X.GetHashCode();
                hashCode = (hashCode * 397) ^ Y.GetHashCode();
                hashCode = (hashCode * 397) ^ Z.GetHashCode();
                return hashCode;
            }
        }
        #endregion

        #region Public methods
        public Quaternion Add(Quaternion q)
        {
            return new Quaternion(W + q.W, X + q.X, Y + q.Y, Z + q.Z);
        }
        public Quaternion Conjugate()
        {
            return new Quaternion(W, -X, -Y, -Z);
        }
        public void SwingTwistDecompose(Vector3D direction, out Quaternion swing, out Quaternion twist)
        {
            SwingTwistDecomposition(this, direction, out swing, out twist);
        }
        public static void SwingTwistDecomposition(Quaternion q, Vector3D direction, out Quaternion swing, out Quaternion twist)
        {
            swing = Identity();
            twist = new Quaternion(q);
            var projection = q.VectorPart.ProjectOnto(direction);
            twist = new Quaternion(projection, q.W).Normalize();
            if (twist.Norm() < Config.Epsilon)
                swing = new AxisAngle(direction, Math.PI).ToQuaternion();
            else
                swing = q * twist.Conjugate();
        }
        public static Quaternion Identity()
        {
            return new Quaternion(1, 0, 0, 0);
        }
        public Quaternion Inverse()
        {
            return (1d / Norm()) * Conjugate();
        }
        public Quaternion Subtract(Quaternion q)
        {
            return new Quaternion(W - q.W, X - q.X, Y - q.Y, Z - q.Z);
        }
        public Quaternion Multiply(double s)
        {
            return new Quaternion(W * s, X * s, Y * s, Z * s);
        }
        public Quaternion Multiply(Quaternion q)
        {
            var w = -X * q.X - Y * q.Y - Z * q.Z + W * q.W;
            var x = X * q.W + Y * q.Z - Z * q.Y + W * q.X;
            var y = -X * q.Z + Y * q.W + Z * q.X + W * q.Y;
            var z = X * q.Y - Y * q.X + Z * q.W + W * q.Z;
            return new Quaternion(w, x, y, z);
        }
        public double Norm()
        {
            return Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
        }
        public Quaternion Normalize()
        {
            var norm = Norm();
            return (norm != 0d) ? (1d / norm) * this : Null();
        }
        public static Quaternion Null()
        {
            return new Quaternion(0, 0, 0, 0);
        }
        public double Real
        {
            get
            {
                return W;
            }
        }
        public Vector3D RotateThis(Vector3D v)
        {
            return ((this * v.ToQuaternion()) * Conjugate()).VectorPart;
        }
        public static Quaternion RotationBetween(Quaternion q1, Quaternion q2)
        {
            return q1.Inverse() * q2;
        }
        public static Quaternion RotationBetween(Vector3D v1, Vector3D v2)
        {
            v1 = v1.Normalize();
            v2 = v2.Normalize();
            var d = v1.DotProduct(v2);

            if (d >= 1d)
                return Identity();

            if (d < (Config.Epsilon - 1d))
            {
                var axis = new Vector3D(1, 0, 0).CrossProduct(v1);
                if (axis.Length() < Config.Epsilon)
                    axis = new Vector3D(0, 1, 0).CrossProduct(v1);

                axis = axis.Normalize();
                return new AxisAngle(axis, Math.PI).ToQuaternion();
            }

            var s = Math.Sqrt((1d + d) * 2d);
            var c = v1.CrossProduct(v2);
            var invs = 1d / s;

            var w = s * 0.5d;
            var x = c.X * invs;
            var y = c.Y * invs;
            var z = c.Z * invs;

            return new Quaternion(w, x, y, z).Normalize();
        }
        public AxisAngle ToAxisAngle()
        {
            var normalizedQuaternion = Normalize();
            var angle = 2d * Math.Acos(normalizedQuaternion.W);
            var axis = new Vector3D();
            var s = Math.Sqrt(1d - normalizedQuaternion.W * normalizedQuaternion.W);

            if (s < Config.Epsilon)
            {
                axis.X = 1;
                axis.Y = 0;
                axis.Z = 0;
            }
            else
            {
                axis.X = normalizedQuaternion.X / s;
                axis.Y = normalizedQuaternion.Y / s;
                axis.Z = normalizedQuaternion.Z / s;
            }

            return new AxisAngle(axis, angle);
        }
        public EulerAngles ToEulerAngles(string sequence = "ZYX")
        {
            return new EulerAngles(this, sequence);
        }
        public Matrix3D ToMatrix3D()
        {
            var sqw = W * W;
            var sqx = X * X;
            var sqy = Y * Y;
            var sqz = Z * Z;

            var invs = 1d / (sqx + sqy + sqz + sqw);

            var m = new double[3, 3];
            m[0, 0] = (sqx - sqy - sqz + sqw) * invs;
            m[1, 1] = (-sqx + sqy - sqz + sqw) * invs;
            m[2, 2] = (-sqx - sqy + sqz + sqw) * invs;
            var tmp1 = X * Y;
            var tmp2 = Z * W;
            m[1, 0] = 2d * (tmp1 + tmp2) * invs;
            m[0, 1] = 2d * (tmp1 - tmp2) * invs;
            tmp1 = X * Z;
            tmp2 = Y * W;
            m[2, 0] = 2d * (tmp1 - tmp2) * invs;
            m[0, 2] = 2d * (tmp1 + tmp2) * invs;
            tmp1 = Y * Z;
            tmp2 = X * W;
            m[2, 1] = 2d * (tmp1 + tmp2) * invs;
            m[1, 2] = 2d * (tmp1 - tmp2) * invs;

            return new Matrix3D(m);
        }
        public override string ToString()
        {
            return W + ", " + X + ", " + Y + ", " + Z;
        }
        public Vector3D VectorPart
        {
            get
            {
                return new Vector3D(X, Y, Z);
            }
        }
        #endregion
    }
    public class AxisAngle
    {
        public Vector3D Axis { get; set; }
        public double Angle { get; set; }
        public AxisAngle(AxisAngle axisAngle)
        {
            Axis = axisAngle.Axis;
            Angle = axisAngle.Angle;
        }
        public AxisAngle(Quaternion q) : this(q.ToAxisAngle())
        { }
        public AxisAngle(Vector3D axis, double angle)
        {
            var normalizedAxis = axis.Normalize();
            Axis = normalizedAxis;
            Angle = angle;
        }

        #region Public methods
        public override string ToString()
        {
            return Axis.ToString() + ", " + Angle;
        }
        public Quaternion ToQuaternion()
        {
            var axisNormalized = Axis * Math.Sin(Angle / 2d);
            var w = Math.Cos(Angle / 2d);
            var x = Axis.X;
            var y = Axis.Y;
            var z = Axis.Z;
            return new Quaternion(w, x, y, z);
        }
        #endregion
    }
    public class EulerAngles
    {
        public const string StandardSequence = "ZYX";
        public string Sequence { get; set; } = StandardSequence;
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public EulerAngles(EulerAngles e)
            : this(e.X, e.Y, e.Z, e.Sequence)
        { }
        public EulerAngles(Quaternion q, string sequence = StandardSequence)
            : this(q.ToMatrix3D(), sequence)
        { }
        public EulerAngles(Matrix3D r, string sequence = StandardSequence)
        {
            Sequence = sequence.ToUpper();
            double x, y, z = 0d;
            Convert(r, sequence, out x, out y, out z);
            X = x;
            Y = y;
            Z = z;
        }
        public EulerAngles(Vector3D data, string sequence = StandardSequence)
            : this(data.X, data.Y, data.Z, sequence)
        { }
        public EulerAngles(double[] data, string sequence = StandardSequence)
            : this(new Vector3D(data), StandardSequence)
        { }
        public EulerAngles(double x = 0, double y = 0, double z = 0, string sequence = StandardSequence)
        {
            Sequence = sequence;
            X = x;
            Y = y;
            Z = z;
        }
        private static void Convert(Matrix3D r, string sequence, out double x, out double y, out double z)
        {
            x = 0d;
            y = 0d;
            z = 0d;
            switch (sequence.ToUpper())
            {
                case "XYZ":
                    if (r[0, 2] < 1d)
                    {
                        if (r[0, 2] > -1d)
                        {
                            y = Math.Asin(r[0, 2]);
                            x = Math.Atan2(-r[1, 2], r[2, 2]);
                            z = Math.Atan2(-r[0, 1], r[0, 0]);
                        }
                        else
                        {
                            y = -Math.PI / 2d;
                            x = -Math.Atan2(r[1, 0], r[1, 1]);
                            z = 0d;
                        }
                    }
                    else
                    {
                        y = Math.PI / 2d;
                        x = Math.Atan2(r[1, 0], r[1, 1]);
                        z = 0d;
                    }
                    break;
                case "XZY":
                    if (r[0, 1] < 1d)
                    {
                        if (r[0, 1] > -1d)
                        {
                            z = Math.Asin(-r[0, 1]);
                            x = Math.Atan2(r[2, 1], r[1, 1]);
                            y = Math.Atan2(r[0, 2], r[0, 0]);
                        }
                        else
                        {
                            z = Math.PI / 2d;
                            x = -Math.Atan2(-r[2, 0], r[2, 2]);
                            y = 0d;
                        }
                    }
                    else
                    {
                        z = -Math.PI / 2d;
                        x = Math.Atan2(-r[2, 0], r[2, 2]);
                        y = 0d;
                    }
                    break;
                case "YXZ":
                    if (r[1, 2] < 1d)
                    {
                        if (r[1, 2] > -1d)
                        {
                            x = Math.Asin(-r[1, 2]);
                            y = Math.Atan2(r[0, 2], r[2, 2]);
                            z = Math.Atan2(r[1, 0], r[1, 1]);
                        }
                        else
                        {
                            x = Math.PI / 2d;
                            y = -Math.Atan2(-r[0, 1], r[0, 0]);
                            z = 0d;
                        }
                    }
                    else
                    {
                        x = -Math.PI / 2d;
                        y = Math.Atan2(-r[0, 1], r[0, 0]);
                        z = 0d;
                    }
                    break;
                case "YZX":
                    if (r[1, 0] < 1d)
                    {
                        if (r[1, 0] > -1d)
                        {
                            z = Math.Asin(r[1, 0]);
                            y = Math.Atan2(-r[2, 0], r[0, 0]);
                            x = Math.Atan2(-r[1, 2], r[1, 1]);
                        }
                        else
                        {
                            z = -Math.PI / 2d;
                            y = -Math.Atan2(r[2, 1], r[2, 2]);
                            x = 0d;
                        }
                    }
                    else
                    {
                        z = Math.PI / 2d;
                        y = Math.Atan2(r[2, 1], r[2, 2]);
                        x = 0d;
                    }
                    break;
                case "ZXY":
                    if (r[2, 1] < 1d)
                    {
                        if (r[2, 1] > -1d)
                        {
                            x = Math.Asin(r[2, 1]);
                            z = Math.Atan2(-r[0, 1], r[1, 1]);
                            y = Math.Atan2(-r[2, 0], r[2, 2]);
                        }
                        else
                        {
                            x = -Math.PI / 2d;
                            z = -Math.Atan2(r[0, 2], r[0, 0]);
                            y = 0d;
                        }
                    }
                    else
                    {
                        x = Math.PI / 2d;
                        z = Math.Atan2(r[0, 2], r[0, 0]);
                        y = 0d;
                    }
                    break;
                case "ZYX":
                    if (r[2, 0] < 1d)
                    {
                        if (r[2, 0] > -1d)
                        {
                            y = Math.Asin(-r[2, 0]);
                            z = Math.Atan2(r[1, 0], r[0, 0]);
                            x = Math.Atan2(r[2, 1], r[2, 2]);
                        }
                        else
                        {
                            y = Math.PI / 2d;
                            z = -Math.Atan2(-r[1, 2], r[1, 1]);
                            x = 0d;
                        }
                    }
                    else
                    {
                        y = -Math.PI / 2d;
                        z = Math.Atan2(-r[1, 2], r[1, 1]);
                        x = 0d;
                    }              
                break;
                default:
                    throw new InvalidParameterException(2);
            }

        }
        public EulerAngles ToEulerAngles(string sequence = StandardSequence)
        {
            if (Sequence == sequence.ToUpper())
                return new EulerAngles(this);
            return new EulerAngles(ToMatrix3D(), sequence);
        }
        public Matrix3D ToMatrix3D()
        {
            var rotX = Matrix3D.RotX(X);
            var rotY = Matrix3D.RotY(Y);
            var rotZ = Matrix3D.RotZ(Z);
            var rot = Matrix3D.Identity();
            switch(Sequence)
            {
                case "XYZ":
                    rot = rotX * rotY * rotZ;
                    break;
                case "XZY":
                    rot = rotX * rotZ * rotY;
                    break;
                case "YXZ":
                    rot = rotY * rotX * rotZ;
                    break;
                case "YZX":
                    rot = rotY * rotZ * rotX;
                    break;
                case "ZXY":
                    rot = rotZ * rotX * rotY;
                    break;
                case "ZYX":
                    rot = rotZ * rotY * rotX;
                    break;
                default:
                    throw new InvalidParameterException(2);
            }
            return rot;
        }
        public Quaternion ToQuaternion()
        {
            return ToMatrix3D().ToQuaternion();
        }
    }
    public class Transform3D
    {
        public Vector3D Pos { get; set; }
        public Quaternion Quat { get; set; }
        public Transform3D(Transform3D t)
            : this (t.Pos, t.Quat)
        { }
        public Transform3D(Vector<double> v) : this(v.ToArray())
        { }
        public Transform3D(Vector3D pos, Quaternion quat)
        {
            Pos = new Vector3D(pos);
            Quat = new Quaternion(quat);
        }
        public Transform3D(double[] data)
        {
            if (data.Length == 7)
            {
                Pos = new Vector3D(data[0], data[1], data[2]);
                Quat = new Quaternion(data[3], data[4], data[5], data[6]);
            }
            else
                throw new InvalidParameterException(1);
        }
        public Transform3D(double x = 0, double y = 0, double z = 0, double w = 1, double imagX = 0, double imagY = 0, double imagZ = 0)
            : this(new Vector3D(x, y, z), new Quaternion(w, imagX, imagY, imagZ))
        { }

        #region Public methods
        public static Transform3D Identity()
        {
            return new Transform3D(Vector3D.Null(), Quaternion.Identity());
        }
        public Matrix4D ToMatrix4D()
        {
            return new Matrix4D(Quat.ToMatrix3D(), Pos);
        }
        public override string ToString()
        {
            return Pos.X + ", " + Pos.Y + ", " + Pos.Z + ", " + Quat.W + ", " + Quat.X + ", " + Quat.Y + ", " + Quat.Z;
        }
        #endregion
    }
    public class Matrix4D : Matrix3D
    {
        public new double[,] Storage { get; private set; } = new double[4, 4];
        public Matrix4D(Matrix4D m)
            : this (m.Storage)
        { }
        public Matrix4D(Matrix<double> m)
        {
            if (m.RowCount == 4 && m.ColumnCount == 4)
                Storage = m.Storage.ToArray();
            else
                throw new InvalidParameterException(1);
        }
        public Matrix4D(Matrix3D rotation, Vector3D translation = null, Vector3D shear = null, double scale = 1)
        {
            Rotation = rotation;
            Translation = (translation == null) ? new Vector3D(0, 0, 0) : translation;
            Shear = (shear == null) ? new Vector3D(0, 0, 0) : shear;
            Scale = scale;
        }
        public Matrix4D(double[,] data)
        {
            if (data.GetLength(0) == 4 && data.GetLength(1) == 4)
                Storage = data;
            else
                throw new InvalidParameterException(1);
        }
        public Matrix4D(double m11 = 1, double m12 = 0, double m13 = 0, double m14 = 0, double m21 = 0, double m22 = 1, double m23 = 0, double m24 = 0, double m31 = 0, double m32 = 0, double m33 = 1, double m34 = 0, double m41 = 0, double m42 = 0, double m43 = 0, double m44 = 1)
        {
            Storage = new[,] { { m11, m12, m13 , m14}, { m21, m22, m23 , m24}, { m31, m32, m33 , m34}, { m41, m42, m43, m44 } };
        }

        #region Operators
        public static Matrix4D operator +(Matrix4D m, Vector3D v)
        {
            return m.Add(v);
        }
        public static Matrix4D operator -(Matrix4D m, Vector3D v)
        {
            return m.Subtract(v);
        }
        #endregion

        #region Public methods
        public Matrix4D Add(Vector3D v)
        {
            return new Matrix4D(Rotation, Translation + v, Shear, Scale);
        }
        public new Vector<double> Diagonal()
        {
            return ToMatrix().Diagonal();
        }
        public static new Matrix4D Identity()
        {
            return new Matrix4D(Matrix3D.Identity());
        }
        public Matrix3D Rotation
        {
            get
            {
                return new Matrix3D(Storage[0, 0], Storage[0, 1], Storage[0, 2], Storage[1, 0], Storage[1, 1], Storage[1, 2], Storage[2, 0], Storage[2, 1], Storage[2, 2]);
            }
            set
            {
                Storage[0, 0] = value[0, 0];
                Storage[0, 1] = value[0, 1];
                Storage[0, 2] = value[0, 2];
                Storage[1, 0] = value[1, 0];
                Storage[1, 1] = value[1, 1];
                Storage[1, 2] = value[1, 2];
                Storage[2, 0] = value[2, 0];
                Storage[2, 1] = value[2, 1];
                Storage[2, 2] = value[2, 2];
            }
        }
        public double Scale
        {
            get
            {
                return Storage[3, 3];
            }
            set
            {
                Storage[3, 3] = value;
            }
        }
        public Vector3D Shear
        {
            get
            {
                return new Vector3D(Storage[3, 0], Storage[3, 1], Storage[3, 2]);
            }
            set
            {
                Storage[3, 0] = value.X;
                Storage[3, 1] = value.Y;
                Storage[3, 2] = value.Z;
            }
        }
        public Matrix4D Subtract(Vector3D v)
        {
            return new Matrix4D(Rotation, Translation - v, Shear, Scale);
        }
        public new Quaternion ToQuaternion()
        {
            return Rotation.ToQuaternion();
        }
        public Transform3D ToTransform()
        {
            return new Transform3D(Translation, Rotation.ToQuaternion());
        }
        public Vector3D Translation
        {
            get
            {
                return new Vector3D(Storage[0, 3], Storage[1, 3], Storage[2, 3]);
            }
            set
            {
                Storage[0, 3] = value.X;
                Storage[1, 3] = value.Y;
                Storage[2, 3] = value.Z;
            }
        }
        #endregion
    }
}

