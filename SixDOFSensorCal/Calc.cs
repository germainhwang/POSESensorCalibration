using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SixDOFSensorCal
{
    public class Calc
    {
        /// <summary>
        /// Takes array of X and Y positions
        /// Returns center position X and Y, and R, raduis of the fitted circle
        /// </summary>
        /// <param name="_pointX"></param>
        /// <param name="_pointY"></param>
        /// <returns></returns>
        public static double[] FindCenter(double[] _pointX, double[] _pointY)
        {
            double[] _center = new double[3];
            double _centroidX = GetMean(_pointX);
            double _centroidY = GetMean(_pointY);
            int _length = _pointX.Length;
            double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0, Mz = 0;
            for (int i = 0; i < _length; i++)
            {
                double Xi = _pointX[i] - _centroidX;
                double Yi = _pointY[i] - _centroidY;
                double Zi = (Xi * Xi) + (Yi * Yi);

                Mxy += (Xi * Yi);
                Mxx += (Xi * Xi);
                Myy += (Yi * Yi);
                Mxz += (Xi * Zi);
                Myz += (Yi * Zi);
                Mzz += (Zi * Zi);
            }

            Mxx = Mxx / _length;
            Myy = Myy / _length;
            Mxy = Mxy / _length;
            Mxz = Mxz / _length;
            Myz = Myz / _length;
            Mzz = Mzz / _length;
            // computing the coefficients of the characteristic polynomial
            Mz = Mxx + Myy;
            double Cov_xy = Mxx * Myy - Mxy * Mxy;
            double Mxz2 = Mxz * Mxz;
            double Myz2 = Myz * Myz;
            double A2 = (4 * Cov_xy) - (3 * Mz * Mz) - Mzz;
            double A1 = (Mzz * Mz) + (4 * Cov_xy * Mz) - Mxz2 - Myz2 - (Mz * Mz * Mz);
            double A0 = (Mxz2 * Myy) + (Myz2 * Mxx) - (Mzz * Cov_xy) - (2 * Mxz * Myz * Mxy) + (Mz * Mz * Cov_xy);
            double A22 = A2 + A2;
            double epsilon = 1e-12;
            double ynew = 1e+20;
            int IterMax = 20;
            double xnew = 0.0;

            // Newton's method starting at x=0
            for (int iter = 0; iter < IterMax; iter++)
            {
                double yold = ynew;
                ynew = A0 + xnew * (A1 + xnew * (A2 + 4 * xnew * xnew));
                if (Math.Abs(ynew) > Math.Abs(yold))
                {
                    Console.Write("Newton-Pratt goes wrong direction: |ynew| > |yold|");
                    xnew = 0;
                    break;
                }
                double Dy = A1 + xnew * (A22 + 16 * xnew * xnew);
                double xold = xnew;
                xnew = xold - ynew / Dy;
                if (Math.Abs((xnew - xold) / xnew) < epsilon)
                    break;
                if (iter >= IterMax)
                {
                    Console.Write("Newton-Pratt will not converge");
                    xnew = 0;
                }
                if (xnew < 0.0)
                {
                    Console.Write("Newton-Pratt negative root:  x={0}", xnew);
                    xnew = 0;
                }
            }

            double det = xnew * xnew - xnew * Mz + Cov_xy;
            double centerX = (Mxz * (Myy - xnew) - Myz * Mxy) / det / 2;
            double centerY = (Myz * (Mxx - xnew) - Mxz * Mxy) / det / 2;
            _center[0] = centerX + _centroidX;
            _center[1] = centerY + _centroidY;
            double centerSq = (centerX * centerX) + (centerY * centerY);
            _center[2] = Math.Sqrt(centerSq + Mz + (2*xnew));


            return _center;
        }

        public static double[,] Normalize(double[] _pointX, double[] _pointY, double[] _center)
        {
            int length = _pointX.Length;
            double[,] _normalized = new double[length,2];
            for(int i = 0; i < length; i++)
            {
                double X = _pointX[i] - _center[0];
                double Y = _pointY[i] - _center[1];
                double sqrtSum = Math.Sqrt(X * X + Y * Y);
                _normalized[i, 0] = X / sqrtSum; //also could replace it with the radius (_center[2])
                _normalized[i, 1] = Y / sqrtSum;
            }
            return _normalized;
        }

        public static double[] GetAnglesFromPosition(double[] _pointX, double[] _pointY)
        {
            int length = _pointX.Length;
            double[] _angles = new double[length];
            for (int i = 0; i< length; i++)
            {
                _angles[i] = Math.Atan2(_pointY[i], _pointX[i]) * 180 / Math.PI;
            }
            return _angles;
        }

        public static double[] GetAnglesFromPosition(double[,] _points)
        {
            int length = _points.Length/2;
            double[] _angles = new double[length];
            for (int i = 0; i < length; i++)
            {
                _angles[i] = Math.Atan2(_points[i,1], _points[i,0]) * 180 / Math.PI;
            }
            return _angles;
        }

        public static double[] GetAnglesFromQuat(double[] _Qw, double[] _Qx, double[] _Qy, double[] _Qz)
        {
            int length = _Qw.Length;
            double[] _angles = new double[length];

            for (int i = 0; i < length; i++)
            {
                //double yterm = _Qw[i]*_Qw[i] + _Qx[i]*_Qx[i] - _Qy[i]*_Qy[i] - _Qz[i]*_Qz[i];
                //double xterm = 2*(_Qw[i]*_Qz[i] + _Qx[i]*_Qy[i]);
                //_angles[i] = Math.Atan2(yterm, xterm) * 180 / Math.PI;
                _angles[i] = GetAngleFromQuat(_Qw[i], _Qx[i], _Qy[i], _Qz[i]);
            }
            return _angles;
        }

        public static double GetMean (double[] _points)
        {
            double _mean = -99999.0f;
            double _sum = 0;
            for (int i = 0; i < _points.Length; i++)
                _sum += _points[i];
            _mean = _sum / _points.Length;
            return _mean;
        }

        public static double GetAngleFromQuat(double _Qw, double _Qx, double _Qy, double _Qz)
        {
            double _angle = -9999;

            double xterm = _Qw * _Qw + _Qx * _Qx - _Qy * _Qy - _Qz * _Qz;
            double yterm = 2 * (_Qw * _Qz + _Qx * _Qy);
            _angle = Math.Atan2(yterm, xterm) * 180 / Math.PI;
            return _angle;
        }

        public static double[] GetAnglesFromPosition(List<double> _pointX, List<double> _pointY)
        {
            double[] _angles = new double[_pointX.Count()];
            double[] _posX = _pointX.ToArray();
            double[] _posY = _pointY.ToArray();
            double[] _center = FindCenter(_posX, _posY);
            double[,] _normalized = Normalize(_posX, _posY, _center);
            _angles = GetAnglesFromPosition(_normalized);
            return _angles;
        }

        public static double[] Stats(List<double> _anglesByPosition, List<double> _anglesByRotation)
        {
            double[] _stats = new double[6]; //mean, std, min, max, reg, b;
            int counts = _anglesByRotation.Count();
            if (_anglesByPosition.Count() != _anglesByRotation.Count())
                return null;

            double[] _difference = new double[counts];
            for (int i = 0; i < counts; i++)
            {
                _difference[i] = _anglesByRotation[i] - _anglesByPosition[i];

                if (_difference[i] < 0)
                    _difference[i] += 360;
            }

            _stats[0] = GetMean(_difference);
            _stats[1] = StandardDeviation(_difference);
            _stats[2] = Min(_difference);
            _stats[3] = Max(_difference);
            _stats[4] = 0;
            _stats[5] = 0;

            if ((_stats[3] - _stats[2]) > 180)
            {
                double midpoint = (_stats[3] + _stats[2]) / 2;
                for (int i = 0; i < counts; i++)
                {
                    if (_difference[i] > midpoint)
                        _difference[i] -= 360;
                }
                _stats[0] = GetMean(_difference);
                _stats[1] = StandardDeviation(_difference);
                _stats[2] = Min(_difference);
                _stats[3] = Max(_difference);
            }

            return _stats;
        }

        public static double Variance(double[] x)
        {
            double mean = GetMean(x), sumSq = 0;
            int n = x.Length;

            for (int i = 0; i < n; i++)
            {
                double delta = x[i] - mean;

                sumSq += delta * delta;
            }

            return sumSq / (n - 1);
        }

        public static double StandardDeviation(double[] x)
        {
            return Math.Sqrt(Variance(x));
        }

        public static double Max(double[] x)
        {
            double max = double.MinValue;

            for (int i = 0; i < x.Length; i++)
                if (x[i] > max)
                    max = x[i];

            return max;
        }

        public static double Min(double[] x)
        {
            double min = double.MaxValue;

            for (int i = 0; i < x.Length; i++)
                if (x[i] < min)
                    min = x[i];

            return min;
        }

        public static double[] GetPlaneFromPoints(List<double> _pointsX, List<double> _pointsY, List<double> _pointsZ)
        {
            //Get XYZ of points, find plane fitting the points
            //return plane eq
            double[] planeEQ = new double[4];

            int _length = _pointsX.Count;
            if (_length != _pointsY.Count)
                return planeEQ;
            if (_length != _pointsZ.Count)
                return planeEQ;

            double _centroidX = GetMean(_pointsX.ToArray());
            double _centroidY = GetMean(_pointsY.ToArray());
            double _centroidZ = GetMean(_pointsZ.ToArray());


            double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0, Mz = 0;
            for (int i = 0; i < _length; i++)
            {
                double Xi = _pointsX[i] - _centroidX;
                double Yi = _pointsY[i] - _centroidY;
                double Zi = _pointsZ[i] - _centroidZ;

                Mxy += (Xi * Yi);
                Mxx += (Xi * Xi);
                Myy += (Yi * Yi);
                Mxz += (Xi * Zi);
                Myz += (Yi * Zi);
                Mzz += (Zi * Zi);
            }

            Mxx = Mxx / _length;
            Myy = Myy / _length;
            Mxy = Mxy / _length;
            Mxz = Mxz / _length;
            Myz = Myz / _length;
            Mzz = Mzz / _length;


            double[] weightedDir = new double[3];
            double[] axisDir = new double[3];
            //in X axis
            axisDir[0] = Myy * Mzz - Myz * Myz;
            axisDir[1] = Mxz * Myz - Mxy * Mzz;
            axisDir[2] = Mxy * Myz - Mxz * Myy;

            double weightX = axisDir[0] * axisDir[0];

            for (int i = 0; i < 3; i++)
                weightedDir[i] += axisDir[i] * weightX;

            //in Y axis
            axisDir[0] = Mxz * Myz - Mxy * Mzz;
            axisDir[1] = Mxx * Mzz - Mxz * Mxz;
            axisDir[2] = Mxy * Mxz - Myz * Mxx;

            double weightY = axisDir[1] * axisDir[1];
            double dotProd = axisDir[0] * weightedDir[0] + axisDir[1] * weightedDir[1] + axisDir[2] * weightedDir[2];
            if (dotProd < 0)
                weightY *= -1;
            for (int i = 0; i < 3; i++)
                weightedDir[i] += axisDir[i] * weightY;

            //in Z axis
            axisDir[0] = Mxy * Myz - Mxz * Myy;
            axisDir[1] = Mxy * Mxz - Myz * Mxx;
            axisDir[2] = Mxx * Myy - Mxy * Mxy;

            double weightZ = axisDir[2] * axisDir[2];
            dotProd = axisDir[0] * weightedDir[0] + axisDir[1] * weightedDir[1] + axisDir[2] * weightedDir[2];
            if (dotProd < 0)
                weightZ *= -1;
            for (int i = 0; i < 3; i++)
                weightedDir[i] += axisDir[i] * weightZ;

            //normalize
            double absSq = weightedDir[0] * weightedDir[0] + weightedDir[1] * weightedDir[1] + weightedDir[2] * weightedDir[2];
            planeEQ[0] = absSq;
            planeEQ[1] = weightedDir[0] / absSq;
            planeEQ[2] = weightedDir[1] / absSq;
            planeEQ[3] = weightedDir[2] / absSq;

            return planeEQ;
        }
        
        public static List<double[]> RotatePointsOnXYPlane(List<double> _pointsX, List<double> _pointY, List<double> _pointZ, double[] _planeEQ)
        {
            //Get XYZ of points, rotate them by plane eq
            //return XY of points
            List<double[]> XYCoords = new List<double[]>();



            return XYCoords;
        }

        public static double[] FindCenter(List<double[]> _XYCoords)
        {
            int listLength = _XYCoords.Count;
            double[] pointX = new double[listLength];
            double[] pointY = new double[listLength];
            for (int i = 0; i< listLength; i++)
            {
                pointX[i] = _XYCoords[i][0];
                pointY[i] = _XYCoords[i][1];
            }
            double[] center = FindCenter(pointX, pointY);
            return center;
        }

    }
}
