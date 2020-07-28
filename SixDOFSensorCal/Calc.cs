using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Linq;
using System.IO;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Security.Policy;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Schema;

namespace SixDOFSensorCal
{
    public class Calc
    {
        /// <summary>
        /// Perform circle fitting on the points and calculate roll angle offset
        /// return doulbe array in the format
        /// {offset angle, stdev, min, max, x center, y center, radius}
        /// </summary>
        /// <param name="data">list of double array. {X,Y,Z,Qw,Qx,Qy,Qz} or {X,Y,Z,Rx,Ry,Rz}</param>
        /// <returns></returns>
        public static double[] Process(List<double[]>data)
        {

            //def variables
            double[] results = new double[7]; 
            int lineCount = data.Count;             
            double[] x = new double[lineCount]; //place holder from the input data
            double[] y = new double[lineCount];
            double[] z = new double[lineCount];
            double[] xyPlaneNormal = new double[3] { 0, 0, -1 };
            double[] xPos = new double[lineCount]; //rotated position on xy plane
            double[] yPos = new double[lineCount]; //rotated position on xy plane
            double[] _Qx = new double[lineCount];
            double[] _Qy = new double[lineCount];
            double[] _Qz = new double[lineCount];
            double[] _Qw = new double[lineCount];
            double[] _Ry = new double[lineCount];
            double[] _Rz = new double[lineCount];
            double[] _Rx = new double[lineCount];
            double[] sensorRollAngles = new double[lineCount];

            //make double array for xyz coordinates
            for (int i = 0; i < lineCount; i++)
            {
                x[i] = data[i][0];
                y[i] = data[i][1];
                z[i] = data[i][2];
                if (data[0].Length == 7)
                {
                    _Qw[i] = data[i][3];
                    _Qx[i] = data[i][4];
                    _Qy[i] = data[i][5];
                    _Qz[i] = data[i][6];
                }
                else
                {
                    _Rx[i] = data[i][3];
                    _Ry[i] = data[i][4];
                    _Rz[i] = data[i][5];
                }
            }
            

            //rotate points on xy plane
            double[] planeNormal = GetPlaneFromPoints(x, y, z);
            List<double[]> rotatedPoints = VectorOps.RotatePointsOntoXYPlane(x, y, z, planeNormal, xyPlaneNormal);
            for (int i = 0; i < rotatedPoints.Count; i++)
            {
                xPos[i] = rotatedPoints[i][0];
                yPos[i] = rotatedPoints[i][1];
            }
            //find the fitted circle
            double[] centeronXYPlane = Calc.FindCenter(xPos, yPos);

            //get normal angles at xy positions
            double[] normalAngles = Calc.GetNormalAnglesAtPositions(xPos, yPos);
            
            //find rotation matrix for rotating measured angles
            double[] rotMatrix = VectorOps.FindRotMatrixFromTwoVectors(planeNormal, xyPlaneNormal);

            //Test 1, Rodriguez rotation on Euler angles
            List<double[]> eulerAngles = new List<double[]>();
            for (int i = 0; i< lineCount; i++)
            {
                if (data[0].Length == 7)
                {
                    double[] eulerFromQuat = GetEulerFromQuat(_Qw[i], _Qx[i], _Qy[i], _Qz[i]);
                    eulerAngles.Add(eulerFromQuat);
                    double[] temp = VectorOps.RotatePointOntoXYPlane(eulerFromQuat[0], eulerFromQuat[1], eulerFromQuat[2], planeNormal, xyPlaneNormal);
                    sensorRollAngles[i] = temp[2];
                }
                else
                {
                    eulerAngles.Add(new double[3] { _Rx[i], _Ry[i], _Rz[i] });
                    double[] temp = VectorOps.RotatePointOntoXYPlane(_Rx[i], _Ry[i], _Rz[i], planeNormal, xyPlaneNormal);
                    sensorRollAngles[i] = temp[2];
                }
            }

            ////Calculation

            using (FileStream fs = new FileStream("d:\\test.csv", FileMode.Create))
            {
                using (StreamWriter sw = new StreamWriter(fs))
                {
                    sw.WriteLine("{0},{1},{2}", "Normal Angle", "Sensor Roll Angle", "Difference Measured-Normal angle");
                    for (int i = 0; i < lineCount; ++i)
                    {
                        sw.WriteLine("{0},{1},{2}", normalAngles[i], sensorRollAngles[i], normalAngles[i] - normalAngles[i]);
                    }
                }
            }

            double[] stats = Calc.Stats(normalAngles, sensorRollAngles);
            results[0] = stats[0];
            results[1] = stats[1];
            results[2] = stats[2];
            results[3] = stats[3];

            results[4] = centeronXYPlane[0];
            results[5] = centeronXYPlane[1];
            results[6] = centeronXYPlane[2];

            return results;
        }
        /// <summary>
        /// Takes array of X and Y positions
        /// Returns center position X and Y, and R, raduis of the fitted circle in 2D space
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

        /// <summary>
        /// From points in 3D space, calculates a normal vector of the plane that fits them the best
        /// Returns a normal vector (x,y,z)
        /// </summary>
        /// <param name="_pointsX"></param>
        /// <param name="_pointsY"></param>
        /// <param name="_pointsZ"></param>
        /// <returns></returns>
        public static double[] GetPlaneFromPoints(List<double> _pointsX, List<double> _pointsY, List<double> _pointsZ)
        {
            //Get XYZ of points, find plane fitting the points
            //return plane eq
            double[] planeEQ = new double[3];
            double[] pointsX = _pointsX.ToArray();
            double[] pointsY = _pointsY.ToArray();
            double[] pointsZ = _pointsZ.ToArray();

            planeEQ = GetPlaneFromPoints(pointsX, pointsY, pointsZ);
            #region dup
            //int _length = _pointsX.Count;
            //if (_length != _pointsY.Count)
            //    return planeEQ;
            //if (_length != _pointsZ.Count)
            //    return planeEQ;

            //double _centroidX = GetMean(_pointsX.ToArray());
            //double _centroidY = GetMean(_pointsY.ToArray());
            //double _centroidZ = GetMean(_pointsZ.ToArray());


            //double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0, Mz = 0;
            //for (int i = 0; i < _length; i++)
            //{
            //    double Xi = _pointsX[i] - _centroidX;
            //    double Yi = _pointsY[i] - _centroidY;
            //    double Zi = _pointsZ[i] - _centroidZ;

            //    Mxy += (Xi * Yi);
            //    Mxx += (Xi * Xi);
            //    Myy += (Yi * Yi);
            //    Mxz += (Xi * Zi);
            //    Myz += (Yi * Zi);
            //    Mzz += (Zi * Zi);
            //}

            //Mxx = Mxx / _length;
            //Myy = Myy / _length;
            //Mxy = Mxy / _length;
            //Mxz = Mxz / _length;
            //Myz = Myz / _length;
            //Mzz = Mzz / _length;


            //double[] weightedDir = new double[3];
            //double[] axisDir = new double[3];
            ////in X axis
            //axisDir[0] = Myy * Mzz - Myz * Myz;
            //axisDir[1] = Mxz * Myz - Mxy * Mzz;
            //axisDir[2] = Mxy * Myz - Mxz * Myy;

            //double weightX = axisDir[0] * axisDir[0];

            //for (int i = 0; i < 3; i++)
            //    weightedDir[i] += axisDir[i] * weightX;

            ////in Y axis
            //axisDir[0] = Mxz * Myz - Mxy * Mzz;
            //axisDir[1] = Mxx * Mzz - Mxz * Mxz;
            //axisDir[2] = Mxy * Mxz - Myz * Mxx;

            //double weightY = axisDir[1] * axisDir[1];
            //double dotProd = axisDir[0] * weightedDir[0] + axisDir[1] * weightedDir[1] + axisDir[2] * weightedDir[2];
            //if (dotProd < 0)
            //    weightY *= -1;
            //for (int i = 0; i < 3; i++)
            //    weightedDir[i] += axisDir[i] * weightY;

            ////in Z axis
            //axisDir[0] = Mxy * Myz - Mxz * Myy;
            //axisDir[1] = Mxy * Mxz - Myz * Mxx;
            //axisDir[2] = Mxx * Myy - Mxy * Mxy;

            //double weightZ = axisDir[2] * axisDir[2];
            //dotProd = axisDir[0] * weightedDir[0] + axisDir[1] * weightedDir[1] + axisDir[2] * weightedDir[2];
            //if (dotProd < 0)
            //    weightZ *= -1;
            //for (int i = 0; i < 3; i++)
            //    weightedDir[i] += axisDir[i] * weightZ;

            ////normalize
            //double absSq = weightedDir[0] * weightedDir[0] + weightedDir[1] * weightedDir[1] + weightedDir[2] * weightedDir[2];
            //planeEQ[0] = weightedDir[0] / Math.Sqrt(absSq);
            //planeEQ[1] = weightedDir[1] / Math.Sqrt(absSq);
            //planeEQ[2] = weightedDir[2] / Math.Sqrt(absSq);
            #endregion
            return planeEQ;
        }
        public static double[] GetPlaneFromPoints(double[] _pointsX, double[] _pointsY, double[] _pointsZ)
        {
            //Get XYZ of points, find plane fitting the points
            //return plane eq
            double[] planeEQ = new double[3];

            int _length = _pointsX.Length;
            if (_length != _pointsY.Length)
                return planeEQ;
            if (_length != _pointsZ.Length)
                return planeEQ;

            double _centroidX = GetMean(_pointsX);
            double _centroidY = GetMean(_pointsY);
            double _centroidZ = GetMean(_pointsZ);


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
            planeEQ[0] = weightedDir[0] / Math.Sqrt(absSq);
            planeEQ[1] = weightedDir[1] / Math.Sqrt(absSq);
            planeEQ[2] = weightedDir[2] / Math.Sqrt(absSq);

            return planeEQ;
        }

        /// <summary>
        /// Centers circle and makes unit circle
        /// Returns transformed XY coordinates
        /// </summary>
        /// <param name="_pointX"></param>
        /// <param name="_pointY"></param>
        /// <param name="_center"></param>
        /// <returns></returns>
        public static double[,] Normalize(double[] _pointX, double[] _pointY, double[] _center)
        {
            int length = _pointX.Length;
            double[,] _normalized = new double[length,2];
            for(int i = 0; i < length; i++)
            {
                double X = _pointX[i] - _center[0];
                double Y = _pointY[i] - _center[1];
                double magnitude = Math.Sqrt(X * X + Y * Y);
                _normalized[i, 0] = X / magnitude; //also could replace it with the radius (_center[2])
                _normalized[i, 1] = Y / magnitude;
            }
            return _normalized;
        }

        /// <summary>
        /// Returns normal angles on the circumference at XY coordinates
        /// </summary>
        /// <param name="_pointX"></param>
        /// <param name="_pointY"></param>
        /// <returns></returns>
        public static double[] GetNormalAnglesAtPositions(double[] _pointX, double[] _pointY)
        {
            int length = _pointX.Length;
            double[] angles = new double[length];
            double[] center = FindCenter(_pointX, _pointY);
            double[,] normalized = Normalize(_pointX, _pointY, center);
            angles = GetNormalAnglesAtPositions(normalized);
            return angles;
        }

        public static double[] GetNormalAnglesAtPositions(double[,] _points)
        {
            int length = _points.Length/2;
            double[] _angles = new double[length];
            for (int i = 0; i < length; i++)
            {
                _angles[i] = Math.Atan2(_points[i,1], _points[i,0]) * 180 / Math.PI;
            }
            return _angles;
        }
        
        /// <summary>
        /// GetAnglesFromQuat
        /// Returns [n,3]array of Rx,Ry,Rz angles
        /// </summary>
        /// <param name="_Qw"></param>
        /// <param name="_Qx"></param>
        /// <param name="_Qy"></param>
        /// <param name="_Qz"></param>
        /// <returns></returns>
        public static List<double[]> GetAnglesFromQuat(double[] _Qw, double[] _Qx, double[] _Qy, double[] _Qz)
        {
            int length = _Qw.Length;
            List<double[]> _angles = new List<double[]>();
            for(int i = 0; i < length; i++)
            {
                _angles.Add(GetEulerFromQuat(_Qw[i], _Qx[i], _Qy[i], _Qz[i]));
            }

            return _angles;
        }

        public static double[] GetEulerFromQuat(double _Qw, double _Qx, double _Qy, double _Qz)
        {
            double[] angles = new double[3];
            // Rx
            double sinr_cosp = 2 * (_Qw * _Qx + _Qy * _Qz);
            double cosr_cosp = 1 - 2 * (_Qx * _Qx + _Qy * _Qy);
            angles[0]= Math.Atan2(sinr_cosp, cosr_cosp);

            // Ry
            double sinp = 2 * (_Qw * _Qy - _Qz * _Qx);
            if (Math.Abs(sinp) >= 1)
                angles[1] = Math.PI * sinp / Math.Abs(sinp); // use 90 degrees if out of range
            else
                angles[1]= Math.Asin(sinp);

            // Rz
            double siny_cosp = 2 * (_Qw * _Qz + _Qx * _Qy);
            double cosy_cosp = 1 - 2 * (_Qy * _Qy + _Qz * _Qz);
            angles[2] = Math.Atan2(siny_cosp, cosy_cosp);
            return angles;
        }
        public static double GetRzFromQuat(double _Qw, double _Qx, double _Qy, double _Qz)
        {
            double _angle = -9999;

            double xterm = _Qw * _Qw + _Qx * _Qx - _Qy * _Qy - _Qz * _Qz;
            double yterm = 2 * (_Qw * _Qz + _Qx * _Qy);
            _angle = Math.Atan2(yterm, xterm) * 180 / Math.PI;
            return _angle;
        }
        public static double[] GetRzFromQuatArray(double[] _Qw, double[] _Qx, double[] _Qy, double[] _Qz)
        {
            int length = _Qw.Length;
            double[] angles = new double[length];
            for (int i = 0; i < length; i++)
            {
                double xterm = _Qw[i] * _Qw[i] + _Qx[i] * _Qx[i] - _Qy[i] * _Qy[i] - _Qz[i] * _Qz[i];
                double yterm = 2 * (_Qw[i] * _Qz[i] + _Qx[i] * _Qy[i]);
                angles[i] = Math.Atan2(yterm, xterm) * 180 / Math.PI;
            }
            return angles;
        }

        public static double[] GetMatrixFromQuat(double _Qw, double _Qx, double _Qy, double _Qz)
        {
            double[] rotMat = new double[9];
            rotMat[0] = 1 - 2 * (_Qy * _Qy + _Qz * _Qz);
            rotMat[1] = 2 * (_Qx * _Qy - _Qz * _Qw);
            rotMat[2] = 2 * (_Qx * _Qz + _Qy * _Qw);
            rotMat[3] = 2 * (_Qx * _Qy + _Qz * _Qw);
            rotMat[4] = 1 - 2 * (_Qx * _Qx + _Qz * _Qz);
            rotMat[5] = 2 * (_Qy * _Qz - _Qx * _Qw);
            rotMat[6] = 2 * (_Qx * _Qz - _Qy * _Qw);
            rotMat[7] = 2 * (_Qy * _Qz + _Qx * _Qw);
            rotMat[8] = 1 - 2 * (_Qx * _Qx + _Qy * _Qy);
            return rotMat;
        }

        public static double[] GetNormalAnglesAtPositions(List<double> _pointX, List<double> _pointY)
        {
            double[] angles = new double[_pointX.Count()];
            double[] posX = _pointX.ToArray();
            double[] posY = _pointY.ToArray();
            //double[] center = FindCenter(posX, posY);
            //double[,] normalized = Normalize(posX, posY, center);
            //angles = GetNormalAnglesAtPositions(normalized);
            angles = GetNormalAnglesAtPositions(posX, posY);
            return angles;
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

        public static double[] Stats(double[] normalAngles, double[] sensorRollAngles)
        {
            double[] results = new double[4];
            double[] _stats = new double[6]; //mean, std, min, max, reg, b;
            int counts = normalAngles.Length;
            if (normalAngles.Length != sensorRollAngles.Length)
                return null;

            double[] _difference = new double[counts];
            for (int i = 0; i < counts; i++)
            {
                _difference[i] = normalAngles[i] - sensorRollAngles[i];

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

    }

    public class VectorOps
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="_pointsX"></param>
        /// <param name="_pointY"></param>
        /// <param name="_pointZ"></param>
        /// <param name="_planeEQ"></param>
        /// <returns></returns> 
        public static List<double[]> RotatePointsOntoXYPlane(List<double> _pointX, List<double> _pointY, List<double> _pointZ, double[] _planeEQ, double[] xyPlaneNormal)
        {
            //Get XYZ of points, rotate them by plane eq
            //return XY of points
            //using Rodrigues Rotation
            List<double[]> XYCoords = new List<double[]>();
            
            if (_pointX.Count != _pointY.Count)
                return XYCoords;
            if (_pointX.Count != _pointZ.Count)
                return XYCoords;
            if (_planeEQ.Length != 3)
                return XYCoords;

            //double[] xyPlaneNormal = new double[3]{ 0, 0, -1 };
            double[] k = CrossProduct(_planeEQ, xyPlaneNormal);
            double cosTheta = DotProduct(_planeEQ, xyPlaneNormal);
            double theta = Math.Acos(cosTheta); //since dot product is of unit vectors
            double sinTheta = Math.Sin(theta);

            for(int i = 0; i< _pointX.Count; i++)
            {
                double[] pt = new double[3] {_pointX[i], _pointY[i], _pointZ[i] }; //Input Point
                double[] rotatedPoint = new double[3];
                double[] crossProd = CrossProduct(k, pt);
                double dotProd = DotProduct(k, pt);
                for (int j = 0; j< 3; j++)
                    rotatedPoint[j] = pt[j] * cosTheta + crossProd[j] * sinTheta + k[j] * dotProd * (1 - cosTheta);

                XYCoords.Add(rotatedPoint);
            }

            return XYCoords;
        }
        public static List<double[]> RotatePointsOntoXYPlane(double[] _pointX, double[] _pointY, double[] _pointZ, double[] _planeEQ, double[] xyPlaneNormal)
        {
            //Get XYZ of points, rotate them by plane eq
            //return XY of points
            //using Rodrigues Rotation
            List<double[]> XYCoords = new List<double[]>();

            if (_pointX.Length != _pointY.Length)
                return XYCoords;
            if (_pointX.Length != _pointZ.Length)
                return XYCoords;
            if (_planeEQ.Length != 3)
                return XYCoords;

            //double[] xyPlaneNormal = new double[3] { 0, 0, -1 };
            double[] k = CrossProduct(_planeEQ, xyPlaneNormal);
            double cosTheta = DotProduct(_planeEQ, xyPlaneNormal);
            double theta = Math.Acos(cosTheta); //since dot product is of unit vectors
            double sinTheta = Math.Sin(theta);

            for (int i = 0; i < _pointX.Length; i++)
            {
                double[] pt = new double[3] { _pointX[i], _pointY[i], _pointZ[i] }; //Input Point
                double[] rotatedPoint = new double[3];
                double[] crossProd = CrossProduct(k, pt);
                double dotProd = DotProduct(k, pt);
                for (int j = 0; j < 3; j++)
                    rotatedPoint[j] = pt[j] * cosTheta + crossProd[j] * sinTheta + k[j] * dotProd * (1 - cosTheta);

                XYCoords.Add(rotatedPoint);
            }

            return XYCoords;
        }

        public static double[] RotatePointOntoXYPlane(double _pointX, double _pointY, double _pointZ, double[] _planeNoraml, double[] xyPlaneNormal)
        {
            double[] results = new double[3];

            //double[] xyPlaneNormal = new double[3] { 0, 0, -1 };
            double[] k = CrossProduct(_planeNoraml, xyPlaneNormal);
            double cosTheta = DotProduct(_planeNoraml, xyPlaneNormal);
            double theta = Math.Acos(cosTheta); //since dot product is of unit vectors
            double sinTheta = Math.Sin(theta);

            double[] pt = new double[3] { _pointX, _pointY, _pointZ }; //Input Point
            double[] crossProd = CrossProduct(k, pt);
            double dotProd = DotProduct(k, pt);
            for (int j = 0; j < 3; j++)
                results[j] = pt[j] * cosTheta + crossProd[j] * sinTheta + k[j] * dotProd * (1 - cosTheta);

            return results;
        }
        /// <summary>
        /// Gets two unit vectors [x,y,z] format and returns rotation matrix between the two
        /// R = [R11, R12, R13, R21, R22, R23, R31, R32, R33];
        /// </summary>
        /// <param name="a">vector {1x3}</param>
        /// <param name="b">vector {1x3}</param>
        /// <returns></returns>
        public static double[] FindRotMatrixFromTwoVectors(double[] a, double[] b)
        {
            double[] rotMat = new double[9];
            double[] vMat = new double[9];
            double[] vMatSq = new double[9];
            double[] iMat = new double[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            if (a.Length != 3 || a.Length != b.Length)
                return rotMat;

            //normalize
            double aMag = Math.Sqrt(DotProduct(a, a));
            if (aMag < 0.999 || aMag> 1.001)
                for (int i = 0; i < 3; i++)
                    a[i] = a[i] / aMag;
            double bMag = Math.Sqrt(DotProduct(b, b));
            if (bMag < 0.999 || bMag > 1.001)
                for (int i = 0; i < 3; i++)
                    b[i] = b[i] / bMag;


            double[] v = CrossProduct(a, b);
            double cosTheta = DotProduct(a, b);
            double sinTheta = Math.Sin(Math.Acos(cosTheta));
            double s = Math.Sqrt(DotProduct(v, v));
            double c = DotProduct(a, b);
            
            if (c == -1) //c cannot be -1. see below
                return rotMat;

            double d = 1 / (1 + c);

            vMat[0] = 0;
            vMat[1] = v[2] * -1;
            vMat[2] = v[1];
            
            vMat[3] = v[2];
            vMat[4] = 0;
            vMat[5] = v[0] * -1;

            vMat[6] = v[1] * -1;
            vMat[7] = v[0];
            vMat[8] = 0;

            //check Matrix op
            vMatSq = MatrixProduct(vMat, vMat);

            for (int i = 0; i < 9; i++)
                rotMat[i] = iMat[i] + vMat[i] + vMatSq[i] * d;

            return rotMat;
        }

        public static double[] CrossProduct (double[] vector1, double[] vector2)
        {
            double[] crossProduct = new double[3];
            double[] v1 = vector1;
            double[] v2 = vector2;
            crossProduct[0] = v1[1] * v2[2] - v1[2] * v2[1];
            crossProduct[1] = v1[2] * v2[0] - v1[0] * v2[2];
            crossProduct[2] = v1[0] * v2[1] - v1[1] * v2[0];
            return crossProduct;
        }
        
        public static double DotProduct(double[] vector1, double[] vector2)
        {
            double dotProduct = 0;
            if (vector1.Length != vector2.Length)
                return dotProduct;
            for (int i = 0; i < vector1.Length; i++)
                dotProduct += vector1[i] * vector2[i];
            return dotProduct;
        }

        public static double[] MatrixProduct(double[] a, double[] b)
        {
            double[] product = new double[9];
            double A11 = a[0];
            double A12 = a[1];
            double A13 = a[2];
            double A21 = a[3];
            double A22 = a[4];
            double A23 = a[5];
            double A31 = a[6];
            double A32 = a[7];
            double A33 = a[8];
            double B11 = b[0];
            double B12 = b[1];
            double B13 = b[2];
            double B21 = b[3];
            double B22 = b[4];
            double B23 = b[5];
            double B31 = b[6];
            double B32 = b[7];
            double B33 = b[8];

            product[0] = DotProduct(new double[3] { A11, A12, A13 }, new double[3] { B11, B21, B31 });
            product[1] = DotProduct(new double[3] { A11, A12, A13 }, new double[3] { B12, B22, B32 });
            product[2] = DotProduct(new double[3] { A11, A12, A13 }, new double[3] { B13, B23, B33 });
            product[3] = DotProduct(new double[3] { A21, A22, A23 }, new double[3] { B11, B21, B31 });
            product[4] = DotProduct(new double[3] { A21, A22, A23 }, new double[3] { B12, B22, B32 });
            product[5] = DotProduct(new double[3] { A21, A22, A23 }, new double[3] { B13, B23, B33 });
            product[6] = DotProduct(new double[3] { A31, A32, A33 }, new double[3] { B11, B21, B31 });
            product[7] = DotProduct(new double[3] { A31, A32, A33 }, new double[3] { B12, B22, B32 });
            product[8] = DotProduct(new double[3] { A31, A32, A33 }, new double[3] { B13, B23, B33 });
            return product;
        }
    }
}
