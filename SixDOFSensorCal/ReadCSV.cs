using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace SixDOFSensorCal
{
    class ReadCSV
    {
        public static double AngleCorrection = -999.0;
        public static double AngleDeviation = -999.0;
        public static double AngleMax = -999.0;
        public static double AngleMin = -999.0;
        public static double[] Center = new double[3];
        public static void ParseNDIFile(string filename)
        {
            //Reading CSV
            using (var reader = new StreamReader(File.OpenRead(filename)))
            {
                var data = CsvParser.ParseHeadAndTail(reader, ',', '"');

                var header = data.Item1;
                var lines = data.Item2;
                int _startIndexAngle= 0;
                int _endIndexAngle = 0;
                int _startIndexPosition = 0;
                //determine valid data from header
                for (int i=0; i < header.Count; i++)
                {
                    if (header[i] == "Q0")
                    {
                        _startIndexAngle = i;
                        _endIndexAngle = i+3;
                    }
                        
                    if (header[i] == "Rz")
                    {
                        _startIndexAngle = i;
                        _endIndexAngle = i;
                    }

                    if (header[i] == "Tx")
                    {
                        _startIndexPosition = i;
                        break; //finish searching for index
                    }
                }

                List<double> _anglesByRotation = new List<double>(); //What NDI reads
                List<double> _anglesByPosition = new List<double>(); //What I get from positions
                List<double> _positionX = new List<double>();
                List<double> _positionY = new List<double>();
                foreach (var line in lines)
                {
                    double _angle = 0;
                    double _posX = 0;
                    double _posY = 0;
                    if (_startIndexAngle < _endIndexAngle) //if quaternion
                    {
                        double _w = 0, _x = 0, _y = 0, _z = 0;
                        try
                        {
                            _w = Convert.ToDouble(line[_startIndexAngle]);
                            _x = Convert.ToDouble(line[_startIndexAngle+1]);
                            _y = Convert.ToDouble(line[_startIndexAngle+2]);
                            _z = Convert.ToDouble(line[_startIndexAngle+3]);
                        }
                        catch
                        {
                            _angle = 999999.0;
                        }
                        finally
                        {
                            double _angleSum = Math.Abs(_w + _x + _y + _z);
                            if ((_angle+_angleSum) < 999999.0)
                                _angle = Calc.GetAngleFromQuat(_w, _x, _y, _z);
                        }
                    }
                    else
                    {
                        try
                        {
                            _angle = Convert.ToDouble(line[_startIndexAngle]);
                        }
                        catch
                        {
                            _angle = 999999.0;
                        }
                    }
                    try
                    {
                        _posX = Convert.ToDouble(line[_startIndexPosition]);
                        _posY = Convert.ToDouble(line[_startIndexPosition+1]);
                    }
                    catch
                    {
                        _posX = 999999.0;
                        _posY = 999999.0;
                    }

                    if (Math.Abs(_angle + _posX +_posY)< 999999.0)
                    {
                        _anglesByRotation.Add(_angle);
                        _positionX.Add(_posX);
                        _positionY.Add(_posY);
                    }
                }


                //Calculation
                Center = Calc.FindCenter(_positionX.ToArray(), _positionY.ToArray());
                 
                double[] _angles = Calc.GetAnglesFromPosition(_positionX, _positionY);
                _anglesByPosition = _angles.ToList();

                using (FileStream fs = new FileStream("d:\\test.csv", FileMode.Create))
                {
                    using (StreamWriter sw = new StreamWriter(fs))
                    {
                        sw.WriteLine("{0},{1},{2}", "Measured Angle", "Normal Angle", "Difference Measured-Normal angle");
                        for (int i = 0; i < _anglesByPosition.Count(); ++i)
                        {
                            sw.WriteLine("{0},{1},{2}", _anglesByPosition[i], _anglesByRotation[i], _anglesByPosition[i] - _anglesByRotation[i]);
                        }
                    }
                }

                double[] stats = Calc.Stats(_anglesByPosition, _anglesByRotation);
                AngleCorrection = stats[0];
                AngleDeviation = stats[1];
                AngleMin = stats[2];
                AngleMax = stats[3];
            }
        }
    }

    public static class CsvParser
    {
        private static Tuple<T, IEnumerable<T>> HeadAndTail<T>(this IEnumerable<T> source)
        {
            if (source == null)
                throw new ArgumentNullException("source");
            var en = source.GetEnumerator();
            en.MoveNext();
            return Tuple.Create(en.Current, EnumerateTail(en));
        }

        private static IEnumerable<T> EnumerateTail<T>(IEnumerator<T> en)
        {
            while (en.MoveNext()) yield return en.Current;
        }

        public static IEnumerable<IList<string>> Parse(string content, char delimiter, char qualifier)
        {
            using (var reader = new StringReader(content))
                return Parse(reader, delimiter, qualifier);
        }

        public static Tuple<IList<string>, IEnumerable<IList<string>>> ParseHeadAndTail(TextReader reader, char delimiter, char qualifier)
        {
            return HeadAndTail(Parse(reader, delimiter, qualifier));
        }

        public static IEnumerable<IList<string>> Parse(TextReader reader, char delimiter, char qualifier)
        {
            var inQuote = false;
            var record = new List<string>();
            var sb = new StringBuilder();

            while (reader.Peek() != -1)
            {
                var readChar = (char)reader.Read();

                if (readChar == '\n' || (readChar == '\r' && (char)reader.Peek() == '\n'))
                {
                    // If it's a \r\n combo consume the \n part and throw it away.
                    if (readChar == '\r')
                        reader.Read();

                    if (inQuote)
                    {
                        if (readChar == '\r')
                            sb.Append('\r');
                        sb.Append('\n');
                    }
                    else
                    {
                        if (record.Count > 0 || sb.Length > 0)
                        {
                            record.Add(sb.ToString());
                            sb.Clear();
                        }

                        if (record.Count > 0)
                            yield return record;

                        record = new List<string>(record.Count);
                    }
                }
                else if (sb.Length == 0 && !inQuote)
                {
                    if (readChar == qualifier)
                        inQuote = true;
                    else if (readChar == delimiter)
                    {
                        record.Add(sb.ToString());
                        sb.Clear();
                    }
                    else if (char.IsWhiteSpace(readChar))
                    {
                        // Ignore leading whitespace
                    }
                    else
                        sb.Append(readChar);
                }
                else if (readChar == delimiter)
                {
                    if (inQuote)
                        sb.Append(delimiter);
                    else
                    {
                        record.Add(sb.ToString());
                        sb.Clear();
                    }
                }
                else if (readChar == qualifier)
                {
                    if (inQuote)
                    {
                        if ((char)reader.Peek() == qualifier)
                        {
                            reader.Read();
                            sb.Append(qualifier);
                        }
                        else
                            inQuote = false;
                    }
                    else
                        sb.Append(readChar);
                }
                else
                    sb.Append(readChar);
            }

            if (record.Count > 0 || sb.Length > 0)
                record.Add(sb.ToString());

            if (record.Count > 0)
                yield return record;
        }
    }
}
