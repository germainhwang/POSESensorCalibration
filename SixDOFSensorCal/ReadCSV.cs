using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Net.Http.Headers;

namespace SixDOFSensorCal
{
    class ReadCSV
    {
        public static double AngleCorrection = -999.0;
        public static double AngleDeviation = -999.0;
        public static double AngleMax = -999.0;
        public static double AngleMin = -999.0;
        public static double[] Center = new double[3];
        /// <summary>
        /// Reads NDI generated CSV file containing orientation and position data
        /// Parses and makes lists of either Quaternion or Euler angles and XYZ positions
        /// Then performs circle fitting to the data and angle offset between the roll angle as the sensor is mounted and the normal angle at positions
        /// </summary>
        /// <param name="filename"></param>
        public static List<double[]> ParseNDIFile(string filename)
        {
            List<double[]> parsedData = new List<double[]>();
            //Reading CSV
            using (var reader = new StreamReader(File.OpenRead(filename)))
            {
                var data = CsvParser.ParseHeadAndTail(reader, ',', '"');

                var header = data.Item1;
                var lines = data.Item2;
                int startIndexAngle= 0;
                int endIndexAngle = 0;
                int startIndexPosition = 0;
                int arrayLength = 0;
                int dataStateIndex = 0;
                //determine valid data from header
                for (int i=0; i < header.Count; i++)
                {
                    if (header[i] == "State")
                        dataStateIndex = i;
                    if (header[i] == "Q0")
                    {
                        // If csv contains quarternion angles, make end index larger than start index
                        startIndexAngle = i;
                        endIndexAngle = i+3;
                        arrayLength = 7;
                    }
                        
                    if (header[i] == "Rz")
                    {
                        // If Euler angles, make start and end index same number.
                        startIndexAngle = i;
                        endIndexAngle = i;
                        arrayLength = 6;
                    }

                    if (header[i] == "Tx")
                    {
                        startIndexPosition = i;
                        break; //finish searching for index
                    }
                }

                
                
                foreach (var line in lines)
                {
                    double[] parsedLine = new double[arrayLength];
                    //double _angle = 0;
                    double _posX = 0;
                    double _posY = 0;
                    double _posZ = 0;
                    //double[] _rotMat = new double[9];
                    if (line[dataStateIndex] == "OK")
                    {
                        if (startIndexAngle < endIndexAngle) //if quaternion
                        {
                            double _w = 0, _x = 0, _y = 0, _z = 0;
                            try
                            {
                                _w = Convert.ToDouble(line[startIndexAngle]);
                                _x = Convert.ToDouble(line[startIndexAngle + 1]);
                                _y = Convert.ToDouble(line[startIndexAngle + 2]);
                                _z = Convert.ToDouble(line[startIndexAngle + 3]);
                            }
                            catch
                            {
                                _w = 999999.0;
                            }
                            finally
                            {
                                //I could check if sqaure sum of quaternion angles is 1
                                double _angleSum = Math.Abs(_w + _x + _y + _z);
                                if (_w < 999999.0)
                                {
                                    parsedLine[3] = _w;
                                    parsedLine[4] = _x;
                                    parsedLine[5] = _y;
                                    parsedLine[6] = _z;
                                }
                            }
                        }
                        else
                        {
                            double _rx = 0, _ry = 0, _rz = 0;
                            try
                            {
                                _rz = Convert.ToDouble(line[startIndexAngle]);
                                _ry = Convert.ToDouble(line[startIndexAngle + 1]);
                                _rx = Convert.ToDouble(line[startIndexAngle + 2]);
                            }
                            catch
                            {
                                _rz = 999999.0;
                            }
                            finally
                            {
                                parsedLine[3] = _rx;
                                parsedLine[4] = _ry;
                                parsedLine[5] = _rz;
                            }
                        }
                        try
                        {
                            _posX = Convert.ToDouble(line[startIndexPosition]);
                            _posY = Convert.ToDouble(line[startIndexPosition + 1]);
                            _posZ = Convert.ToDouble(line[startIndexPosition + 2]);
                        }
                        catch
                        {
                            _posX = 999999.0;
                        }

                        if (_posX < 999999.0) //check for any invalid value - NDI returns non numeric value                    {
                        {
                            parsedLine[0] = _posX;
                            parsedLine[1] = _posY;
                            parsedLine[2] = _posZ;
                            parsedData.Add(parsedLine);
                        }
                    }
                }
            }
            return parsedData;
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
