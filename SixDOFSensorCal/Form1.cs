using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;

namespace SixDOFSensorCal
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void buttonOpenFile_Click(object sender, EventArgs e)
        {
            if (openFileDialog1.ShowDialog() == GetOK())
            {
                //Read CSV file and get Pose data in double[]
                //{X, Y, Z, Qw, Qx, Qy, Qz} or
                //{X, Y, Z, Rx, Ry, Rz}
                List<double[]> poseData = new List<double[]>();
                poseData.AddRange(ReadCSV.ParseNDIFile(openFileDialog1.FileName));

                //Perform circle fitting on the points and calculate roll angle offset
                //return doulbe array in the format
                //{offset angle, stdev, min, max, x center, y center, radius}
                double[] results = Calc.Process(poseData);


                this.textBox1.Text = results[0].ToString("F5");
                this.textBox2.Text = results[1].ToString("F5");
                this.textBox3.Text = results[2].ToString("F5");
                this.textBox4.Text = results[3].ToString("F5");
                StringBuilder sb = new StringBuilder();
                sb.AppendLine(results[4].ToString("F5"));
                sb.AppendLine(results[5].ToString("F5"));
                sb.AppendLine(results[6].ToString("F5"));
                richTextBox1.Text = sb.ToString();
            }
        }

        private static DialogResult GetOK()
        {
            return System.Windows.Forms.DialogResult.OK;
        }
    }
}
