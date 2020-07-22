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

                //Calculate 
                Calc.Process()


                this.textBox1.Text = ReadCSV.AngleCorrection.ToString("F5");
                this.textBox2.Text = ReadCSV.AngleDeviation.ToString("F5");
                this.textBox3.Text = ReadCSV.AngleMin.ToString("F5");
                this.textBox4.Text = ReadCSV.AngleMax.ToString("F5");
                StringBuilder sb = new StringBuilder();
                sb.AppendLine(ReadCSV.Center[0].ToString("F5"));
                sb.AppendLine(ReadCSV.Center[1].ToString("F5"));
                sb.AppendLine(ReadCSV.Center[2].ToString("F5"));
                richTextBox1.Text = sb.ToString();
            }
        }

        private static DialogResult GetOK()
        {
            return System.Windows.Forms.DialogResult.OK;
        }
    }
}
