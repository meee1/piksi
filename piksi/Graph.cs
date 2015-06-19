using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using ZedGraph;

namespace piksi
{
    public partial class Graph : Form
    {
        RollingPointPairList line1 = new RollingPointPairList(500);
        RollingPointPairList line2 = new RollingPointPairList(500);
        RollingPointPairList line3 = new RollingPointPairList(500); 
        RollingPointPairList line4 = new RollingPointPairList(500);
        RollingPointPairList line5 = new RollingPointPairList(500);
        RollingPointPairList line6 = new RollingPointPairList(500);

        public static Graph instance;

        public Graph()
        {
            InitializeComponent();

            instance = this;

            createGraph();

            timer1.Start();
        }

        void createGraph()
        {
            GraphPane myPane = zedGraphControl1.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = "GPS";
            myPane.XAxis.Title.Text = "Time";
            myPane.XAxis.Type = AxisType.Date;
            myPane.YAxis.Title.Text = "Deltas";

            LineItem myCurve;

            myCurve = myPane.AddCurve("line1", line1, Color.Red, SymbolType.None);
            myCurve = myPane.AddCurve("line2", line2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("line3", line3, Color.Blue, SymbolType.None);
            myCurve.IsY2Axis = true;
            myCurve = myPane.AddCurve("line4", line4, Color.Purple, SymbolType.None);
            myCurve.IsY2Axis = true;
            myCurve = myPane.AddCurve("line5", line5, Color.Orange, SymbolType.None);
            myCurve = myPane.AddCurve("line6", line6, Color.Pink, SymbolType.None);
        }

        public void AddData(int line, double data)
        {
            if (Math.Abs(data) > 10000)
                return;

            var date = new XDate(DateTime.Now);

            switch (line)
            {
                case 1:
                    line1.Add(date, data);
                    break;
                case 2:
                    line2.Add(date, data);
                    break;
                case 3:
                    line3.Add(date, data);
                    break;
                case 4:
                    line4.Add(date, data);
                    break;
                case 5:
                    line5.Add(date, data);
                    break;
                case 6:
                    line6.Add(date, data);
                    break;
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //zedGraphControl1.ZoomOutAll(zedGraphControl1.GraphPane);

            // Calculate the Axis Scale Ranges
            try
            {
                zedGraphControl1.AxisChange();
            }
            catch { }

            zedGraphControl1.Invalidate();
        }
    }
}

