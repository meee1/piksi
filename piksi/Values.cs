using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    public class values
    {
        Dictionary<int, List<double>> data = new Dictionary<int, List<double>>();
        public int size = 30;

        public values()
        {
            for (int a = 0; a < 255; a++)
            {
                data[a] = new List<double>();
            }
        }

        public void Add(int sat, double pr)
        {
            data[sat].Add(pr);

            while (data[sat].Count > size)
                data[sat].RemoveAt(0);
        }

        public double average(int sat)
        {
            double total = 0;
            foreach (double item in data[sat])
            {
                total += item;
            }

            return total / data[sat].Count;
        }

        public double lastValue(int sat)
        {
            try
            {
                return data[sat][data[sat].Count - 1];
            }
            catch { return 0; }
        }

        public double linearRegression(int sat)
        {
            double[] values = data[sat].ToArray();
            //double[] values = { 4.8, 4.8, 4.5, 3.9, 4.4, 3.6, 3.6, 2.9, 3.5, 3.0, 2.5, 2.2, 2.6, 2.1, 2.2 };

            double xAvg = 0;
            double yAvg = 0;

            for (int x = 0; x < values.Length; x++)
            {
                xAvg += x;
                yAvg += values[x];
            }

            xAvg = xAvg / values.Length;
            yAvg = yAvg / values.Length;


            double v1 = 0;
            double v2 = 0;

            for (int x = 0; x < values.Length; x++)
            {
                v1 += (x - xAvg) * (values[x] - yAvg);
                v2 += Math.Pow(x - xAvg, 2);
            }

            double a = v1 / v2;
            double b = yAvg - a * xAvg;


            //Console.Write("y = ax + b ");
            // Console.WriteLine("a = {0}, the slope of the trend line. ", Math.Round(a, 2));
            //Console.WriteLine("b = {0}, the intercept of the trend line.", Math.Round(b, 2));

            if (double.IsNaN(a))
                a = 0;

            return a;

            //Console.ReadLine();
        }
    }
}
