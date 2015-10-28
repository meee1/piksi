using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    public class prsmooth
    {
        Dictionary<int, List<double>> datapr = new Dictionary<int, List<double>>();
        Dictionary<int, List<double>> datacp = new Dictionary<int, List<double>>();
        public int size = 100;

        // store history point
        double[] P = new double[255];
        double[] C = new double[255];

        // output
        double[] S = new double[255];

        public prsmooth()
        {
            for (int a = 0; a < 255; a++)
            {
                datapr[a] = new List<double>();
            }
            for (int a = 0; a < 255; a++)
            {
                datacp[a] = new List<double>();
            }
        }

        void resetsat(int sat)
        {
            datapr[sat] = new List<double>();
            datacp[sat] = new List<double>();
        }

        public double Add(int sat, double pr, double cp)
        {
            datapr[sat].Add(pr);

            while (datapr[sat].Count > size)
                datapr[sat].RemoveAt(0);

            datacp[sat].Add(cp);

            while (datacp[sat].Count > size)
                datacp[sat].RemoveAt(0);

            return prsmoothed(sat);
        }

        double prsmoothed(int sat)
        {
            double slip = 1500;
            //double Nmax = 100;
            int prn = sat;

            double Pn = datapr[sat][datapr[sat].Count - 1];
            double Cn = datacp[sat][datapr[sat].Count - 1];



            if ((datapr[sat].Count > 1) && Math.Abs((Pn - P[prn]) - (Cn - C[prn])) > slip)
            { resetsat(sat); return Pn; }// {cycle slip found, re-initialize filter}

            if (datapr[sat].Count == 1)
            { S[prn] = Pn; } //{first observation}
            else
            {// {following observation}
                S[prn] = Pn / datapr[sat].Count + (S[prn] + Cn - C[prn]) * (datapr[sat].Count - 1) / datapr[sat].Count;
            }

            P[prn] = Pn; //{store Pn for next epoch}
            C[prn] = Cn; //{store Cn for next epoch}

            Console.WriteLine("sv {0,2} raw {1} Smooth {2} dl {3,8} prdl {4,8} cpdl {5,8} regdl {6, 8}   ", prn, (Pn).ToString("0.000"), (S[prn]).ToString("0.000"), (Pn - S[prn]).ToString("0.000"), linearRegressionpr(prn).ToString("0.000"), linearRegressioncp(prn).ToString("0.000"), (linearRegressioncp(prn) - linearRegressionpr(prn)).ToString("0.000"));          

            return S[prn];
        }

        public double getSmoothed(int sat)
        {
            return S[sat];
        }

        public double linearRegressionpr(int sat)
        {
            double[] values = datapr[sat].ToArray();
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


        public double linearRegressioncp(int sat)
        {
            double[] values = datacp[sat].ToArray();
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
