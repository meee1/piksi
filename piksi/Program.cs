using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    class Program
    {
        static void Main(string[] args)
        {

            RTCM3 rtcm = new RTCM3();

            rtcm.gen_rtcm();

            BinaryReader br = new BinaryReader(File.OpenRead(@"C:\Users\hog\Desktop\gps data\rtcm3.1.raw"));

            while (br.BaseStream.Position < br.BaseStream.Length)
            {
                rtcm.Read(br.ReadByte());
            }

            return;

            SerialPort comport = new SerialPort("com31", 1000000);
            //SerialPort comport = new SerialPort("com15", 115200);

            comport.Open();

            /*
            BinaryReader sr = new BinaryReader(File.OpenRead(@"H:\piksi.raw"));

            piksi pk = new piksi();

            while (sr.BaseStream.Position < sr.BaseStream.Length)
            {
                pk.read((byte)sr.ReadByte());
            }

            //sr.Close();
            */

            piksi pk = new piksi();

            while (true) 
            {
                while (comport.BytesToRead > 0)
                {
                    pk.read((byte)comport.ReadByte());
                }

                System.Threading.Thread.Sleep(5);
            }

            Console.ReadLine();
        }
    }
}
