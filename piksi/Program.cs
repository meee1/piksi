using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    class Program
    {
        static void Main(string[] args)
        {
            BinaryReader sr = new BinaryReader(File.OpenRead(@"H:\piksi.raw"));

            piksi pk = new piksi();

            while (sr.BaseStream.Position < sr.BaseStream.Length)
            {
                pk.read((byte)sr.ReadByte());
            }

            sr.Close();

            Console.ReadLine();
        }
    }
}
