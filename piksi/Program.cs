using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    class Program
    {
        static RTCM3 rtcm = new RTCM3();

        static TcpListener listener = new TcpListener(989);

        static TcpClient client;

        static void Main(string[] args)
        {
            listener.Start();

            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);

            //rtcm.gen_rtcm();

            BinaryReader br = new BinaryReader(File.OpenRead(@"C:\Users\hog\Desktop\gps data\rtcm3.1.raw"));

            while (br.BaseStream.Position < br.BaseStream.Length)
            {
                rtcm.Read(br.ReadByte());
            }

         //   return;

            SerialPort comport = new SerialPort("com37", 115200);
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

            pk.ObsMessage += pk_ObsMessage;

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

     private static void DoAcceptTcpClientCallback(IAsyncResult ar)
        {
            // Get the listener that handles the client request.
            TcpListener listener = (TcpListener)ar.AsyncState;

            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);

            // End the operation and display the received data on  
            // the console.
            using (
            client = listener.EndAcceptTcpClient(ar))
            {
                while (client.Connected)
                {
                    System.Threading.Thread.Sleep(100);
                }
            }
        }

        static void pk_ObsMessage(object sender, EventArgs e)
        {
            piksi.header msg = (piksi.header)sender;

            var hdr = msg.payload.ByteArrayToStructure<piksi.msg_obs_header_t>(0);

            // total is number of packets
            int total = hdr.seq >> piksi.MSG_OBS_HEADER_SEQ_SHIFT;
            // this is packet count number
            int count = hdr.seq & piksi.MSG_OBS_HEADER_SEQ_MASK;

            int lenhdr = Marshal.SizeOf(hdr);

            int lenobs = Marshal.SizeOf(new piksi.msg_obs_content_t());

            int obscount = (msg.length - lenhdr) / lenobs;

            int linebase = (count > 0) ? 8 : 0;

            RTCM3.type1002 t1002 = new RTCM3.type1002();

            for (int a = 0; a < obscount; a++)
            {
                var ob = msg.payload.ByteArrayToStructure<piksi.msg_obs_content_t>(lenhdr + a * lenobs);

                RTCM3.type1002.ob rtcmob = new RTCM3.type1002.ob();

                rtcmob.prn = ob.prn;
                rtcmob.snr = (byte)(ob.snr);
                rtcmob.pr = (ob.P / piksi.MSG_OBS_P_MULTIPLIER);
                rtcmob.cp = ob.L.Li + (ob.L.Lf / 256.0);
                rtcmob.week = hdr.t.wn;
                rtcmob.tow = hdr.t.tow;

                t1002.obs.Add(rtcmob);
            }

            byte[] rtcmpacket = rtcm.gen_rtcm(t1002);

            try
            {
                if (client != null && client.Connected)
                    client.GetStream().Write(rtcmpacket, 0, rtcmpacket.Length);
            }
            catch { }
        }
    }
}
