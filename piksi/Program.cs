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

        static SerialPort comport;

        static void Main(string[] args)
        {
            if (args.Length != 3)
            {
                Console.WriteLine("Usage: program.exe outputformat port baud");
                Console.WriteLine("outputformat = rtcm,sbp\nport = [comport of piksi]\nbaud = [baudrate of piksi]");
                Console.WriteLine("the application will output data on port 989");
                return;
            }

            string mode = args[0];
            string port = args[1];
            int baudrate = int.Parse(args[2]);

            listener.Start();

            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);

            /*
            BinaryReader br = new BinaryReader(File.OpenRead(@"C:\Users\hog\Desktop\gps data\rtcm3.11004.raw"));

            while (br.BaseStream.Position < br.BaseStream.Length)
            {
                rtcm.Read(br.ReadByte());
            }

            return;
            */

            comport = new SerialPort(args[1], int.Parse(args[2]));

            comport.Open();

            piksi pk = new piksi();

            // sbp to rtcm
            if (mode.ToLower() == "rtcm")
            {
                pk.ObsMessage += pk_ObsMessage;

                while (true)
                {
                    while (comport.BytesToRead > 0)
                    {
                        pk.read((byte)comport.ReadByte());
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            if (mode.ToLower() == "sbp")
            {
                rtcm.ObsMessage += rtcm_ObsMessage;
                while (true)
                {
                    while (comport.BytesToRead > 0)
                    {
                        rtcm.Read((byte)comport.ReadByte());
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            Console.ReadLine();
        }

        static void rtcm_ObsMessage(object sender, EventArgs e)
        {
            List<RTCM3.ob> msg = (List<RTCM3.ob>)sender;

            byte total = 1;
            byte count = 1;

            piksi.msg_obs_header_t head = new piksi.msg_obs_header_t();
            head.seq = (byte)((total << piksi.MSG_OBS_HEADER_SEQ_SHIFT) | (count & piksi.MSG_OBS_HEADER_SEQ_MASK));
            head.t.wn = (ushort)(msg[0].week);
            head.t.tow = (uint)(msg[0].tow  * piksi.MSG_OBS_TOW_MULTIPLIER);

            List<piksi.msg_obs_content_t> obs = new List<piksi.msg_obs_content_t>();

            foreach (var item in msg)
            {
                piksi.msg_obs_content_t ob = new piksi.msg_obs_content_t();
                ob.prn = item.prn;
                ob.P = (uint)(item.pr * piksi.MSG_OBS_P_MULTIPLIER);
                ob.L.Li = (int)item.cp;
                ob.L.Lf = (byte)((item.cp - ob.L.Li) * 256.0);
                ob.snr = (byte)(item.snr * piksi.MSG_OBS_SNR_MULTIPLIER);
            }

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

                RTCM3.ob rtcmob = new RTCM3.ob();

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
