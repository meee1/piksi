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

        static TcpListener listenerraw = new TcpListener(990);

        static TcpClient client;

        static TcpClient clientraw;

        static SerialPort comport;

        static piksi pk = new piksi();

        static void Main(string[] args)
        {
            if (args.Length != 3)
            {
                Console.WriteLine("Usage: program.exe outputformat port baud");
                Console.WriteLine("outputformat = rtcm,sbp\nport = [comport of piksi]\nbaud = [baudrate of piksi]");
                Console.WriteLine("the application will output data on port 989");
                return;
            }

            string outmode = args[0];
            string port = args[1];
            int baudrate = int.Parse(args[2]);

            listener.Start();

            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);

            listenerraw.Start();

            listenerraw.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallbackraw), listenerraw);

            /*
            BinaryReader br = new BinaryReader(File.OpenRead(@"C:\Users\hog\Desktop\gps data\rtcm3.11004.raw"));

            while (br.BaseStream.Position < br.BaseStream.Length)
            {
                rtcm.Read(br.ReadByte());
            }

            return;
            */

            /*
            BinaryReader br = new BinaryReader(File.OpenRead(@"C:\Users\hog\Desktop\gps data\sbp.raw"));

            while (br.BaseStream.Position < br.BaseStream.Length)
            {
                pk.read(br.ReadByte());
            }

            return;
            */

            comport = new SerialPort(args[1], int.Parse(args[2]));

            comport.Open();           

            // sbp to rtcm
            if (outmode.ToLower() == "rtcm")
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

            if (outmode.ToLower() == "sbp")
            {
                rtcm.ObsMessage += rtcm_ObsMessage;
                while (true)
                {
                    while (client != null && client.Available > 0)
                    {
                        rtcm.Read((byte)client.GetStream().ReadByte());
                    }

                    while (comport.BytesToRead > 0)
                    {
                        try {
                            byte[] data = new byte[1000];
                            int len = comport.Read(data,0,data.Length);
                            if (clientraw != null && clientraw.Connected)
                                clientraw.GetStream().Write(data, 0, len);
                            //pk.read((byte)comport.ReadByte());
                        } catch {}
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            Console.ReadLine();
        }

        static void rtcm_ObsMessage(object sender, EventArgs e)
        {
            List<RTCM3.ob> msg = (List<RTCM3.ob>)sender;

            if (msg.Count == 0)
                return;

            byte total = 1;
            byte count = 0;

            piksi.msg_obs_header_t head = new piksi.msg_obs_header_t();
            head.seq = (byte)((total << piksi.MSG_OBS_HEADER_SEQ_SHIFT) | (count & piksi.MSG_OBS_HEADER_SEQ_MASK));
            head.t.wn = (ushort)(msg[0].week);
            head.t.tow = (uint)((msg[0].tow * piksi.MSG_OBS_TOW_MULTIPLIER));

            // rounding - should not need this, but testing against a ublox requires some "lieing"
            double addextra = (10 - head.t.tow % 10);

            //head.t.tow += (uint)addextra;

            double soln_freq = 10;
            double obs_output_divisor = 2;

            double epoch_count = (head.t.tow / piksi.MSG_OBS_TOW_MULTIPLIER) * (soln_freq / obs_output_divisor);

            double checkleft = Math.Abs(epoch_count - Math.Round(epoch_count));

            Console.WriteLine(head.t.tow + " " + checkleft.ToString("0.000") + " " + epoch_count.ToString("0.000") + " " + Math.Round(epoch_count) + " > " + 1e-3);

            List<piksi.msg_obs_content_t> obs = new List<piksi.msg_obs_content_t>();

            foreach (var item in msg)
            {
                piksi.msg_obs_content_t ob = new piksi.msg_obs_content_t();
                ob.prn = (byte)(item.prn-1);
                ob.P = (uint)(item.pr * piksi.MSG_OBS_P_MULTIPLIER);
                ob.L.Li = (int)item.cp;
                ob.L.Lf = (byte)((item.cp - ob.L.Li) * 256.0);
                ob.snr = (byte)(item.snr * piksi.MSG_OBS_SNR_MULTIPLIER);

                obs.Add(ob);
            }


            //create piksi packet

            piksi.header msgpreamble = new piksi.header();

            int lenpre = Marshal.SizeOf(msgpreamble) - 1; // 8
            int lenhdr = Marshal.SizeOf(head);
            int lenobs = Marshal.SizeOf(new piksi.msg_obs_content_t());

            byte[] allbytes = new byte[lenpre + lenhdr + lenobs * obs.Count];

            msgpreamble.crc = 0x1234;
            msgpreamble.preamble = 0x55;
            msgpreamble.msgtype = (ushort)piksi.MSG.MSG_PACKED_OBS;
            msgpreamble.sender = 1;
            msgpreamble.length = (byte)(obs.Count * lenobs + lenhdr);
            msgpreamble.payload = new byte[msgpreamble.length];

            int payloadcount = (lenpre-2) + lenhdr; // exclude checksum

            foreach (var ob in obs)
            {
                byte[] obbytes = StaticUtils.StructureToByteArray(ob);
                Array.Copy(obbytes, 0, allbytes, payloadcount, obbytes.Length);
                payloadcount += lenobs;
            }

            byte[] preamblebytes = StaticUtils.StructureToByteArray(msgpreamble);

            Array.Copy(preamblebytes, 0, allbytes, 0, preamblebytes.Length-2);

            byte[] headbytes = StaticUtils.StructureToByteArray(head);

            Array.Copy(headbytes, 0, allbytes, lenpre-2, headbytes.Length);

            Crc16Ccitt crc = new Crc16Ccitt(InitialCrcValue.Zeros);
            ushort crcpacket = 0;
            for (int i = 1; i < (allbytes.Length-2); i++)
            {
                crcpacket = crc.Accumulate(allbytes[i], crcpacket);
            }

            allbytes[allbytes.Length - 2] = (byte)(crcpacket & 0xff);
            allbytes[allbytes.Length - 1] = (byte)((crcpacket >> 8) & 0xff);


            //Console.WriteLine();
            comport.Write(allbytes, 0, allbytes.Length);

            //foreach (var ch in allbytes)
            {
               // pk.read(ch);
            }

            //Console.ReadLine();
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

     private static void DoAcceptTcpClientCallbackraw(IAsyncResult ar)
     {
         // Get the listener that handles the client request.
         TcpListener listener = (TcpListener)ar.AsyncState;

         listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallbackraw), listener);

         // End the operation and display the received data on  
         // the console.
         using (
         clientraw = listener.EndAcceptTcpClient(ar))
         {
             while (clientraw.Connected)
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
