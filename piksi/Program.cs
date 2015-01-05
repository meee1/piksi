using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using piksi.Comms;

namespace piksi
{
    class Program
    {
        static RTCM3 rtcm = new RTCM3();

        static Comms.IStreamExtra inputstream;

        static Comms.IStreamExtra deststream;

        static StreamWriter rinexoutput;

        static piksi piksi = new piksi();

        /*
import sbp_piksi
link.link.send_message(sbp_piksi.SETTINGS, 'uart_ftdi\0mode\0SBP\0')

import sbp_piksi
self.link.send_message(sbp_piksi.SETTINGS, 'uart_ftdi\0baudrate\0%s\0' % ('1000000'.encode('ascii')))
         
import sbp_piksi
self.link.send_message(sbp_piksi.SETTINGS, 'uart_uarta\0sbp_message_mask\0%s\0' % ('65535'.encode('ascii')))

self.link.send_message(sbp_piksi.RESET, '')
         */

        static void Main(string[] args)
        {
            Console.Clear();

            if (args.Length < 3)
            {
                Console.WriteLine("Piksi v0.1.3 beta By Michael Oborne");
                Console.WriteLine("Copyright Michael Oborne 2015");
                Console.WriteLine("Usage: program.exe outputformat source destination");
                Console.WriteLine("outputformat = rtcm, sbp, trimble, rinex");
                Console.WriteLine("rtcm = read sbp from source and output rtcm to destination");
                Console.WriteLine("sbp = read rtcm from source and output sbp to destination");
                Console.WriteLine("trimble = read sbp from source and output trimble rt17 to destination");
                Console.WriteLine("rinex = read sbp from source file and output rinex to destination file (Files only)");
                Console.WriteLine();
                Console.WriteLine("source/destination can be 'COM? 115200' or 'tcp://host:port' or 'ntrip://user:pass@host/source' or 'tcp://0.0.0.0:989'");
                
                Console.ReadLine();
                return;
            }

            string outmode = args[0];

            // rinex output
            if (outmode.ToLower() == "rinex")
            {
                rinexoutput = new StreamWriter(args[2]);

                rinexoutput.WriteLine(@"     3.02           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE
                                                            MARKER NAME         
                                                            MARKER NUMBER       
                                                            MARKER TYPE         
                                                            OBSERVER / AGENCY   
                                                            REC # / TYPE / VERS 
                                                            ANT # / TYPE        
        0.0000        0.0000        0.0000                  APPROX POSITION XYZ 
        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N
G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES 
G                                                           SYS / PHASE SHIFT   
                                                            END OF HEADER       ");

                piksi.ObsMessage += pkrinex_ObsMessage;

                BinaryReader br = new BinaryReader(File.OpenRead(args[1]));

                while (br.BaseStream.Position < br.BaseStream.Length)
                {
                    piksi.read(br.ReadByte());
                }

                rinexoutput.Close();

                return;
            }
            
            // process other commands
            int nextarg = 2;
            string port = args[1];

            // input
            if (port.ToLower().Contains("tcp://0.0.0.0"))
            {
                inputstream = new TCPServer(int.Parse(port.Split(':')[2]));
                nextarg = 2;
            } 
            else if (port.ToLower().Contains("tcp://"))
            {
                inputstream = new TCPClient(port.ToLower().Replace("tcp://",""), int.Parse(port.Split(':')[2]));
                nextarg = 2;
            } 
            else if (port.ToLower().Contains("ntrip://"))
            {
                inputstream = new NTRIP(port.Replace("ntrip://", ""));

                nextarg = 2;
            }
            else
            {
                int baudrate = int.Parse(args[2]);
                inputstream = new SerialPort(port,baudrate);

                nextarg = 3;            
            }

            inputstream.Open();

            string portout = args[nextarg];
            // output
            if (port.ToLower().Contains("tcp://0.0.0.0"))
            {
                deststream = new TCPServer(int.Parse(portout.Split(':')[2]));
                nextarg = 2;
            }
            else if (portout.ToLower().Contains("tcp://"))
            {
                deststream = new TCPClient(portout.ToLower().Replace("tcp://", ""), int.Parse(portout.Split(':')[2]));
                nextarg++;
            }
            else if (portout.ToLower().Contains("ntrip://"))
            {
                deststream = new NTRIP(portout.Replace("ntrip://", ""));

                nextarg++;
            }
            else
            {
                int baudrate = int.Parse(args[nextarg + 1]);
                deststream = new SerialPort(portout, baudrate);

                nextarg = nextarg + 2;
            }

            deststream.Open();

            if (outmode.ToLower() == "trimble")
            {
                piksi.ObsMessage += pktrimble_ObsMessage;
                piksi.EphMessage += pktrimble_EphMessage;

                DateTime ephdeadline = DateTime.Now.AddSeconds(12);

                while (true)
                {
                    while (inputstream.dataToRead)
                    {
                        piksi.read((byte)inputstream.ReadByte());
                    }

                    if (ephdeadline < DateTime.Now)
                    {
                        string[] files = Directory.GetFiles(".", "*.eph");
                        foreach (var ephfile in files)
                        {
                            var info = new FileInfo(ephfile);

                            // check age
                            if (info.LastWriteTime.AddHours(4) < DateTime.Now)
                                continue;

                            byte[] ephbytes = File.ReadAllBytes(ephfile);

                            piksi.header msg = new piksi.header();

                            msg.payload = ephbytes;

                            pktrimble_EphMessage(msg, null);
                        }

                        ephdeadline = DateTime.Now.AddSeconds(30);
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            // sbp to rtcm
            if (outmode.ToLower() == "rtcm")
            {
                piksi.ObsMessage += pkrtcm_ObsMessage;

                while (true)
                {
                    while (inputstream.dataToRead)
                    {
                        piksi.read((byte)inputstream.ReadByte());
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            if (outmode.ToLower() == "sbp")
            {
                rtcm.ObsMessage += rtcm_ObsMessage;
                rtcm.BasePosMessage += rtcm_BasePosMessage;

                while (true)
                {
                    while (inputstream.dataToRead)
                    {
                        rtcm.Read((byte)inputstream.ReadByte());
                    }

                    System.Threading.Thread.Sleep(5);
                }
            }

            Console.ReadLine();
        }

        static void pktrimble_EphMessage(object sender, EventArgs e)
        {
            piksi.header msg = (piksi.header)sender;

            var eph = msg.payload.ByteArrayToStructure<piksi.ephemeris_t>(0);

            if (eph.valid > 0)
            {
                try
                {
                    Trimble.writeTrimble55_1(deststream, eph, eph.prn + 1);
                }
                catch { }
            }
        }

        const double CLIGHT = 299792458.0;   /* speed of light (m/s) */
        static double wl = CLIGHT / 1.57542E9;

        private static void pktrimble_ObsMessage(object sender, EventArgs e)
        {
            piksi.header msg = (piksi.header)sender;

            var hdr = msg.payload.ByteArrayToStructure<piksi.msg_obs_header_t>(0);

            // relay packet
            if (msg.sender == 0)
                return;

            // total is number of packets
            int total = hdr.seq >> piksi.MSG_OBS_HEADER_SEQ_SHIFT;
            // this is packet count number
            int count = hdr.seq & piksi.MSG_OBS_HEADER_SEQ_MASK;

            int lenhdr = Marshal.SizeOf(hdr);

            int lenobs = Marshal.SizeOf(new piksi.msg_obs_content_t());

            int obscount = (msg.length - lenhdr) / lenobs;

            DateTime gpstime = StaticUtils.GetFromGps(hdr.t.wn, hdr.t.tow / 1000.0);

            DateTime local = gpstime.ToLocalTime();

            List<piksi.msg_obs_content_t> obs = new List<piksi.msg_obs_content_t>();

            for (int a = 0; a < obscount; a++)
            {
                var ob = msg.payload.ByteArrayToStructure<piksi.msg_obs_content_t>(lenhdr + a * lenobs);

                obs.Add(ob);
            }

            if (deststream != null)
            {
                try
                {
                    if (DateTime.Now.Second % 10 == 0)
                        Trimble.writeTrimble15(deststream, hdr.t.wn, hdr.t.tow);

                    Trimble.writeTrimbleR17(deststream, hdr.t.tow, obs);
                } catch (Exception) 
                {

                }
            }
        }

        private static void pkrinex_ObsMessage(object sender, EventArgs e)
        {
            if (rinexoutput == null)
                return;

            piksi.header msg = (piksi.header)sender;

            var hdr = msg.payload.ByteArrayToStructure<piksi.msg_obs_header_t>(0);

            // relay packet
            if (msg.sender == 0)
                return;

            // total is number of packets
            int total = hdr.seq >> piksi.MSG_OBS_HEADER_SEQ_SHIFT;
            // this is packet count number
            int count = hdr.seq & piksi.MSG_OBS_HEADER_SEQ_MASK;

            int lenhdr = Marshal.SizeOf(hdr);

            int lenobs = Marshal.SizeOf(new piksi.msg_obs_content_t());

            int obscount = (msg.length - lenhdr) / lenobs;

            DateTime gpstime = StaticUtils.GetFromGps(hdr.t.wn, hdr.t.tow / 1000.0);

            DateTime local = gpstime.ToLocalTime();

            rinexoutput.WriteLine("> {0,4} {1,2} {2,2} {3,2} {4,2} {5,10} {6,1} {7,-2}", gpstime.Year, gpstime.Month, gpstime.Day, gpstime.Hour, gpstime.Minute, (gpstime.Second + (gpstime.Millisecond / 1000.0)).ToString("0.0000000",System.Globalization.CultureInfo.InvariantCulture), 0, obscount);

            for (int a = 0; a < obscount; a++)
            {
                var ob = msg.payload.ByteArrayToStructure<piksi.msg_obs_content_t>(lenhdr + a * lenobs);

                rinexoutput.WriteLine("G{0,2} {1,13} {2,15} {3,31}", (ob.prn+1).ToString("00"), (ob.P / piksi.MSG_OBS_P_MULTIPLIER).ToString("0.000", System.Globalization.CultureInfo.InvariantCulture), ((ob.L.Li + (ob.L.Lf / 256.0))).ToString("0.0000", System.Globalization.CultureInfo.InvariantCulture), ob.snr.ToString("0.000", System.Globalization.CultureInfo.InvariantCulture));
            }
        }

        static double RE_WGS84 = 6378137.0;          /* earth semimajor axis (WGS84) (m) */

        static double FE_WGS84 = (1.0 / 298.257223563); /* earth flattening (WGS84) */

        const double PI = Math.PI; /* pi */

        const double D2R = (PI / 180.0);   /* deg to rad */
        const double R2D = (180.0 / PI);   /* rad to deg */

        static double dot(double[] a, double[] b, int n)
        {
            double c = 0.0;

            while (--n >= 0) c += a[n] * b[n];
            return c;
        }

        static double fabs(double input)
        {
            return Math.Abs(input);
        }

        static double atan(double input)
        {
            return Math.Atan(input);
        }
        static double atan2(double input, double input2)
        {
            return Math.Atan2(input, input2);
        }

        static double sqrt(double input)
        {
            return Math.Sqrt(input);
        }

        static void ecef2pos(double[] r, ref double[] pos)
        {
            double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

            for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4; )
            {
                zk = z;
                sinp = z / sqrt(r2 + z * z);
                v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
                z = r[2] + v * e2 * sinp;
            }
            pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
            pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
            pos[2] = sqrt(r2 + z * z) - v;
        }

        static void rtcm_BasePosMessage(object sender, EventArgs e)
        {
            var msg1 = sender as RTCM3.type1005;
            var msg2 = sender as RTCM3.type1006;

            if (msg1 != null)
            {
                piksi.msg_base_pos_t bpos = new piksi.msg_base_pos_t();

                // in radians
                double[] llhr = new double[3];

                ecef2pos(new double[] { msg1.rr0 * 0.0001, msg1.rr1 * 0.0001, msg1.rr2 * 0.0001 }, ref llhr);

                bpos.pos_lat = llhr[0] * R2D;
                bpos.pos_lon = llhr[1] * R2D;
                bpos.pos_alt = llhr[2];

                byte[] packet = piksi.GeneratePacket(bpos, piksi.MSG.MSG_BASE_POS);

                inputstream.Write(packet, 0, packet.Length);
            }
            if (msg2 != null)
            {
                piksi.msg_base_pos_t bpos = new piksi.msg_base_pos_t();

                // in radians
                double[] llhr = new double[3];

                ecef2pos(new double[] { msg2.rr0 * 0.0001, msg2.rr1 * 0.0001, msg2.rr2 * 0.0001 }, ref llhr);

                bpos.pos_lat = llhr[0] * R2D;
                bpos.pos_lon = llhr[1] * R2D;
                bpos.pos_alt = llhr[2];

                byte[] packet = piksi.GeneratePacket(bpos, piksi.MSG.MSG_BASE_POS);

                inputstream.Write(packet, 0, packet.Length);
            }
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
            //head.t.tow = (uint)(Math.Round((decimal)(head.t.tow / 1000.0))*(decimal)1000.0);

            double soln_freq = 10;
            double obs_output_divisor = 2;

            double epoch_count = (head.t.tow / piksi.MSG_OBS_TOW_MULTIPLIER) * (soln_freq / obs_output_divisor);

            double checkleft = Math.Abs(epoch_count - Math.Round(epoch_count));

            Console.WriteLine(head.t.tow + " " + checkleft.ToString("0.000") + " " + epoch_count.ToString("0.000") + " " + Math.Round(epoch_count) + " > " + 1e-3);

            List<piksi.msg_obs_content_t> obs = new List<piksi.msg_obs_content_t>();

            foreach (var item in msg)
            {
                item.cp *= -1;

                piksi.msg_obs_content_t ob = new piksi.msg_obs_content_t();
                ob.prn = (byte)(item.prn-1);
                ob.P = (uint)(item.pr * piksi.MSG_OBS_P_MULTIPLIER);
                ob.L.Li = (int)item.cp;
                ob.L.Lf = (byte)((item.cp - ob.L.Li) * 256.0);
                ob.snr = (byte)(item.snr);// / piksi.MSG_OBS_SNR_MULTIPLIER);

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
            inputstream.Write(allbytes, 0, allbytes.Length);

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
            /*using (
            client = listener.EndAcceptTcpClient(ar))
            {
                while (client.Connected)
                {
                    System.Threading.Thread.Sleep(100);
                }
            }*/
        }

     private static void DoAcceptTcpClientCallbackraw(IAsyncResult ar)
     {
         // Get the listener that handles the client request.
         TcpListener listener = (TcpListener)ar.AsyncState;

         listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallbackraw), listener);

         // End the operation and display the received data on  
         // the console.
         /*using (
         clientraw = listener.EndAcceptTcpClient(ar))
         {
             while (clientraw.Connected)
             {
                 System.Threading.Thread.Sleep(100);
             }
         }*/
     }

     private static int[] lockcount = new int[33];

        static void pkrtcm_ObsMessage(object sender, EventArgs e)
        {
            piksi.header msg = (piksi.header)sender;

            var hdr = msg.payload.ByteArrayToStructure<piksi.msg_obs_header_t>(0);

            // relay packet
            if (msg.sender == 0)
                return;

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

                rtcmob.prn = (byte)(ob.prn+1);
                rtcmob.snr = (byte)(ob.snr);
                rtcmob.pr = (ob.P / piksi.MSG_OBS_P_MULTIPLIER);
                rtcmob.cp = -(ob.L.Li + (ob.L.Lf / 256.0));
                rtcmob.week = hdr.t.wn;
                rtcmob.tow = hdr.t.tow;

                if (lockcount[rtcmob.prn] == ob.lock_counter)
                {
                    rtcmob.raw.lock1 = 127;
                }

                lockcount[rtcmob.prn] = ob.lock_counter;

                t1002.obs.Add(rtcmob);
            }

            byte[] rtcmpacket = rtcm.gen_rtcm(t1002);

            try
            {
                if (deststream != null)
                    deststream.Write(rtcmpacket, 0, rtcmpacket.Length);
            }
            catch { }
        }
    }
}
