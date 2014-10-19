using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using u8 = System.Byte;
using u16 = System.UInt16;
using s32 = System.Int32;
using u32 = System.UInt32;
using gps_time_t = System.UInt64;
using System.Runtime.InteropServices;

namespace piksi
{
    public class piksi
    {
        public event EventHandler ObsMessage;
        public event EventHandler BasePosMessage;

        public enum MSG
        {
            MSG_PRINT = 0x10, /**< Piksi  -> Host  */
            MSG_DEBUG_VAR = 0x11, /**< Piksi  -> Host  */

            MSG_ALMANAC = 0x69, /**< Host   -> Piksi */
            MSG_SET_TIME = 0x68, /**< Host   -> Piksi */

            MSG_BOOTLOADER_HANDSHAKE = 0xB0,  /**< Host  <-> Piksi */
            MSG_BOOTLOADER_JUMP_TO_APP = 0xB1,  /**< Host   -> Piksi */

            MSG_RESET = 0xB2,  /**< Host   -> Piksi */

            MSG_CW_RESULTS = 0xC0, /**< Piksi  -> Host  */
            MSG_CW_START = 0xC1, /**< Host   -> Piksi */

            MSG_NAP_DEVICE_DNA = 0xDD, /**< Host  <-> Piksi */

            MSG_FLASH_PROGRAM = 0xE0, /**< Host   -> Piksi */
            MSG_FLASH_DONE = 0xE0, /**< Piksi  -> Host  */
            MSG_FLASH_READ = 0xE1, /**< Host  <-> Piksi */
            MSG_FLASH_ERASE = 0xE2, /**< Host   -> Piksi */

            MSG_STM_FLASH_LOCK_SECTOR = 0xE3, /**< Host   -> Piksi */
            MSG_STM_FLASH_UNLOCK_SECTOR = 0xE4, /**< Host   -> Piksi */

            MSG_STM_UNIQUE_ID = 0xE5, /**< Host  <-> Piksi */

            MSG_M25_FLASH_WRITE_STATUS = 0xF3, /**< Host   -> Piksi */

            MSG_RESET_FILTERS = 0x22, /**< Host   -> Piksi */
            MSG_INIT_BASE = 0x23, /**< Host   -> Piksi */

            MSG_SETTINGS = 0xA0, /**< Host  <-> Piksi */
            MSG_SETTINGS_SAVE = 0xA1, /**< Host   -> Piksi */
            MSG_SETTINGS_READ_BY_INDEX = 0xA2, /**< Host   -> Piksi */

            MSG_FILEIO_READ = 0xA8, /**< Host  <-> Piksi */
            MSG_FILEIO_READ_DIR = 0xA9, /**< Host  <-> Piksi */
            MSG_FILEIO_REMOVE = 0xAC,  /**< Host   -> Piksi */
            MSG_FILEIO_WRITE = 0xAD,  /**< Host  <-> Piksi */

            MSG_SIMULATION_ENABLED = 0xAA, /**< Host  <-> Piksi */

            MSG_OBS_HDR = 0x40, /**< Piksi  -> Host  */

            MSG_OBS = 0x41, /**< Piksi  -> Host  */
            MSG_OLD_OBS = 0x42, /**< Piksi  -> Host  */
            MSG_PACKED_OBS = 0x43,  /**< Piksi  -> Host  */

            MSG_BASE_POS = 0x44,

            MSG_TRACKING_STATE = 0x16, /**< Piksi  -> Host  */
            MSG_IAR_STATE = 0x19, /**< Piksi  -> Host  */

            MSG_THREAD_STATE = 0x17, /**< Piksi  -> Host  */

            MSG_UART_STATE = 0x18, /**< Piksi  -> Host  */


            MSG_ACQ_RESULT = 0x15, /**< Piksi  -> Host  */

            SBP_STARTUP = 0xFF00,
            SBP_HEARTBEAT = 0xFFFF,
            SBP_GPS_TIME = 0x0100,
            SBP_DOPS = 0x0206,
            SBP_POS_ECEF = 0x0200,
            SBP_POS_LLH = 0x0201,
            SBP_BASELINE_ECEF = 0x0202,
            SBP_BASELINE_NED = 0x0203,
            SBP_VEL_ECEF = 0x0204,
            SBP_VEL_NED = 0x0205,
        }

        public const int MSG_OBS_HEADER_SEQ_SHIFT = 4;
        public const int MSG_OBS_HEADER_SEQ_MASK = ((1 << 4) - 1);
        public uint MSG_OBS_HEADER_MAX_SIZE = MSG_OBS_HEADER_SEQ_MASK;
        public const double MSG_OBS_TOW_MULTIPLIER = ((double)1000.0);

        public const double MSG_OBS_P_MULTIPLIER = ((double)1e2);
        public const float MSG_OBS_SNR_MULTIPLIER = ((float)4);
        public const double MSG_OSB_LF_MULTIPLIER = ((double)(1 << 8));

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_startup_t
        {
            public u32 reserved; /**< Reserved */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_heartbeat_t
        {
            public u32 flags; /**< Status flags */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_gps_time_t
        {
            public u16 wn;    /**< GPS week number [weeks] */
            public u32 tow;   /**< GPS Time of Week rounded to the nearest ms [ms] */
            public s32 ns;    /**< Nanosecond remainder of rounded tow [ns] */
            public u8 flags; /**< Status flags (reserved) */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_dops_t
        {
            public u32 tow;  /**< GPS Time of Week [ms] */
            public u16 gdop; /**< Geometric Dilution of Precision [0.01] */
            public u16 pdop; /**< Position Dilution of Precision [0.01] */
            public u16 tdop; /**< Time Dilution of Precision [0.01] */
            public u16 hdop; /**< Horizontal Dilution of Precision [0.01] */
            public u16 vdop; /**< Vertical Dilution of Precision [0.01] */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_pos_ecef_t
        {
            public u32 tow;      /**< GPS Time of Week [ms] */
            public double x;        /**< ECEF X coordinate [m] */
            public double y;        /**< ECEF Y coordinate [m] */
            public double z;        /**< ECEF Z coordinate [m] */
            public u16 accuracy; /**< Position accuracy estimate [mm] */
            public u8 n_sats;   /**< Number of satellites used in solution */
            public u8 flags;    /**< Status flags */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_pos_llh_t
        {
            public u32 tow;        /**< GPS Time of Week [ms] */
            public double lat;        /**< Latitude [deg] */
            public double lon;        /**< Longitude [deg] */
            public double height;     /**< Height [m] */
            public u16 h_accuracy; /**< Horizontal position accuracy estimate [mm] */
            public u16 v_accuracy; /**< Vertical position accuracy estimate [mm] */
            public u8 n_sats;     /**< Number of satellites used in solution */
            public u8 flags;      /**< Status flags */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_baseline_ecef_t
        {
            public u32 tow;      /**< GPS Time of Week [ms] */
            public s32 x;        /**< Baseline ECEF X coordinate [mm] */
            public s32 y;        /**< Baseline ECEF Y coordinate [mm] */
            public s32 z;        /**< Baseline ECEF Z coordinate [mm] */
            public u16 accuracy; /**< Position accuracy estimate [mm] */
            public u8 n_sats;   /**< Number of satellites used in solution */
            public u8 flags;    /**< Status flags */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_baseline_ned_t
        {
            public u32 tow;        /**< GPS Time of Week [ms] */
            public s32 n;          /**< Baseline North coordinate [mm] */
            public s32 e;          /**< Baseline East coordinate [mm] */
            public s32 d;          /**< Baseline Down coordinate [mm] */
            public u16 h_accuracy; /**< Horizontal position accuracy estimate [mm] */
            public u16 v_accuracy; /**< Vertical position accuracy estimate [mm] */
            public u8 n_sats;     /**< Number of satellites used in solution */
            public u8 flags;      /**< Status flags */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_vel_ecef_t
        {
            public u32 tow;      /**< GPS Time of Week [ms] */
            public s32 x;        /**< Velocity ECEF X coordinate [mm/s] */
            public s32 y;        /**< Velocity ECEF Y coordinate [mm/s] */
            public s32 z;        /**< Velocity ECEF Z coordinate [mm/s] */
            public u16 accuracy; /**< Velocity accuracy estimate [mm/s] */
            public u8 n_sats;   /**< Number of satellites used in solution */
            public u8 flags;    /**< Status flags (reserved) */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct sbp_vel_ned_t
        {
            public u32 tow;        /**< GPS Time of Week [ms] */
            public s32 n;          /**< Velocity North coordinate [mm/s] */
            public s32 e;          /**< Velocity East coordinate [mm/s] */
            public s32 d;          /**< Velocity Down coordinate [mm/s] */
            public u16 h_accuracy; /**< Horizontal velocity accuracy estimate [mm/s] */
            public u16 v_accuracy; /**< Vertical velocity accuracy estimate [mm/s] */
            public u8 n_sats;     /**< Number of satellites used in solution */
            public u8 flags;      /**< Status flags (reserved) */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct acq_result_msg_t
        {
            public float snr; /* SNR of best point. */
            public float cp;  /* Code phase of best point. */
            public float cf;  /* Carr freq of best point. */
            public u8 prn;    /* PRN searched for. */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct msg_uart_state_t
        {
            public uarts uart1;
            public uarts uart2;
            public uarts uart3;
            public latency_t obs_latency;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct uarts
        {
            public float tx_throughput;
            public float rx_throughput;
            public u16 crc_error_count;
            public u8 tx_buffer_level;
            public u8 rx_buffer_level;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct latency_t
        {
            public s32 avg;
            public s32 min;
            public s32 max;
            public s32 current;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct msg_thread_state_t
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public char[] name; //[20]
            public u16 cpu;
            public u32 stack_free;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct msg_iar_state_t
        {
            public u32 num_hyps;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct msg_base_pos_t
        {
            public double pos_lat;
            public double pos_lon;
            public double pos_alt;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct msg_obs_content_t
        {
            public u32 P;     /**< Pseudorange (cm) */
            [StructLayout(LayoutKind.Sequential, Pack = 1)]
            public struct Ls
            {
                public s32 Li;  /**< Carrier phase (integer seconds) */
                public u8 Lf;   /**< Carrier phase (scaled fractional seconds) */
            }        /**< Fixed point carrier phase (seconds) */
            public Ls L;
            public u8 snr;    /**< Signal-to-Noise ratio (cn0 * 4 for 0.25 precision and
                  0-64 range) */
            public u8 prn;    /**< Satellite number. */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct msg_obs_header_t
        {
            [StructLayout(LayoutKind.Sequential, Pack = 1)]
            public struct ts
            {
                public u32 tow;  /**< milliseconds since start of week. */
                public u16 wn;   /**< GPS Week Numer. */
            }       /**< Compcated millisecond-accurate GPS time. */
            public ts t;
            public u8 seq;     /**< First nibble is the size of the sequence (n), second
                   nibble is the zero-indexed counter (ith packet of n) */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct msg_obs_hdr_t
        {
            public gps_time_t t; /**< GPS time of observation. */
            public u8 count;     /**< Serial count of obervation. */
            public u8 n_obs;     /**< Number of observation records to follow. */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct tracking_state_msg_t
        {
            public u8 state;  /**< State of the tracking channel. */
            public u8 prn;    /**< PRN being tracked by the tracking channel. */
            public float cn0; /**< SNR of the tracking channel. */
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct header
        {
            public byte preamble; // 0x55
            public UInt16 msgtype;
            public UInt16 sender;
            public byte length; // payload length
            [MarshalAs(UnmanagedType.ByValArray)]
            public byte[] payload;
            public UInt16 crc; // - preamble
        }

        /** Value specifying the size of the SBP framing */
        public const int SBP_FRAMING_SIZE_BYTES = 8;
        /** Value defining maximum SBP packet size */
        public const int SBP_FRAMING_MAX_PAYLOAD_SIZE = 255;

        public byte[] GeneratePacket(object indata, MSG msgtype)
        {
            byte[] data = StaticUtils.StructureToByteArray(indata);

            byte[] packet = new u8[data.Length + 6 + 2];

            piksi.header msgpreamble = new piksi.header();
            msgpreamble.crc = 0x1234;
            msgpreamble.preamble = 0x55;
            msgpreamble.msgtype = (ushort)msgtype;
            msgpreamble.sender = 1;
            msgpreamble.length = (byte)(data.Length);
            msgpreamble.payload = new byte[msgpreamble.length];

            byte[] preamblebytes = StaticUtils.StructureToByteArray(msgpreamble);

            Array.Copy(preamblebytes, 0, packet, 0, preamblebytes.Length - 2);

            Array.Copy(data, 0, packet, 6, data.Length);


            Crc16Ccitt crc = new Crc16Ccitt(InitialCrcValue.Zeros);
            ushort crcpacket = 0;
            for (int i = 1; i < (packet.Length - 2); i++)
            {
                crcpacket = crc.Accumulate(packet[i], crcpacket);
            }

            packet[packet.Length - 2] = (byte)(crcpacket & 0xff);
            packet[packet.Length - 1] = (byte)((crcpacket >> 8) & 0xff);

            return packet;
        }

        // packet read step state
        int state = 0;

        header msg = new header();

        int length = 0;

        Crc16Ccitt crc;

        ushort crcpacket = 0;

        public void read(byte data)
        {
            switch (state)
            {
                case 0:
                    if (data == 0x55)
                    {
                        state++;
                        msg = new header();
                        msg.preamble = data;
                        crc = new Crc16Ccitt(InitialCrcValue.Zeros);
                        crcpacket = (ushort)InitialCrcValue.Zeros;
                    }
                    else
                    {

                    }
                    break;
                case 1:
                    msg.msgtype = (u16)(data);
                    crcpacket = crc.Accumulate(data, crcpacket);
                    state++;
                    break;
                case 2:
                    msg.msgtype = (u16)(msg.msgtype + (data << 8));
                    crcpacket = crc.Accumulate(data, crcpacket);
                    state++;
                    break;
                case 3:
                    msg.sender = (u16)(data);
                    crcpacket = crc.Accumulate(data, crcpacket);
                    state++;
                    break;
                case 4:
                    msg.sender = (u16)(msg.sender + (data << 8));
                    crcpacket = crc.Accumulate(data, crcpacket);
                    state++;
                    break;
                case 5:
                    msg.length = data;
                    crcpacket = crc.Accumulate(data, crcpacket);
                    msg.payload = new u8[msg.length];
                    length = 0;
                    state++;
                    break;
                case 6:
                    if (length == msg.length)
                    {
                        state++;
                        goto case 7;
                    }
                    else
                    {
                        msg.payload[length] = data;
                        crcpacket = crc.Accumulate(data, crcpacket);
                        length++;
                    }
                    break;
                case 7:
                    msg.crc = (u16)(data);
                    state++;
                    break;
                case 8:
                    msg.crc = (u16)(msg.crc + (data << 8));
                    state = 0;

                    if (msg.crc == crcpacket)
                    {

                        if ((MSG)msg.msgtype == MSG.SBP_GPS_TIME)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_gps_time_t>(0);

                            Console.SetCursorPosition(0, 0);
                            Console.WriteLine(test.wn + " " + test.tow);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_POS_LLH)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_pos_llh_t>(0);

                            //Console.WriteLine(test.lat + " " + test.lon + " " + test.height);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_DOPS)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_dops_t>(0);

                            //Console.WriteLine(test.gdop + " " + test.hdop + " " + test.tow);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_POS_ECEF)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_pos_ecef_t>(0);

                             //Console.WriteLine(test.x + " " + test.y + " " + test.z);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_VEL_NED)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_vel_ned_t>(0);

                            //Console.WriteLine(test.n + " " + test.e + " " + test.d);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_VEL_ECEF)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_vel_ecef_t>(0);

                            //Console.WriteLine(test.x + " " + test.y + " " + test.z);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_BASELINE_NED)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_baseline_ned_t>(0);

                            //Console.WriteLine(test.n + " " + test.e + " " + test.d);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_BASELINE_ECEF)
                        {
                            var test = msg.payload.ByteArrayToStructure<sbp_baseline_ecef_t>(0);

                             //Console.WriteLine(test.x + " " + test.y + " " + test.z);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_PACKED_OBS)
                        {

                            var hdr = msg.payload.ByteArrayToStructure<msg_obs_header_t>(0);

                            // total is number of packets
                            int total = hdr.seq >> MSG_OBS_HEADER_SEQ_SHIFT;
                            // this is packet count number
                            int count = hdr.seq & MSG_OBS_HEADER_SEQ_MASK;

                            int lenhdr = Marshal.SizeOf(hdr);

                            int lenobs = Marshal.SizeOf(new msg_obs_content_t());

                            int obscount = (msg.length - lenhdr) / lenobs;

                            int linebase = (count > 0) ? 8 : 0;

                            for (int a = 0; a < obscount; a++)
                            {
                                var ob = msg.payload.ByteArrayToStructure<msg_obs_content_t>(lenhdr + a * lenobs);

                                Console.SetCursorPosition(0, 15 + a + linebase);

                                Console.WriteLine(msg.sender + " " + hdr.t.tow + " " + ob.prn + "\t" + (ob.snr / MSG_OBS_SNR_MULTIPLIER) + "\t" + (ob.P / MSG_OBS_P_MULTIPLIER) + "\t" + (ob.L.Li + (ob.L.Lf / 256.0)) + "    ");
                            }

                            if (ObsMessage != null)
                                ObsMessage(msg, null);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_BASE_POS)
                        {
                            var bpos = msg.payload.ByteArrayToStructure<msg_base_pos_t>(0);

                            Console.WriteLine("base pos {0} {1} {2}",bpos.pos_lat,bpos.pos_lon,bpos.pos_alt);

                            if (BasePosMessage != null)
                                BasePosMessage(msg, null);                            
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_IAR_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_iar_state_t>(0);

                            //Console.WriteLine(test.num_hyps);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_PRINT)
                        {
                            Console.SetCursorPosition(0,14);
                            Console.Write(ASCIIEncoding.ASCII.GetString(msg.payload));
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_TRACKING_STATE)
                        {
                            int len = Marshal.SizeOf(new tracking_state_msg_t());

                            for (int a = 0; a < msg.length; a += len)
                            {
                                var test = msg.payload.ByteArrayToStructure<tracking_state_msg_t>(a);

                                Console.SetCursorPosition(60, a / len);

                                Console.WriteLine(test.prn + "\t" + test.state + " " + test.cn0 + "         ");
                            }

                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_UART_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_uart_state_t>(0);

                            Console.SetCursorPosition(0, 13);
                            Console.WriteLine("uart3 " + test.uart3.tx_throughput + " uart2 " + test.uart2.tx_throughput + " obs lat " + test.obs_latency.current + "     ");
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_THREAD_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_thread_state_t>(0);
                            //Console.WriteLine(new String(test.name) + " cpu " + test.cpu / 10.0 + "\tstackfree " + test.stack_free + "   ");
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_HEARTBEAT)
                        {
                            //Console.Clear();
                            Console.SetCursorPosition(0, 0);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_ACQ_RESULT)
                        {
                            var test = msg.payload.ByteArrayToStructure<acq_result_msg_t>(0);
                            Console.SetCursorPosition(0, 17);
                            Console.WriteLine("aqn\t" + test.prn + "\t" + test.snr.ToString("0.00") + "\t" + test.cp + "\t" + test.cf + "\t\t");
                        }
                        else
                        {
                            Console.WriteLine((MSG)msg.msgtype + " " + msg.length + " " + msg.sender);
                        }
                    }
                    else
                    {
                        Console.WriteLine("sbp crc fail");
                    }
                    break;
            }
        }
    }
}