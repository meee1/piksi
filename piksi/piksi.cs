﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using u8 = System.Byte;
using u16 = System.UInt16;
using s32 = System.Int32;
using u32 = System.UInt32;
using s64 = System.Int64;
using System.Runtime.InteropServices;
using System.IO;

namespace piksi
{
    public class piksi
    {
        public event EventHandler ObsMessage;
        public event EventHandler BasePosMessage;
        public event EventHandler EphMessage;

        header msgobs = new header();

        values prtest = new values();
        values cptest = new values();
        values doptest = new values();
        values satdisttest = new values();

        prsmooth prsmoothdata = new prsmooth();

        ephemeris_t[] eph = new ephemeris_t[33];

        public bool consoleoutput = true;

        int printline = 60;

        /** Approximate average distance to the GPS satellites in m. */
        public const double GPS_NOMINAL_RANGE = 22.980e6;

/** GPS C/A code chipping rate in Hz. */
const double GPS_CA_CHIPPING_RATE =1.023e6;

        /** The official GPS value of the speed of light in m / s. 
 * \note This is the exact value of the speed of light in vacuum (by the definition of meters). */
const double GPS_C =299792458.0;

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
            MSG_PACKED_OBS = 0x45,  /**< Piksi  -> Host  */

            MSG_BASE_POS = 0x44,

            MSG_TRACKING_STATE = 0x16, /**< Piksi  -> Host  */
            MSG_IAR_STATE = 0x19, /**< Piksi  -> Host  */

            MSG_THREAD_STATE = 0x17, /**< Piksi  -> Host  */

            MSG_UART_STATE = 0x18, /**< Piksi  -> Host  */

            MSG_EPHEMERIS = 0x46,  /**< Piksi  -> Host  */

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


        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //----------------------------------------------------------
        //[StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct channel_measurement_t
        {
            public u8 prn;
            public double code_phase_chips;
            public double code_phase_rate;
            public double carrier_phase;
            public double carrier_freq;
            public u32 time_of_week_ms;
            public double receiver_time;
            public double snr;
            public u16 lock_counter;
        }

        //[StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct navigation_measurement_t
        {
            public double raw_pseudorange;
            public double pseudorange;
            public double carrier_phase;
            public double raw_doppler;
            public double doppler;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] sat_pos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] sat_vel;
            public double snr;
            public double lock_time;
            public gps_time_t tot;
            public u8 prn;
            public u16 lock_counter;
        }

        // from pvt.h
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct gnss_solution
        {
            /*
   * Be careful of stuct packing to avoid (very mild) slowness,
   * try to keep all the types aligned i.e. put the 64bit
   * things together at the top, then the 32bit ones etc.
   */
            /** Receiver position latitude [deg], longitude [deg], altitude [m] */
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public double[] pos_llh;
            /** Receiver position ECEF XYZ [m] */
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] 
            public double[] pos_ecef;
            /** Receiver velocity in NED [m/s] */
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] 
            public double[] vel_ned;
            /** Receiver velocity in ECEF XYZ [m/s] */
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] 
            public double[] vel_ecef;

            /* This is the row-first upper diagonal matrix of error covariances
   * in x, y, z (all receiver clock covariance terms are ignored).  So
   * it goes like so:
   *
   *    0  1  2
   *    _  3  4
   *    _  _  5
   *
   *    Index 6 is the GDOP.
   */
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public double[] err_cov;

            public double clock_offset;
            public double clock_bias;

            /* GPS time */
            public gps_time_t time;

            /* 0 = invalid, 1 = code phase */
            public u8 valid;
            /* Number of channels used in the soluton. */
            public u8 n_used;
        }

        navigation_measurement_t[] meas_last = new navigation_measurement_t[33];

         [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct gps_time_t
        {
            public double tow; /**< Seconds since the GPS start of week. */
            public u16 wn;     /**< GPS week number. */
}

         [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct ephemeris_t
        {
            public double tgd;
            public double crs, crc, cuc, cus, cic, cis;
            public double dn, m0, ecc, sqrta, omega0, omegadot, w, inc, inc_dot;
            public double af0, af1, af2;
            public gps_time_t toe, toc;
            public u8 valid;
            public u8 healthy;
            public u8 prn;

            /** Time difference in seconds between two GPS times.
 * \param end Higher bound of the time interval whose length is calculated.
 * \param beginning Lower bound of the time interval whose length is
 *                  calculated. If this describes a time point later than end,
 *                  the result is negative.
 * \return The time difference in seconds between `beginning` and `end`.
 */

            private double gpsdifftime(gps_time_t end, gps_time_t beginning)
            {
                return (end.wn - beginning.wn)*7*24*3600 +
                       end.tow - beginning.tow;
            }

            public int calc_sat_pos(double[] pos, double[] vel,
                ref double clock_err, ref double clock_rate_err,
                gps_time_t tot)
            {
                /****************************************************************************
   * Calculate satellite position, velocity and clock offset from ephemeris
   * Taken from IS-GPS-200D, Section 20.3.3.3.3.1 and Table 20-IV
   *
   * Usage: unsigned int ProcessEphemeris(unsigned int week, double secs,
   *                              unsigned int sv, nav_info_t *Eph)
   *  week: GPS week number
   *  sec: GPS seconds of the week of time of transmission
   *  sv: Satellite vehicle number
   *    Eph: Ephemeris data structure located within channel structure
   ****************************************************************************/

                double tempd1 = 0, tempd2, tempd3;
                double tdiff;
                double a; // semi major axis
                double ma, ma_dot; // mean anomoly and first derivative (mean motion)
                double ea, ea_dot, ea_old; // eccentric anomoly, first deriv, iteration var
                double einstein; // relativistic correction
                double al, al_dot; // argument of lattitude and first derivative
                double cal, cal_dot; // corrected argument of lattitude and first deriv
                double r, r_dot; // radius and first derivative
                double inc, inc_dot; // inclination and first derivative
                double x, x_dot, y, y_dot; // position in orbital plan and first derivatives
                double om, om_dot; // omega and first derivatives

                const double NAV_OMEGAE_DOT = 7.2921151467e-005;
                const double NAV_GM = 3.986005e14;

                // Satellite clock terms
                // Seconds from clock data reference time (toc)
                tdiff = gpsdifftime(tot, this.toc);

                clock_err = this.af0 + tdiff*(this.af1 + tdiff*this.af2) - this.tgd;
                clock_rate_err = this.af1 + 2.0*tdiff*this.af2;

                // Seconds from the time from ephemeris reference epoch (toe)
                tdiff = gpsdifftime(tot, this.toe);

                // If tdiff is too large our ephemeris isn't valid, maybe we want to wait until we get a
                // new one? At least let's warn the user.
                // TODO: this doesn't exclude ephemerides older than a week so could be made better.
                //if (Math.Abs(tdiff) > 4*3600)
                //    Console.Write(" WARNING: uSing ephemeris older (or newer!) than 4 hours.\n");

                // Calculate position per IS-GPS-200D p 97 Table 20-IV
                a = this.sqrta*this.sqrta; // [m] Semi-major axis
                ma_dot = Math.Sqrt(NAV_GM / (a * a * a)) + this.dn; // [rad/sec] Corrected mean motion
                ma = this.m0 + ma_dot*tdiff; // [rad] Corrected mean anomaly

                // Iteratively solve for the Eccentric Anomaly (from Keith Alter and David Johnston)
                ea = ma; // Starting value for E
                double ecc = this.ecc;
                u32 count = 0;

                /* TODO: Implement convergence test uSing integer difference of doubles,
   * http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm */
                do
                {
                    ea_old = ea;
                    tempd1 = 1.0 - ecc * Math.Cos(ea_old);
                    ea = ea + (ma - ea_old + ecc * Math.Sin(ea_old)) / tempd1;
                    count++;
                    if (count > 5)
                        break;
                } while (Math.Abs(ea - ea_old) > 1.0E-14);
                ea_dot = ma_dot/tempd1;

                // Relativistic correction term
                einstein = -4.442807633E-10 * ecc * this.sqrta * Math.Sin(ea);

                // Begin calc for True Anomaly and Argument of Latitude
                tempd2 = Math.Sqrt(1.0 - ecc * ecc);
                al = Math.Atan2(tempd2 * Math.Sin(ea), Math.Cos(ea) - ecc) + this.w;
                    // [rad] Argument of Latitude = True Anomaly + Argument of Perigee
                al_dot = tempd2*ea_dot/tempd1;

                // Calculate corrected argument of latitude based on position
                cal = al + this.cus * Math.Sin(2.0 * al) + this.cuc * Math.Cos(2.0 * al);
                cal_dot =
                    al_dot*(1.0 +
                            2.0 * (this.cus * Math.Cos(2.0 * al) -
                                 this.cuc * Math.Sin(2.0 * al)));

                // Calculate corrected radius based on argument of latitude
                r =
                    a * tempd1 + this.crc * Math.Cos(2.0 * al) +
                    this.crs * Math.Sin(2.0 * al);
                r_dot =
                    a * ecc * Math.Sin(ea) * ea_dot +
                    2.0 * al_dot * (this.crs * Math.Cos(2.0 * al) -
                                this.crc * Math.Sin(2.0 * al));

                // Calculate inclination based on argument of latitude
                inc =
                    this.inc + this.inc_dot*tdiff +
                    this.cic * Math.Cos(2.0 * al) + this.cis * Math.Sin(2.0 * al);
                inc_dot =
                    this.inc_dot + 2.0 * al_dot * (this.cis * Math.Cos(2.0 * al) -
                                               this.cic * Math.Sin(2.0 * al));

                // Calculate position and velocity in orbital plane
                x = r * Math.Cos(cal);
                y = r * Math.Sin(cal);
                x_dot = r_dot * Math.Cos(cal) - y * cal_dot;
                y_dot = r_dot * Math.Sin(cal) + x * cal_dot;

                // Corrected longitude of ascenting node
                om_dot = this.omegadot - NAV_OMEGAE_DOT;
                om = this.omega0 + tdiff*om_dot - NAV_OMEGAE_DOT*this.toe.tow;

                // Compute the satellite's position in Earth-Centered Earth-Fixed coordiates
                pos[0] = x * Math.Cos(om) - y * Math.Cos(inc) * Math.Sin(om);
                pos[1] = x * Math.Sin(om) + y * Math.Cos(inc) * Math.Cos(om);
                pos[2] = y * Math.Sin(inc);

                tempd3 = y_dot * Math.Cos(inc) - y * Math.Sin(inc) * inc_dot;

                // Compute the satellite's velocity in Earth-Centered Earth-Fixed coordiates
                vel[0] = -om_dot * pos[1] + x_dot * Math.Cos(om) - tempd3 * Math.Sin(om);
                vel[1] = om_dot * pos[0] + x_dot * Math.Sin(om) + tempd3 * Math.Cos(om);
                vel[2] = y * Math.Cos(inc) * inc_dot + y_dot * Math.Sin(inc);

                clock_err += einstein;

                return 0;
            }
        }

        double nav_tc = 0;

        //----------------------------------------------------------



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

                public double GetValue()
                {
                    return (double)(Li + (Lf / 256.0));
                }

                public void SetValue(double value)
                {
                    Li = (int)value;
                    Lf = (byte)((value - Li) * 256.0);
                }
            }        /**< Fixed point carrier phase (seconds) */
            public Ls L;
            public u8 snr;    /**< Signal-to-Noise ratio (cn0 * 4 for 0.25 precision and
                  0-64 range) */
            public u16 lock_counter; /**< Lock counter. Increments on new lock. */
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

        public void GetSettings()
        {
            for (int a = 0; a < 100; a++)
            {
                GeneratePacket(a, MSG.MSG_SETTINGS_READ_BY_INDEX);
            }
        }

        public void SendAllUartA()
        {
            // section, setting, value, formattype
              GeneratePacket("uart_uarta\0sbp_message_mask\065535\0", MSG.MSG_SETTINGS);
        }

        public byte[] GeneratePacket(object indata, MSG msgtype)
        {
            byte[] data = new byte[0];

            if (indata == null) {

            } 
            else if (indata is string)
            {
                data = ASCIIEncoding.ASCII.GetBytes((string)indata);
            }
            else if (indata.GetType().IsValueType)
            {
                data = StaticUtils.StructureToByteArray(indata);
            }

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

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 0);
                                Console.WriteLine(test.wn + " " + test.tow);
                            }
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
                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 1);
                                Console.WriteLine("bl " + test.x + " " + test.y + " " + test.z);
                            }
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_PACKED_OBS)
                        {

                            var hdr = msg.payload.ByteArrayToStructure<msg_obs_header_t>(0);

                            // total is number of packets
                            int total = hdr.seq >> MSG_OBS_HEADER_SEQ_SHIFT;
                            // this is packet count number
                            int count = hdr.seq & MSG_OBS_HEADER_SEQ_MASK;

                            if (count == 0)
                            {
                                msgobs = msg;
                            }

                            int lenhdr = Marshal.SizeOf(hdr);

                            int lenobs = Marshal.SizeOf(new msg_obs_content_t());

                            int obscount = (msg.length - lenhdr) / lenobs;

                            int linebase = (count > 0) ? 7 : 0;

                            //todo add tow check
                            if (count > 0 && msgobs.payload != null)
                            {
                                // resize msgobs payload to include current msg obs
                                int currentpayloadend = msgobs.payload.Length;
                                Array.Resize<byte>(ref msgobs.payload, msgobs.payload.Length + obscount * lenobs);
                                Array.Copy(msg.payload, lenhdr, msgobs.payload, currentpayloadend, obscount * lenobs);
                                msgobs.length += (byte)(obscount * lenobs);
                            }

                            calcPos(hdr, obscount);

                            for (int a = 0; a < obscount; a++)
                            {
                                var ob = msg.payload.ByteArrayToStructure<msg_obs_content_t>(lenhdr + a * lenobs);

                                if (consoleoutput)
                                {
                                    Console.SetCursorPosition(0, 15 + a + linebase);

                                    Console.WriteLine("{0,6} {1,10} {2,2} {3,5} {4,11} {5,17} {6,17}           ",
                                        msg.sender, hdr.t.tow, (ob.prn + 1),
                                        (ob.snr/MSG_OBS_SNR_MULTIPLIER).ToString("0"),
                                        (ob.P/MSG_OBS_P_MULTIPLIER).ToString("0.00"),
                                        (ob.L.Li + (ob.L.Lf/256.0)).ToString("0.000000"), ob.lock_counter);
                                }
                            }

                            if (count == (total - 1) && msgobs.payload != null)
                            {
                                if (ObsMessage != null)
                                    ObsMessage(msgobs, null);
                            }
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_BASE_POS)
                        {
                            var bpos = msg.payload.ByteArrayToStructure<msg_base_pos_t>(0);

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 2);
                                Console.WriteLine("base pos {0} {1} {2}", bpos.pos_lat, bpos.pos_lon, bpos.pos_alt);
                            }

                            if (BasePosMessage != null)
                                BasePosMessage(msg, null);                            
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_IAR_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_iar_state_t>(0);

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 3);
                                Console.WriteLine("IAR " + test.num_hyps);
                            }
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_PRINT)
                        {
                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, printline);
                                Console.Write(printline + " " + ASCIIEncoding.ASCII.GetString(msg.payload));
                            }

                            printline++;

                            if (printline > 68)
                                printline = 58;
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_TRACKING_STATE)
                        {
                            int len = Marshal.SizeOf(new tracking_state_msg_t());

                            for (int a = 0; a < msg.length; a += len)
                            {
                                var test = msg.payload.ByteArrayToStructure<tracking_state_msg_t>(a);

                                if (consoleoutput)
                                {
                                    Console.SetCursorPosition(65, a/len);
                                    Console.WriteLine("{0,2} {1,1} {2,10}", test.prn + 1, test.state, test.cn0);
                                }
                            }

                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_UART_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_uart_state_t>(0);

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 13);
                                Console.WriteLine("uart3 " + test.uart3.tx_throughput + " uart2 " +
                                                  test.uart2.tx_throughput + " obs lat " + test.obs_latency.current +
                                                  "     ");
                            }
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_THREAD_STATE)
                        {
                            var test = msg.payload.ByteArrayToStructure<msg_thread_state_t>(0);
                            //Console.WriteLine(new String(test.name) + " cpu " + test.cpu / 10.0 + "\tstackfree " + test.stack_free + "   ");
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_HEARTBEAT)
                        {
                            if (consoleoutput)
                            {
                                Console.WriteLine("HB");
                            }
                            //Console.Clear();
                            //Console.SetCursorPosition(0, 0);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_ACQ_RESULT)
                        {
                            var test = msg.payload.ByteArrayToStructure<acq_result_msg_t>(0);
                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 7);
                                Console.WriteLine("aqn\t" + (test.prn + 1) + "\t" + test.snr.ToString("0.00") + "\t" +
                                                  test.cp + "\t" + test.cf + "\t\t");
                            }
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_SETTINGS_READ_BY_INDEX)
                        {
                            string test = ASCIIEncoding.ASCII.GetString(msg.payload);

                            string[] items = test.Split('\0');

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 4);
                                Console.WriteLine("setting " + test);
                            }

                            //var test = msg.payload.ByteArrayToStructure<>(0);
                            }
                        else if ((MSG)msg.msgtype == MSG.MSG_BOOTLOADER_HANDSHAKE)
                        {
                            string test = ASCIIEncoding.ASCII.GetString(msg.payload);
                            //var test = msg.payload.ByteArrayToStructure<>(0);
                        }
                        else if ((MSG)msg.msgtype == MSG.SBP_STARTUP)
                        {
                            //var test = msg.payload.ByteArrayToStructure<>(0);
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_DEBUG_VAR)
                        {
                           // Console.Clear();
                            var value = BitConverter.ToDouble(msg.payload, 0);
                            string debug = ASCIIEncoding.ASCII.GetString(msg.payload,8,msg.payload.Length - 8);
                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 59);
                                Console.WriteLine(debug + " " + (value) + "    ");
                            }

                            nav_tc = (value);
                        }
                        else if (msg.msgtype == 0x207)
                        {
                            int lenitem = Marshal.SizeOf(new channel_measurement_t());

                            var meas = msg.payload.ByteArrayToStructure<channel_measurement_t>(0);
                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 26 + meas.prn);
                            }

                            var nav_meas = new navigation_measurement_t();

                            nav_meas.prn = meas.prn;

                            double nav_time = nav_tc;

                            //rx time rolls over at 262
                            //Each chip is about 977.5 ns
                            //The total code period contains 1,023 chips.
                            //With a chip rate of 1.023 MHz, 1,023 chips last 1 ms; therefore, the C/ A code
                            //is 1 ms long. This code repeats itself every millisecond.
                            //http://www.insidegnss.com/node/2898

                            double test = meas.code_phase_chips / GPS_CA_CHIPPING_RATE;

                            nav_meas.tot.tow = meas.time_of_week_ms * 1e-3;
                            nav_meas.tot.tow += meas.code_phase_chips / GPS_CA_CHIPPING_RATE;
                            nav_meas.tot.tow += (nav_time - meas.receiver_time) * (meas.code_phase_rate / GPS_CA_CHIPPING_RATE);

                            //var clock_err = eph[meas.prn + 1].clock_err(nav_meas.tot.tow);

                            nav_meas.carrier_phase = meas.carrier_phase;
                            nav_meas.carrier_phase += (nav_time - meas.receiver_time) * meas.carrier_freq;

                            nav_meas.raw_doppler = meas.carrier_freq;

                            nav_meas.lock_counter = meas.lock_counter;

                            nav_meas.raw_pseudorange = (Math.Round(nav_meas.tot.tow) - nav_meas.tot.tow) * GPS_C;// +GPS_NOMINAL_RANGE;

                            int satno = meas.prn + 1;


                            prtest.Add(satno, nav_meas.raw_pseudorange);
                            cptest.Add(satno, nav_meas.carrier_phase);
                            doptest.Add(satno, nav_meas.raw_doppler);

                            var file = File.Open(satno + "-chmeas.csv", FileMode.Append);

                            string datas = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},extra,{9},{10},{11},{12}\n", meas.prn, meas.code_phase_chips, meas.code_phase_rate, meas.carrier_phase, meas.carrier_freq, meas.time_of_week_ms, meas.receiver_time, meas.snr, meas.lock_counter,
                                nav_tc, nav_meas.tot.tow, nav_meas.carrier_phase, nav_meas.raw_pseudorange);

                            file.Write(ASCIIEncoding.ASCII.GetBytes(datas), 0, datas.Length);

                            file.Close();

                            if (consoleoutput)
                            {
                                //Console.WriteLine("{0,2} {1} {2}", satno, nav_meas.raw_doppler, meas.carrier_phase);
                                Console.WriteLine("{0,2} {1,17} {2,17} {3,17} {4,17} {5,17}", meas.prn + 1,
                                    nav_meas.tot.tow.ToString("0.000"), meas.code_phase_chips,
                                    meas.code_phase_rate/1000.0, nav_meas.carrier_phase, nav_meas.raw_pseudorange);
                            }

                            meas_last[nav_meas.prn] = nav_meas;
                        }
                        else if (msg.msgtype == 0x208)
                        {
                            int lenitem = Marshal.SizeOf(new navigation_measurement_t());

                            var test = msg.payload.ByteArrayToStructure<navigation_measurement_t>(0);

                            int satno = test.prn + 1;

                            double lam1 = 299792458.0 / 1.57542E9;

                            double[] mypos = new double[] { -2444182.6,4625619.0,-3636118.1 };

                            double satdist = Math.Sqrt(Math.Pow(test.sat_pos[0] - mypos[0], 2) + Math.Pow(test.sat_pos[1] - mypos[1], 2) + Math.Pow(test.sat_pos[2] - mypos[2], 2));

                            prtest.Add(satno, test.raw_pseudorange);
                            cptest.Add(satno, test.carrier_phase);
                            doptest.Add(satno, test.raw_doppler);
                            satdisttest.Add(satno,satdist);

                            double smoothed = prsmoothdata.Add(satno, test.raw_pseudorange, test.carrier_phase * -lam1);

                            if (consoleoutput)
                            {
                                Console.SetCursorPosition(0, 26 + test.prn);
                                Console.WriteLine(
                                    "{0,2} rpr {1,16} tot {2,16} lock {3,2} dop {4,10} cpd {5,10} prd {6} satd {7}   ",
                                    test.prn + 1, test.raw_pseudorange, test.tot.tow, test.lock_time,
                                    (test.doppler*lam1).ToString("0.000"),
                                    (cptest.linearRegression(satno)*-lam1).ToString("0.000"),
                                    prtest.linearRegression(satno).ToString("0.000"),
                                    satdisttest.linearRegression(satno).ToString("0.000"));
                            }

                            var file = File.Open(satno + "-obs.csv", FileMode.Append);

                            string datas = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},extra,{16}\n", test.raw_pseudorange, test.pseudorange, test.carrier_phase, test.raw_doppler, test.doppler, test.sat_pos[0], test.sat_pos[1], test.sat_pos[2], test.sat_vel[0], test.sat_vel[1], test.sat_vel[2], test.snr, test.lock_time, test.tot.tow, test.prn, test.lock_counter, 
                                satdist);

                            file.Write(ASCIIEncoding.ASCII.GetBytes(datas), 0, datas.Length);

                            file.Close();
                        }
                        else if ((MSG)msg.msgtype == MSG.MSG_EPHEMERIS)
                        {
                            int lenitem = Marshal.SizeOf(new ephemeris_t());

                            var test = msg.payload.ByteArrayToStructure<ephemeris_t>(0);

                            eph[test.prn+1] = test;

                            File.WriteAllBytes((test.prn + 1) + ".eph", msg.payload);

                            if (EphMessage != null)
                                EphMessage(msg, null);
                        }
                        else
                        {
                            Console.SetCursorPosition(0, 5);
                            Console.WriteLine("UNK: " + (MSG)msg.msgtype + " " + msg.length + " " + msg.sender);
                        }
                    }
                    else
                    {
                        Console.SetCursorPosition(0, 6);
                        Console.WriteLine(DateTime.Now + " sbp crc fail");
                    }
                    break;
            }
        }

        public struct obinfo
        {
            public msg_obs_content_t rawob;
            public double[] sat_pos;
            public double[] sat_vel;
            public double clock_err;
            public double clock_rate_err;
        }

        double[] lastpos = new double[4];

        private void calcPos(msg_obs_header_t hdr, int obscount)
        {
            const double CLIGHT = 299792458.0;   /* speed of light (m/s) */

            int lenhdr = Marshal.SizeOf(hdr);

            int lenobs = Marshal.SizeOf(new msg_obs_content_t());

            List<obinfo> obs = new List<obinfo>();

            for (int a = 0; a < obscount; a++)
            {
                var ob = msg.payload.ByteArrayToStructure<msg_obs_content_t>(lenhdr + a*lenobs);

                gps_time_t tt = new gps_time_t() { tow = hdr.t.tow / 1000.0, wn = hdr.t.wn };

                double[] pos = new double[3];
                double[] vel = new double[3];
                double clock_err = 0, clock_rate_err = 0;

                eph[ob.prn + 1].calc_sat_pos(pos, vel, ref clock_err, ref clock_rate_err, tt);

                if (double.IsNaN(clock_err))
                    continue;

                obs.Add(new obinfo(){ clock_err = clock_err,clock_rate_err = clock_rate_err,rawob = ob,sat_pos = pos, sat_vel = vel });
            }

            if (obs.Count == 0)
                return;

            double[] x = lastpos;

            double epsg = 1e-4;
            double epsf = 0;
            double epsx = 0;
            int maxits = 0;
            alglib.minlmstate state;
            alglib.minlmreport rep;

            alglib.minlmcreatev(obs.Count, x, 1, out state);
            alglib.minlmsetcond(state, epsg, epsf, epsx, maxits);
            alglib.minlmoptimize(state, sphere_errorpos, null, obs);
            alglib.minlmresults(state, out x, out rep);

            //log.InfoFormat("{0}", rep.terminationtype);
            // log.InfoFormat("{0}", alglib.ap.format(x, 2));

            Console.SetCursorPosition(0, 40);
            Console.WriteLine("lsq {0} {1} {2} {3} {4} {5} {6} {7}", x[0], x[1], x[2], x[3], rep.terminationtype, rep.iterationscount, 0, x[3] / CLIGHT);

            lastpos = x;
        }

        private static void sphere_errorpos(double[] xi, double[] fi, object obj)
        {
            List<obinfo> obs = obj as List<obinfo>;
            double[] myposi = new double[3];
            myposi[0] = xi[0];
            myposi[1] = xi[1];
            myposi[2] = xi[2];
            double clockbias = xi[3];

            int a = 0;
            foreach (var ob in obs)
            {
                double geodist = Math.Sqrt(Math.Pow((ob.sat_pos[0] - myposi[0]), 2) + Math.Pow((ob.sat_pos[1] - myposi[1]), 2) + Math.Pow((ob.sat_pos[2] - myposi[2]), 2));

                //(dist + rclockbias - CLIGHT * raw.dts[sat] + ion + trp);

                double pr = (ob.rawob.P / MSG_OBS_P_MULTIPLIER);

                // get the measured range and minus the calced range
                double err = pr - (geodist + clockbias - GPS_C * ob.clock_err);

                fi[a] = err;
                a++;
                //Console.WriteLine(err);
            }
        }
    }
}

