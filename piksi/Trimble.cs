using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    class Trimble
    {
        public class CdatType17
        {

            public CdatType17()
            {

            }

            private measT[] m_svMeas= new measT[1];


            public double clockOffset { get; set; }
            public int clockRolloverCount { get; set; }
            public int gpsWeek { get; set; }
            public byte nMeasurementSets { get; set; }
            public CdatType17.measT[] svMeas { get { return m_svMeas; } set { m_svMeas = value; } }
            public double tmeasGPS { get; set; }
            public double tmeasRCVR { get; set; }

            public void svMeasResize(int newSize)
            {
                if (newSize != this.svMeas.Length)
                {
                    Array.Resize<measT>(ref m_svMeas, newSize);
                    this.m_svMeas.Initialize();
                }
                for (int i = 0; i < this.m_svMeas.Length; i++)
                {
                    if (this.m_svMeas[i] == null)
                    {
                        this.m_svMeas[i] = new measT();
                    }
                }

            }

            public class measT
            {
                public measT() { }

                public double auxPseudorange { get; set; }
                public short azimuthDeg { get; set; }
                public double doppler { get; set; }
                public short elevationDeg { get; set; }
                public byte flags1 { get; set; }
                public byte flags2 { get; set; }
                public byte flagStatus { get; set; }
                public double lidcpr { get; set; }
                public double pCodeL2minusCcodeL1 { get; set; }
                public double phaseBiasL1 { get; set; }
                public double phaseBiasL2 { get; set; }
                public double phaseL1 { get; set; }
                public double phaseL2 { get; set; }
                public byte prn { get; set; }
                public double pseudorange { get; set; }
                public double snCountsL1 { get; set; }
                public double snCountsL2 { get; set; }

            }
        }

        public class CtcomTail
        {
            public CtcomTail() { }

            public byte checksum { get; set; }
            public byte postamble = 0x3;

            public void writeToStream(BinaryWriter bw)
            {
                bw.Write(this.checksum);
                bw.Write(this.postamble);
            }
        }

        public enum tcomType
        {
            STATIC = 1,
            ROVING = 2,
            STATION = 3,
            INFO = 4,
            REMARK = 5,
            REQ_RECVR_ANT = 6,
            GETSERIAL = 6,
            RET_RECVR_ANT = 7,
            RSERIAL = 7,
            GETSTAT1 = 8,
            RETSTAT1 = 9,
            GETPOS = 11,
            RETPOS = 12,
            REQ_TRACKING = 13,
            RET_TRACKING = 14,
            GET_TIME = 20,
            RET_TIME = 21,
            REQ_SATINFO = 22,
            RET_SATINFO = 23,
            GETPOS3 = 48,
            RETPOS3 = 49,
            SETSESSTN = 66,
            GETSESSTN = 67,
            RETSESSTN = 68,
            SETCOMMS = 72,
            COMMOUT = 73,
            GETOPT = 74,
            RETOPT = 75,
            STARTSURV = 76,
            ENDSURV = 77,
            SETIDLE = 79,
            COMMCTRL = 81,
            CHANCTRL = 83,
            GET_SVDATA = 84,
            RET_SVDATA = 85,
            GET_RAWDATA = 86,
            RET_RAWDATA = 87,
            RESETRCVR = 88,
            UNDELFILE = 96,
            RFILE_PACKET = 97,
            RFILE_RESTART = 98,
            RFILE_VALIDATE_REC = 99,
            APPFILE = 100,
            REQAPPFILE = 101,
            REQAPPDIR = 102,
            RETAPPDIR = 103,
            RFILE_DELFILE = 104,
            DIR_INFO_GET = 105,
            RET_DIRINFO = 106,
            REQ_DIRECTORY = 107,
            DIR_PAGEPACKET = 108,
            ACTAPPFILE = 109,
            BREAKRET = 110,
            BREAKREQ = 111,
            GETXBASE = 117,
            RETXBASE = 118,
            SET_TARGET_MEM = 128,
            GET_TARGET_MEM = 130,
            SCREEN_DUMP = 130,
            HH_OLD_COMMAND = 131,
            PIPE = 132,
            FUNNEL = 133,
            BREAK_TO_MONITOR = 135,
            MONITOR_RESET_RCVR = 136,
            ERROR_LOG_COMMAND = 145,
            TARGET_MEMORY_MSG = 146,
            ERROR_LOG_RESPONSE = 146,
            CMR_MESSAGE = 147,
            E_FILE_PACKET = 149,
            E_FILE_REQUEST = 150,
            E_DIR_REQUEST = 156,
            E_DIR_RETURN = 157,
            E_DIR_STOP = 158,
            BOOT_ERROR_REQUEST = 160,
            E_DELETE_FILE = 160,
            BOOT_ERROR_RESPONSE = 161,
            E_UNDELETE_FILE = 161,
            MET_TILT_CMD = 181,
        }

        static byte[] convert(string value, uint nBytes)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(value);
            uint num = (value.Length > nBytes) ? nBytes : ((uint)value.Length);
            byte[] buffer2 = new byte[nBytes];
            for (uint i = 0; i < nBytes; i++)
            {
                buffer2[i] = (byte)' ';
            }
            for (uint i = 0; i < num; i++)
            {
                buffer2[i] = bytes[i];
            }
            return buffer2;
        }

        public static void writeTrimble15(Stream outputto, int week, uint tow)
        {

            MemoryStream dat = new MemoryStream();

            BinaryWriter bw = new BinaryWriter(dat);

            bw.Write((byte)0x02);
            bw.Write((byte)0x00);
            bw.Write((byte)tcomType.RET_TIME);
            bw.Write((byte)20); // 20

            reverseEndian(bw, ((int)(tow / 1000)).ToString().PadLeft(7, '0')); // tow

            reverseEndian(bw, convert(week.ToString(), 4)); // week

            reverseEndian(bw, convert("16", 2)); // leap seconds

            reverseEndian(bw, convert("0000", 4)); // tz offset

            reverseEndian(bw, convert("UTC", 3)); // tz

            // exclude startchar
            bw.Seek(1, SeekOrigin.Begin);
            int num = 0;
            while (dat.Position < dat.Length)
            {
                byte num2 = (byte)dat.ReadByte();
                num = (byte)(num + num2);
            }

            bw = new BinaryWriter(outputto);

            byte[] data = dat.ToArray();

            bw.Write(data, 0, (int)data.Length);

            CtcomTail tail = new CtcomTail();
            tail.checksum = (byte)(num);
            tail.writeToStream(bw);
        }

        private static int trimblerawreply;

        private static double[] cpold = new double[33];
        private static int[] lockcount = new int[33];

        public static void writeTrimbleR17(Stream outputto, double towinms, List<piksi.msg_obs_content_t> obs)
        {
            CdatType17 type = new CdatType17
            {
                tmeasRCVR = towinms,
                clockOffset = 0,
                nMeasurementSets = 0
            };
            
            type.svMeasResize(obs.Count);

            for (int i = 0; i < obs.Count; i++)
            {
                {
                    double[] azel = new double[2] { 0, 0 };
                    try
                    {
                        // satazel(myposllh, raw2.satxyzscale[raw.data[i].SV], ref azel);

                        if (azel[0] < 0)
                            azel[0] = 0;

                        if (azel[1] < 0)
                            azel[1] = 0;
                    }
                    catch { }

                    type.svMeas[type.nMeasurementSets].prn = (byte)(obs[i].prn + 1);
                    type.svMeas[type.nMeasurementSets].elevationDeg = (short)((azel[1]));//Convert.ToInt16(this.m_measurement[i].header.elevation);
                    type.svMeas[type.nMeasurementSets].azimuthDeg = (short)((azel[0]));//Convert.ToInt16(this.m_measurement[i].header.azimuth);
                    type.svMeas[type.nMeasurementSets].flags1 = 0; // 249
                    type.svMeas[type.nMeasurementSets].flags2 = 0; // 4
                    type.svMeas[type.nMeasurementSets].flagStatus = 0;
                    // l1 only
                    type.svMeas[type.nMeasurementSets].snCountsL1 = obs[i].snr;
                    type.svMeas[type.nMeasurementSets].pseudorange = obs[i].P / piksi.MSG_OBS_P_MULTIPLIER;
                    type.svMeas[type.nMeasurementSets].phaseL1 = (obs[i].L.Li + (obs[i].L.Lf / 256.0));
                    type.svMeas[type.nMeasurementSets].doppler = 0;// (type.svMeas[type.nMeasurementSets].phaseL1 - cpold[obs[i].prn]);
                    cpold[obs[i].prn] = type.svMeas[type.nMeasurementSets].phaseL1;
                    type.svMeas[type.nMeasurementSets].lidcpr = 0.0;
                    CdatType17.measT st1 = type.svMeas[type.nMeasurementSets];
                    st1.flags1 = (byte)(st1.flags1 | 0x40); // l1 data valid
                    CdatType17.measT st2 = type.svMeas[type.nMeasurementSets];
                    st2.flags1 = (byte)(st2.flags1 | 0x80); // new positon computed

                    if (lockcount[obs[i].prn] == obs[i].lock_counter)
                    {
                        st2.flags1 = (byte)(st2.flags1 | 0x10); // l1 phase valid
                        st2.flags1 = (byte)(st2.flags1 | 0x8); // L1 Phase Lock Point
                    }

                    lockcount[obs[i].prn] = obs[i].lock_counter;

                    type.svMeas[type.nMeasurementSets].auxPseudorange = 0.0;
                    CdatType17.measT st21 = type.svMeas[type.nMeasurementSets];
                    st21.flagStatus = (byte)(st21.flagStatus | 1);
                    CdatType17.measT st22 = type.svMeas[type.nMeasurementSets];
                    st22.flagStatus = (byte)(st22.flagStatus | 2);

                    type.nMeasurementSets = (byte)(type.nMeasurementSets + 1);
                }
            }


            MemoryStream dat = new MemoryStream();

            writeToStream(new BinaryWriter(dat), type);

            BinaryWriter bw = new BinaryWriter(outputto);

            CtcomTail tail = new CtcomTail();

            byte packetsize = 244;

            int totalleft = (int)dat.Length;
            int loops = (totalleft / packetsize) + 1;
            byte sending = 0;
            int pageno = 1;

            for (int a = 0; a < loops; a++)
            {
                //trace(3, "%d/%d\n", pageno, loops);
                if (totalleft > packetsize)
                {
                    sending = packetsize;
                }
                else
                {
                    sending = (byte)totalleft;
                }

                bw.Write((byte)0x02); // stx
                bw.Write((byte)0x00); // status
                bw.Write((byte)tcomType.RET_RAWDATA); // type
                bw.Write((byte)(sending + 4)); // 4 extra for 4 ytes below

                bw.Write((byte)0);// record type
                byte page = Convert.ToByte((int)((pageno << 4) & 240));
                page = (byte)(page | Convert.ToByte((int)(loops & 15)));
                bw.Write((byte)page);// page
                bw.Write((byte)trimblerawreply); // replyno
                bw.Write((byte)0); // flags - 0 = expanded and no realtimedata

                byte[] packet = dat.ToArray();

                bw.Write(packet, ((int)dat.Length - totalleft), sending);

                // single bytes only
                tail.checksum = computeChecksum((byte)0x00, (byte)tcomType.RET_RAWDATA, (byte)(sending), (byte)0, (byte)page, (byte)trimblerawreply, (byte)0, dat, ((int)dat.Length - totalleft));
                tail.writeToStream(bw);
                pageno++;
                totalleft -= sending;
            }

            trimblerawreply++;
    
        }

        static byte computeChecksum(byte headstatus, byte headtype, byte headlength, byte rtype, byte page, byte reply, byte flags, Stream data, int pos)
        {
            byte num = (byte)((int)((headstatus + headtype) + (headlength + 4) + rtype + page + reply + flags));
            data.Seek(pos, SeekOrigin.Begin);
            int len = headlength;
            while (len > 0)
            {
                byte num2 = (byte)data.ReadByte();
                num = (byte)(num + num2);
                len--;
            }
            return num;
        }

        static void writeToStream(BinaryWriter outStream, CdatType17 thiss)
        {
            reverseEndian(outStream, thiss.tmeasRCVR);
            reverseEndian(outStream, thiss.clockOffset);
            reverseEndian(outStream, thiss.nMeasurementSets);
            foreach (CdatType17.measT st in thiss.svMeas)
            {
                writeToStreammeasT(outStream, st);
            }
        }

        static void writeToStreammeasT(BinaryWriter outStream, CdatType17.measT thiss)
        {
            reverseEndian(outStream, thiss.prn);
            reverseEndian(outStream, thiss.flags1);
            reverseEndian(outStream, thiss.flags2);
            reverseEndian(outStream, thiss.flagStatus);
            reverseEndian(outStream, thiss.elevationDeg);
            reverseEndian(outStream, thiss.azimuthDeg);
            if ((thiss.flags1 & 64) != 0)
            {
                reverseEndian(outStream, thiss.snCountsL1);
                reverseEndian(outStream, thiss.pseudorange);
                reverseEndian(outStream, thiss.phaseL1);
                reverseEndian(outStream, thiss.doppler);
                reverseEndian(outStream, thiss.lidcpr);
            }
            if ((thiss.flags1 & 1) != 0)
            {
                reverseEndian(outStream, thiss.snCountsL2);
                reverseEndian(outStream, thiss.phaseL2);
                reverseEndian(outStream, thiss.pCodeL2minusCcodeL1);
                if ((thiss.flags2 & 0x20) != 0)
                {
                    reverseEndian(outStream, thiss.auxPseudorange);
                }
            }
        }

        static void reverseEndian(BinaryWriter outStream, object input)
        {
            byte[] temp;
            if (input is double)
            {
                temp = BitConverter.GetBytes((double)input);
            }
            else if (input is Single)
            {
                temp = BitConverter.GetBytes((Single)input);
            }
            else if (input is short)
            {
                temp = BitConverter.GetBytes((short)input);
            }
            else if (input is ushort)
            {
                temp = BitConverter.GetBytes((ushort)input);
            }
            else if (input is int)
            {
                temp = BitConverter.GetBytes((int)input);
            }
            else if (input is uint)
            {
                temp = BitConverter.GetBytes((uint)input);
            }
            else if (input is byte)
            {
                temp = new byte[] { (byte)input };
            }
            else if (input is sbyte)
            {
                temp = new byte[] { (byte)(sbyte)input };
            }
            else if (input is string)
            {
                temp = Encoding.ASCII.GetBytes((string)input);
                Array.Reverse(temp);
            }
            else if (input is byte[])
            {
                temp = (byte[])input;
                Array.Reverse(temp);
            }
            else
            {
                throw new Exception("ERROR");
            }

            Array.Reverse(temp);
            outStream.Write(temp, 0, temp.Length);
        }

        const double SC2RAD = 3.1415926535898;   /* semi-circle to radian (IS-GPS) */

        public static void writeTrimble55_1(Stream outputto, piksi.ephemeris_t es, int prn)
        {
            MemoryStream dat = new MemoryStream();

            BinaryWriter bw = new BinaryWriter(dat);

            bw.Write((byte)0x02);
            bw.Write((byte)0x00);
            bw.Write((byte)tcomType.RET_SVDATA);
            bw.Write((byte)(180 - 4)); // 4 for header

            reverseEndian(bw, (byte)0x1); // Data type
            reverseEndian(bw, (byte)prn);

            //int tow = (int)time2gpst(es.ttr, ref weekwork);
            //int toc = (int)time2gpst(es.toc, ref weekwork);
            //int toe = (int)time2gpst(es.toe, ref weekwork);

            //if (toe < tow)
            {
              //  weekwork--;
            }

            //int iodc =

            reverseEndian(bw, (ushort)(es.toe.wn));
            reverseEndian(bw, (ushort)0);//es.iodc);
            reverseEndian(bw, (byte)0); // res
            reverseEndian(bw, (byte)0);//es.iode);
            reverseEndian(bw, (int)es.toc.tow);
            reverseEndian(bw, (int)es.toc.tow);
            reverseEndian(bw, (int)es.toe.tow);
            reverseEndian(bw, (double)es.tgd); // check scale ex -1.72294676303864e-8
            reverseEndian(bw, (double)es.af2);
            reverseEndian(bw, (double)es.af1);
            reverseEndian(bw, (double)es.af0);
            reverseEndian(bw, (double)es.crs);
            reverseEndian(bw, (double)es.dn / SC2RAD);
            reverseEndian(bw, (double)es.m0 / SC2RAD);
            reverseEndian(bw, (double)es.cuc / SC2RAD);
            reverseEndian(bw, (double)es.ecc);
            reverseEndian(bw, (double)es.cus / SC2RAD);
            reverseEndian(bw, (double)es.sqrta); // checkme
            reverseEndian(bw, (double)es.cic / SC2RAD);
            reverseEndian(bw, (double)es.omega0 / SC2RAD);
            reverseEndian(bw, (double)es.cis / SC2RAD);
            reverseEndian(bw, (double)es.inc / SC2RAD);
            reverseEndian(bw, (double)es.crc);
            reverseEndian(bw, (double)es.w / SC2RAD);
            reverseEndian(bw, (double)es.omegadot / SC2RAD);
            reverseEndian(bw, (double)es.inc_dot / SC2RAD);

            uint flags = 0;

            flags = flags | (uint)(0 & 1);
            flags = flags | (uint)((1 & 3) << 1);
            flags = flags | (uint)((1) << 3);
            flags = flags | (uint)(((es.healthy-1) & 63) << 4);
            flags = flags | (uint)(((uint)0 & 1) << 10);
            flags = flags | (uint)((0 & 15) << 11);
            //flags = flags | (uint)((raw.alm[sat - 1].svconf & 7) << 16);
            flags = flags | (uint)((0) << 19);

            reverseEndian(bw, (int)flags);

            // exclude startchar
            bw.Seek(1, SeekOrigin.Begin);
            int num = 0;
            while (dat.Position < dat.Length)
            {
                byte num2 = (byte)dat.ReadByte();
                num = (byte)(num + num2);
            }

            bw = new BinaryWriter(outputto);

            bw.Write(dat.ToArray(), 0, (int)dat.Length);

            CtcomTail tail = new CtcomTail();
            tail.checksum = (byte)num;
            tail.writeToStream(bw);
        }

    }
}
