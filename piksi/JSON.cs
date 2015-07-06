using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    public class JSON
    {
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct jsonmessage
        {
            public ulong timestamp;
            public object metadata;
            public piksi.piksimsg data;
            public ulong delta;
        }

            public static void test()
        {
            StreamReader sr = new StreamReader(@"C:\Users\michael\AppData\Local\VirtualStore\Program Files (x86)\Swift Navigation\Piksi Console\serial-link-20150706-090309.log.json");

            piksi pk = new piksi();

            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine();
                var item = getpacket(line);

                pk.ProcessMessage(item);
            }
        }

        public static piksi.piksimsg getpacket(string line)
        {
            var item = Newtonsoft.Json.JsonConvert.DeserializeObject<jsonmessage>(line);
            
            var data = ((piksi.piksimsg)item.data);

            /*
            var msgtype = data["msg_type"].Value<int>();

            switch ((piksi.MSG)msgtype)
            {
                case piksi.MSG.MSG_PACKED_OBS:
                    var hdr = data.ToObject<piksi.msg_obs_header_t>();
                    break;
            }
            */
            return data;
        }
    }
}
