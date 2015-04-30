using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

namespace piksi.Comms
{
    public class TCPClient: System.Net.Sockets.TcpClient, IStreamExtra
    {
        private string p1;
        private int p2;

        public TCPClient(string p1, int p2)
        {
            base.NoDelay = true;

            int index = p1.IndexOf(':');

            if (index > 0)
                p1 = p1.Substring(0, index);

            this.p1 = p1;
            this.p2 = p2;
        }

        public bool dataToRead
        {
            get
            {
                return base.Available > 0;
            }
            set
            {
                throw new NotImplementedException();
            }
        }


        public byte ReadByte()
        {
            return (byte)base.GetStream().ReadByte();
        }

        public void Open()
        {
            ServicePointManager.SetTcpKeepAlive(true, 4000, 4000);
            base.Connect(p1, p2);
        }

        public void Write(byte[] buffer, int offset, int count)
        {
            base.GetStream().Write(buffer, offset, count);
        }

        public void Write(byte p)
        {
            Write(new byte[] { p }, 0, 1);
        }

        public int Read(byte[] buffer, int offset, int count)
        {
            return base.GetStream().Read(buffer, offset, count);
        }
    }
}
