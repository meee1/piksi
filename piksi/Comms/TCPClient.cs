using System;
using System.Collections.Generic;
using System.Linq;
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
            base.Connect(p1, p2);
        }

        public void Write(byte[] buffer, int offset, int count)
        {
            base.GetStream().Write(buffer, offset, count);
        }

        public int Read(byte[] buffer, int offset, int count)
        {
            return base.GetStream().Read(buffer, offset, count);
        }
    }
}
