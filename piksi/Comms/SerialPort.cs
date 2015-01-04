using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi.Comms
{
    public class SerialPort: System.IO.Ports.SerialPort, IStreamExtra
    {
        public SerialPort(string p1, int p2): 
            base(p1,p2)
        {

        }

        public bool dataToRead
        {
            get
            {
                return this.BytesToRead > 0;
            }
            set
            {
                throw new NotImplementedException();
            }
        }


        public new byte ReadByte()
        {
            return (byte)base.ReadByte();
        }

        public void Write(byte p)
        {
            Write(new byte[] { p }, 0, 1);
        }
    }
}
