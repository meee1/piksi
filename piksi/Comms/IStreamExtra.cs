using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace piksi.Comms
{
    interface IStreamExtra
    {
        bool dataToRead { get; set; }

        byte ReadByte();

        void Open();

        void Close();

        void Write(byte[] buffer, int offset, int count);

        int Read(byte[] buffer, int offset, int count);

        void Write(byte p);
    }
}
