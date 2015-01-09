using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace piksi.Comms
{
    public class TCPServer: IStreamExtra
    {
        public System.Net.Sockets.TcpClient Client;

        System.Net.Sockets.TcpListener listener;

        public TCPServer(int port)
        {
            listener = new TcpListener(System.Net.IPAddress.Any, port);

            listener.Start();
        }

        private void DoAcceptTcpClientCallback(IAsyncResult ar)
        {
            // Get the listener that handles the client request.
            TcpListener listener = (TcpListener)ar.AsyncState;

            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);

            // End the operation and display the received data on  
            // the console.
            using (
            Client = listener.EndAcceptTcpClient(ar))
            {
                while (Client.Connected)
                {
                    System.Threading.Thread.Sleep(100);
                }
            }
        }

        public bool dataToRead
        {
            get
            {
                return Client.Available > 0;
            }
            set
            {
                throw new NotImplementedException();
            }
        }


        public byte ReadByte()
        {
            return (byte)Client.GetStream().ReadByte();
        }

        public void Open()
        {
            Console.WriteLine("Waiting for first connection to "+ listener.Server.LocalEndPoint);

            Client = listener.AcceptTcpClient();

            // any connection from here on in will auto connect
            listener.BeginAcceptTcpClient(new AsyncCallback(DoAcceptTcpClientCallback), listener);
        }

        public void Write(byte[] buffer, int offset, int count)
        {
            Client.GetStream().Write(buffer, offset, count);
        }

        public void Write(byte p)
        {
            Write(new byte[] { p }, 0, 1);
        }

        public int Read(byte[] buffer, int offset, int count)
        {
            return Client.GetStream().Read(buffer, offset, count);
        }

        public void Close()
        {
            Client.Close();
        }
    }
}
