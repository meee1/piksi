using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace piksi.Comms
{
    public class NTRIP: IStreamExtra
    {
        public System.Net.Sockets.TcpClient Client;
        private string url;
        Uri uri;

        public NTRIP(string url)
        {
            uri = new Uri("http://"+url);
            this.url = url;

            Client = new TcpClient();
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
            Client.Connect(uri.Host, uri.Port);

            string usernamePassword = uri.UserInfo;
            string auth = "Authorization: Basic " + Convert.ToBase64String(new ASCIIEncoding().GetBytes(usernamePassword));

            if (usernamePassword == "")
                auth = "";

            NetworkStream ns = Client.GetStream();

            StreamWriter sw = new StreamWriter(ns);
            BinaryReader sr = new BinaryReader(ns);

            string line = "GET " + uri.PathAndQuery + " HTTP/1.1\r\nHost: " + uri.Host
          + "\r\nNtrip-Version: Ntrip/1.0\r\nUser-Agent: NTRIP BNC/2.6\r\n"
          + auth + "\r\nConnection: close\r\nAccept-Encoding: gzip\r\nAccept-Language: en-AU,*\r\n\r\n";

            sw.Write(line);

            sw.Flush();

            System.Threading.Thread.Sleep(2000);

            if (Client.Available > 0)
            {
                byte[] data = new byte[Client.Available];
                ns.Read(data, 0, data.Length);

                line = ASCIIEncoding.ASCII.GetString(data);

                if (!line.Contains("200"))
                    throw new Exception("Bad ntrip Responce\n\n" + line);
            }
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
