using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace piksi
{
    static class StaticUtils
    {

        public static TPacket ByteArrayToStructure<TPacket>(this byte[] bytearray, int startoffset) where TPacket : struct
        {
            object newPacket = new TPacket();
            ByteArrayToStructure(bytearray, ref newPacket, startoffset);
            return (TPacket)newPacket;
        }

        public static void ByteArrayToStructure(byte[] bytearray, ref object obj, int startoffset)
        {
            int len = Marshal.SizeOf(obj);

            IntPtr i = Marshal.AllocHGlobal(len);

            // create structure from ptr
            obj = Marshal.PtrToStructure(i, obj.GetType());

            try
            {
                // copy byte array to ptr
                Marshal.Copy(bytearray, startoffset, i, len);
            }
            catch (Exception ex)
            {
                Console.WriteLine("ByteArrayToStructure FAIL " + ex.Message);
            }

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);
        }

        public static void ByteArrayToStructureEndian(byte[] bytearray, ref object obj, int startoffset)
        {

            int len = Marshal.SizeOf(obj);
            IntPtr i = Marshal.AllocHGlobal(len);
            byte[] temparray = (byte[])bytearray.Clone();

            // create structure from ptr
            obj = Marshal.PtrToStructure(i, obj.GetType());

            // do endian swap
            object thisBoxed = obj;
            Type test = thisBoxed.GetType();

            int reversestartoffset = startoffset;

            // Enumerate each structure field using reflection.
            foreach (var field in test.GetFields())
            {
                // field.Name has the field's name.
                object fieldValue = field.GetValue(thisBoxed); // Get value

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                if (typeCode != TypeCode.Object)
                {
                    Array.Reverse(temparray, reversestartoffset, Marshal.SizeOf(fieldValue));
                    reversestartoffset += Marshal.SizeOf(fieldValue);
                }
                else
                {
                    reversestartoffset += ((byte[])fieldValue).Length;
                }

            }

            try
            {
                // copy byte array to ptr
                Marshal.Copy(temparray, startoffset, i, len);
            }
            catch (Exception ex)
            {
                Console.WriteLine("ByteArrayToStructure FAIL" + ex.ToString());
            }

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

        }

    }
}
