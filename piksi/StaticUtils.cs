using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

static class StaticUtils
{

    public static DateTime GetFromGps(int weeknumber, double seconds)
    {
        DateTime datum = new DateTime(1980, 1, 6, 0, 0, 0,DateTimeKind.Utc);
        DateTime week = datum.AddDays(weeknumber * 7);
        DateTime time = week.AddSeconds(seconds);
        return time;
    }

    public static void GetFromTime(DateTime time, ref int week, ref double seconds)
    {
        DateTime datum = new DateTime(1980, 1, 6, 0, 0, 0, DateTimeKind.Utc);

        TimeSpan dif = time - datum;

        int weeks = (int)(dif.TotalDays / 7);

        week = weeks;

        dif = time - datum.AddDays(weeks * 7);

        seconds = dif.TotalSeconds;
    }

    public static TPacket ByteArrayToStructure<TPacket>(this byte[] bytearray, int startoffset) where TPacket : struct
    {        
        return ReadUsingPointer<TPacket>(bytearray, startoffset);

    }

    static T ReadUsingPointer<T>(byte[] data, int startoffset) where T : struct
    {
        unsafe
        {
            fixed (byte* p = &data[startoffset])
            {
                return (T)Marshal.PtrToStructure(new IntPtr(p), typeof(T));
            }
        }
    }


    public static T ByteArrayToStructureGC<T>(byte[] bytearray, int startoffset) where T : struct
    {
        GCHandle gch = GCHandle.Alloc(bytearray, GCHandleType.Pinned);
        try
        {
            return (T)Marshal.PtrToStructure(gch.AddrOfPinnedObject() + startoffset, typeof(T));
        }
        finally
        {
            gch.Free();
        }
    }

    public static unsafe void ByteArrayToStructure(byte[] bytearray, ref object obj, int startoffset)
    {
        int len = Marshal.SizeOf(obj);

        IntPtr i = Marshal.AllocHGlobal(len);

        // create structure from ptr
        //obj = Marshal.PtrToStructure(i, obj.GetType());

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

    public static byte[] StructureToByteArray(object obj, int extralen = 0)
    {
        int len = Marshal.SizeOf(obj);
        len += extralen;
        byte[] arr = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(obj, ptr, true);
        Marshal.Copy(ptr, arr, 0, len);
        Marshal.FreeHGlobal(ptr);
        return arr;
    }
}
