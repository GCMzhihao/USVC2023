

/*
MAVLink protocol implementation (auto-generated by mavgen.py)

Note: this file has been auto-generated. DO NOT EDIT
*/

using System;
using System.Collections;
using System.Collections.Generic;
    
namespace MavLink
{
    public static class MavlinkSettings
    {
		public const string WireProtocolVersion = "1.0";
		public const byte ProtocolMarker = 0xfe;
		public const bool CrcExtra = true;
		public const bool IsLittleEndian = true;
    }
    
    public delegate MavlinkMessage MavlinkPacketDeserializeFunc(byte[] bytes, int offset);

    //returns the message ID, offset is advanced by the number of bytes used to serialize
    public delegate int MavlinkPacketSerializeFunc(byte[] bytes, ref int offset, object mavlinkPacket);
 
    public class MavPacketInfo
    {
        public MavlinkPacketDeserializeFunc Deserializer;
        public int [] OrderMap;
        public byte CrcExtra;

         public MavPacketInfo(MavlinkPacketDeserializeFunc deserializer, byte crcExtra)
         {
             this.Deserializer = deserializer;
             this.CrcExtra = crcExtra;
         }
    }
 
    public static class MavLinkSerializer
    {
        public static void SetDataIsLittleEndian(bool isLittle)
        {
            bitconverter.SetDataIsLittleEndian(isLittle);
        }
    
        private static readonly FrameworkBitConverter bitconverter = new FrameworkBitConverter(); 
    
        public static Dictionary<int, MavPacketInfo> Lookup = new Dictionary<int, MavPacketInfo>
        {
			{0, new MavPacketInfo(Deserialize_ROCKER, 253)},
			{1, new MavPacketInfo(Deserialize_PARAM_READ_REQUEST, 97)},
			{2, new MavPacketInfo(Deserialize_PARAM_READ_ACK, 14)},
			{3, new MavPacketInfo(Deserialize_PARAM_WRITE, 11)},
			{4, new MavPacketInfo(Deserialize_PARAM_WRITE_ACK, 54)},
			{5, new MavPacketInfo(Deserialize_CMD_WRITE, 220)},
			{6, new MavPacketInfo(Deserialize_CMD_ACK, 251)},
			{7, new MavPacketInfo(Deserialize_USV_STATE, 99)},
			{8, new MavPacketInfo(Deserialize_USV_SET, 174)},
		};

		internal static MavlinkMessage Deserialize_ROCKER(byte[] bytes, int offset)
		{
			return new Msg_rocker
			{
				leftX = bitconverter.ToInt16(bytes, offset + 0),
				leftY = bitconverter.ToInt16(bytes, offset + 2),
				rightX = bitconverter.ToInt16(bytes, offset + 4),
				rightY = bitconverter.ToInt16(bytes, offset + 6),
				switchA = bitconverter.ToInt16(bytes, offset + 8),
				switchB = bitconverter.ToInt16(bytes, offset + 10),
				switchC = bitconverter.ToInt16(bytes, offset + 12),
				switchD = bitconverter.ToInt16(bytes, offset + 14),
				switchE = bitconverter.ToInt16(bytes, offset + 16),
				switchF = bitconverter.ToInt16(bytes, offset + 18),
				switchG = bitconverter.ToInt16(bytes, offset + 20),
			};
		}

		internal static MavlinkMessage Deserialize_PARAM_READ_REQUEST(byte[] bytes, int offset)
		{
			return new Msg_param_read_request
			{
				SYS_TYPE = bytes[offset + 0],
				DEV_ID = bytes[offset + 1],
				param_id = bytes[offset + 2],
			};
		}

		internal static MavlinkMessage Deserialize_PARAM_READ_ACK(byte[] bytes, int offset)
		{
			return new Msg_param_read_ack
			{
				value = bitconverter.ToSingle(bytes, offset + 0),
				param_id = bytes[offset + 4],
			};
		}

		internal static MavlinkMessage Deserialize_PARAM_WRITE(byte[] bytes, int offset)
		{
			return new Msg_param_write
			{
				value = bitconverter.ToSingle(bytes, offset + 0),
				param_id = bytes[offset + 4],
			};
		}

		internal static MavlinkMessage Deserialize_PARAM_WRITE_ACK(byte[] bytes, int offset)
		{
			return new Msg_param_write_ack
			{
				param_id = bytes[offset + 0],
			};
		}

		internal static MavlinkMessage Deserialize_CMD_WRITE(byte[] bytes, int offset)
		{
			return new Msg_cmd_write
			{
				SYS_TYPE = bytes[offset + 0],
				DEV_ID = bytes[offset + 1],
				cmd_id = bytes[offset + 2],
			};
		}

		internal static MavlinkMessage Deserialize_CMD_ACK(byte[] bytes, int offset)
		{
			return new Msg_cmd_ack
			{
				cmd_id = bytes[offset + 0],
				cmd_ack_id = bytes[offset + 1],
			};
		}

		internal static MavlinkMessage Deserialize_USV_STATE(byte[] bytes, int offset)
		{
			return new Msg_usv_state
			{
				longitude = bitconverter.ToSingle(bytes, offset + 0),
				latitude = bitconverter.ToSingle(bytes, offset + 4),
				speed = bitconverter.ToSingle(bytes, offset + 8),
				heading = bitconverter.ToSingle(bytes, offset + 12),
				battery_voltage = bitconverter.ToSingle(bytes, offset + 16),
			};
		}

		internal static MavlinkMessage Deserialize_USV_SET(byte[] bytes, int offset)
		{
			return new Msg_usv_set
			{
				Speed = bitconverter.ToSingle(bytes, offset + 0),
				Heading = bitconverter.ToSingle(bytes, offset + 4),
				SYS_TYPE = bytes[offset + 8],
				DEV_ID = bytes[offset + 9],
			};
		}

		internal static int Serialize_ROCKER(this Msg_rocker msg, byte[] bytes, ref int offset)
		{
			bitconverter.GetBytes(msg.leftX, bytes, offset + 0);
			bitconverter.GetBytes(msg.leftY, bytes, offset + 2);
			bitconverter.GetBytes(msg.rightX, bytes, offset + 4);
			bitconverter.GetBytes(msg.rightY, bytes, offset + 6);
			bitconverter.GetBytes(msg.switchA, bytes, offset + 8);
			bitconverter.GetBytes(msg.switchB, bytes, offset + 10);
			bitconverter.GetBytes(msg.switchC, bytes, offset + 12);
			bitconverter.GetBytes(msg.switchD, bytes, offset + 14);
			bitconverter.GetBytes(msg.switchE, bytes, offset + 16);
			bitconverter.GetBytes(msg.switchF, bytes, offset + 18);
			bitconverter.GetBytes(msg.switchG, bytes, offset + 20);
			offset += 22;
			return 0;
		}

		internal static int Serialize_PARAM_READ_REQUEST(this Msg_param_read_request msg, byte[] bytes, ref int offset)
		{
			bytes[offset + 0] = msg.SYS_TYPE;
			bytes[offset + 1] = msg.DEV_ID;
			bytes[offset + 2] = msg.param_id;
			offset += 3;
			return 1;
		}

		internal static int Serialize_PARAM_READ_ACK(this Msg_param_read_ack msg, byte[] bytes, ref int offset)
		{
			bitconverter.GetBytes(msg.value, bytes, offset + 0);
			bytes[offset + 4] = msg.param_id;
			offset += 5;
			return 2;
		}

		internal static int Serialize_PARAM_WRITE(this Msg_param_write msg, byte[] bytes, ref int offset)
		{
			bitconverter.GetBytes(msg.value, bytes, offset + 0);
			bytes[offset + 4] = msg.param_id;
			offset += 5;
			return 3;
		}

		internal static int Serialize_PARAM_WRITE_ACK(this Msg_param_write_ack msg, byte[] bytes, ref int offset)
		{
			bytes[offset + 0] = msg.param_id;
			offset += 1;
			return 4;
		}

		internal static int Serialize_CMD_WRITE(this Msg_cmd_write msg, byte[] bytes, ref int offset)
		{
			bytes[offset + 0] = msg.SYS_TYPE;
			bytes[offset + 1] = msg.DEV_ID;
			bytes[offset + 2] = msg.cmd_id;
			offset += 3;
			return 5;
		}

		internal static int Serialize_CMD_ACK(this Msg_cmd_ack msg, byte[] bytes, ref int offset)
		{
			bytes[offset + 0] = msg.cmd_id;
			bytes[offset + 1] = msg.cmd_ack_id;
			offset += 2;
			return 6;
		}

		internal static int Serialize_USV_STATE(this Msg_usv_state msg, byte[] bytes, ref int offset)
		{
			bitconverter.GetBytes(msg.longitude, bytes, offset + 0);
			bitconverter.GetBytes(msg.latitude, bytes, offset + 4);
			bitconverter.GetBytes(msg.speed, bytes, offset + 8);
			bitconverter.GetBytes(msg.heading, bytes, offset + 12);
			bitconverter.GetBytes(msg.battery_voltage, bytes, offset + 16);
			offset += 20;
			return 7;
		}

		internal static int Serialize_USV_SET(this Msg_usv_set msg, byte[] bytes, ref int offset)
		{
			bitconverter.GetBytes(msg.Speed, bytes, offset + 0);
			bitconverter.GetBytes(msg.Heading, bytes, offset + 4);
			bytes[offset + 8] = msg.SYS_TYPE;
			bytes[offset + 9] = msg.DEV_ID;
			offset += 10;
			return 8;
		}
	}

}

