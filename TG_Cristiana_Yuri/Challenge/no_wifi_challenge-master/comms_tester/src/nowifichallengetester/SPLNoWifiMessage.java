package nowifichallengetester;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Locale;

public class SPLNoWifiMessage {
	
    public final static int DATA_SIZE = SPLNoWifiConstants.DATA_PAYLOAD_MAX_LEN;
    
	public final static int TYPE_LOCATION = 1;
	public final static int TYPE_DATA = 2;
	
	/**
	 * 
	 */
	public static class Header {
		public final static int SIZE = 2; // uint16_t for type
				
		public int type;

		

		@Override
        public boolean equals(Object obj) {
		    Header other = (Header) obj;
		    
		    return type == other.type;
        }

        public byte[] toByteArray() {
			ByteBuffer buffer = ByteBuffer.allocate(SIZE);
	        buffer.order(ByteOrder.LITTLE_ENDIAN);

	        buffer.putShort((short) (type & 0xFFFF));

	        return buffer.array();			
		}
		
	    public boolean fromByteArray(ByteBuffer buffer) {
	    	try {
	            buffer.order(ByteOrder.LITTLE_ENDIAN);

	            type = (buffer.getShort() & 0xFFFF);
	            
	            return true;
	    		
	        } catch (RuntimeException e) {
	            return false;
	        }
	    }
	}
	
	/**
	 * 
	 */
	public static class LocationPayload {
		public final static int SIZE = 2 // int16_t for x
				+ 2; // int16_t for y
				
		public int x;
		public int y;

		public byte[] toByteArray() {
			ByteBuffer buffer = ByteBuffer.allocate(SIZE);
	        buffer.order(ByteOrder.LITTLE_ENDIAN);

	        buffer.putShort((short) x);
	        buffer.putShort((short) y);

	        return buffer.array();			
		}
		
	    public boolean fromByteArray(ByteBuffer buffer) {
	    	try {
	            buffer.order(ByteOrder.LITTLE_ENDIAN);

	            x = buffer.getShort();
	            y = buffer.getShort();
	            
	            return true;
	    		
	        } catch (RuntimeException e) {
	            return false;
	        }
	    }
	}
	
	/**
	 * 
	 */
	public static class DataPayload {
		public final static int HEADER_SIZE = 2 // uint16_t for offset
				+ 2; // uint16_t for length
		public final static int DATA_SIZE = SPLNoWifiMessage.DATA_SIZE;
		public final static int SIZE = HEADER_SIZE + DATA_SIZE;
		
		public int fragmentOffset;
		public int fragmentLength;
		public byte[] data;
		
		public byte[] toByteArray() {
			ByteBuffer buffer = ByteBuffer.allocate(SIZE);
	        buffer.order(ByteOrder.LITTLE_ENDIAN);

	        buffer.putShort((short)(fragmentOffset & 0xFFFF));
	        buffer.putShort((short)(fragmentLength & 0xFFFF));
	        buffer.put(data, 0, fragmentLength);
	        
	        return Arrays.copyOf(buffer.array(), buffer.position());			
		}
		
	    public boolean headerFromByteArray(ByteBuffer buffer) {
	    	try {
	            buffer.order(ByteOrder.LITTLE_ENDIAN);

	            fragmentOffset = (buffer.getShort() & 0xFFFF);
	            fragmentLength = (buffer.getShort() & 0xFFFF);
	            
	            return true;
	    		
	        } catch (RuntimeException e) {
	            return false;
	        }
	    }

	    public boolean dataFromByteArray(ByteBuffer buffer) {
	    	try {
	            buffer.order(ByteOrder.LITTLE_ENDIAN);

	            if ((data == null) || (data.length < fragmentLength))
	            	data = new byte[DATA_SIZE];
	            buffer.get(data, 0, fragmentLength);
	            
	            return true;
	    		
	        } catch (RuntimeException e) {
	            return false;
	        }
	    }
	}

	
	Header header;
	LocationPayload location;
	DataPayload data;
	
	public String toString() {
		if (header == null)
			return "corrupt";
		
		if (header.type == TYPE_LOCATION) {
			if (location == null)
				return "Location missing/corrupt";
			else
				return String.format(Locale.US, "Location x=%d, y=%d", 
						location.x, location.y);
		} else if (header.type == TYPE_DATA) {
			if (data == null)
				return "Data missing/corrupt";
			else {
				return String.format(Locale.US, "Data offset=%d, length=%d",
						data.fragmentOffset, data.fragmentLength);
			}
		} else {
			return "Header corrupt";
		}
	}
	
	
	public static SPLNoWifiMessage locationMessage(int x, int y) {
		SPLNoWifiMessage message = new SPLNoWifiMessage();
		
		message.header = new Header();		
		message.header.type = SPLNoWifiMessage.TYPE_LOCATION;
		
		message.location = new SPLNoWifiMessage.LocationPayload();
		message.location.x = x;
		message.location.y = y;

		return message;
	}
	
	public static SPLNoWifiMessage dataMessage(byte[] data, int len) {
		SPLNoWifiMessage message = new SPLNoWifiMessage();
		
		message.header = new Header();		
		message.header.type = SPLNoWifiMessage.TYPE_DATA;
		
		message.data = new SPLNoWifiMessage.DataPayload();
		message.data.fragmentOffset = 0;
		message.data.fragmentLength = len;
		message.data.data = Arrays.copyOf(data, len);
		
		return message;
	}
}
