package nowifichallengetester;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;

public class TransmitterConnection {
	private static final Logger logger = Logger.getLogger(TransmitterConnection.class.getName());
	
	Socket toTransmitter;
	CommsTester commsTester;
	
	
    static TransmitterConnection instance = null;
    static TransmitterConnection getInstance() { return instance; }
    static TransmitterConnection init(CommsTester commsTester) {
        instance = new TransmitterConnection(commsTester);
        return instance;
    }
    
    ConnectionChangedListener connectionListener;
    public void setConnectionListener(ConnectionChangedListener listener) {
        connectionListener = listener;
    }
    public void notifyConnectionChanged() {
        if (connectionListener != null)
            connectionListener.onConnectionChanged();
    }
    
    
	
	private TransmitterConnection(CommsTester commsTester) {
		this.commsTester = commsTester;
	}

	// public API (callable by UI)
	
	public synchronized boolean isConnected() {
	    return toTransmitter != null;
	}
	
	public synchronized boolean connect(String host, int port) {
		if (toTransmitter != null)
			return false;
		
		try {
			toTransmitter = new Socket(host, port);
			toTransmitter.setTcpNoDelay(false); // disable nagle's algorithm
		} catch (UnknownHostException e) {
			logger.warning("Unknown transmitter host: " + host);
			return false;
		} catch (IOException e) {
			logger.severe("Problem connecting to transmitter: " + e.getMessage());
			logger.log(Level.FINE, e.getMessage(), e);
			return false;
		}
		
		notifyConnectionChanged();
		return true;
	}
	
	public synchronized boolean disconnect() {
		if (toTransmitter == null)
			return false;
		
		boolean result;
		try {
			toTransmitter.shutdownOutput();
			result = true;
		} catch (IOException e) {
			logger.severe("Problem disconnecting transmitter: " + e.getMessage());
			logger.log(Level.FINE, e.getMessage(), e);
			result = false;
		} finally {
			try {
				toTransmitter.close();
			} catch (Exception e) {}
			toTransmitter = null;
		}
		notifyConnectionChanged();
		return result;
	}
	
//	private synchronized Socket getToTransmitter() {
//	    return toTransmitter;
//	}
	
	public boolean sendLocation(int x, int y) {
		// take the time at the start to prevent impact from delays on send
		long timeSentMs = ClockHelper.getAccurateTimeMs();
		
		SPLNoWifiMessage message = SPLNoWifiMessage.locationMessage(x,y);

		try {
			ByteArrayOutputStream bos = new ByteArrayOutputStream();
			bos.write(message.header.toByteArray());
			bos.write(message.location.toByteArray());
			
			OutputStream os = toTransmitter.getOutputStream();
			
			os.write(bos.toByteArray());
		} catch (IOException e) {
			logger.severe("Problem sending location to transmitter: " + e.getMessage());
			logger.log(Level.FINE, "Problem details: ", e);
			return false;
		}
		
		commsTester.setTxMessage(message, timeSentMs);
		
		return true;
	}
	
	public boolean sendData(byte[] data, int len) {
		// take the time at the start to prevent impact from delays on send
		long timeSentMs = ClockHelper.getAccurateTimeMs();

		SPLNoWifiMessage message = SPLNoWifiMessage.dataMessage(data, len);

		try {
			ByteArrayOutputStream bos = new ByteArrayOutputStream();
			bos.write(message.header.toByteArray());
			bos.write(message.data.toByteArray());
			
			OutputStream os = toTransmitter.getOutputStream();
			
			os.write(bos.toByteArray());
		} catch (IOException e) {
			logger.severe("Problem sending data to transmitter: " + e.getMessage());
			logger.log(Level.FINE, "Problem details: ", e);
			return false;
		}
		
		commsTester.setTxMessage(message, timeSentMs);
		
		return true;
	}
}
