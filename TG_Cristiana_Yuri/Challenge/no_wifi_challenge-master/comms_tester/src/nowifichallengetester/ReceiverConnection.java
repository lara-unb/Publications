package nowifichallengetester;

import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ReceiverConnection {
	private static final Logger logger = Logger.getLogger(ReceiverConnection.class.getName());
	
	private ServerSocket server;
	private Socket fromReceiver;
	private CommsTester commsTester;
	
	ExecutorService executor;
	
    static ReceiverConnection instance = null;
    static ReceiverConnection getInstance() { return instance; }
    static ReceiverConnection init(CommsTester commsTester) {
        if (instance != null)
            throw new IllegalStateException("Cannot init ReceiverConnection more than once");
        
        instance = new ReceiverConnection(commsTester);
        
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
    
	
	private ReceiverConnection(CommsTester commsTester) {
		this.commsTester = commsTester;

		// start listening for incoming connections
		resetConnection(SPLNoWifiConstants.COMMS_TESTER_PORT);
		initMessageReceiver();
	}

	public synchronized void disconnectReceiver(boolean notifyRemote) {
        if (fromReceiver != null) {
            logger.fine("disconnectReceiver - fromReceiver not null, notifyRemote=" + notifyRemote);
            
            if (notifyRemote) {
                try {
                    fromReceiver.shutdownInput();
                } catch (IOException e) {
                    logger.warning("Problem disconnecting receiver");
                    logger.log(Level.FINE, "Problem details: ", e);
                } finally {
                    try {
                        fromReceiver.close();
                    } catch (Exception e) {}
                    fromReceiver = null;
                }
            } else {
                try {
                    fromReceiver.close();
                } catch (Exception e) {
                    logger.warning("Unexpected problem closing receiver connection: " + e.getMessage());
                }
                fromReceiver = null;
            }
        }
        
        notifyConnectionChanged();
    }
	
	private synchronized void disconnectServer() {
        if (server != null) {
            logger.fine("disconnectServer - server not null");
            try {
                server.close();
            } catch (IOException e) {
                logger.warning("Problem closing listening port");
                logger.log(Level.FINE, "Problem deatils: ", e);
            } finally {
                server = null;
            }
        }

        notifyConnectionChanged();
	}
	
	public synchronized boolean resetConnection(int listenPort) {
		// shutdown the receiver connection
	    disconnectReceiver(true);
		
		// shutdown the listening port
	    disconnectServer();
		
		// reopen the (possibly new) listening port

		try {
			server = new ServerSocket(listenPort, 1);
		} catch (IOException e) {
			logger.severe("Problem opening listening port");
			logger.log(Level.FINE, "Problem deatils: ", e);
			server = null;
			return false;
		}
		
        notifyConnectionChanged();
		return true;
	}
	
	public synchronized boolean isListening() {
	    return (server != null);
	}
	
	public synchronized boolean isReceiverConnected() {
	    return (fromReceiver != null);
	}
	
	public synchronized void setSocketFromReceiver(Socket skt) {
	    if (fromReceiver != null)
	        throw new IllegalStateException("fromReceiver already set");
	    
	    fromReceiver = skt;

	    notifyConnectionChanged();
	}
	
	/**
	 * read specified num bytes completely or fail
	 * 
	 * @param inputStream
	 * @param buffer
	 * @param len
	 * @return true if len bytes read, false on EOF
	 * @throws IOException
	 */
	private boolean readCompletely(InputStream inputStream, byte[] buffer, int len) 
			throws IOException {
		int remaining = len;
		int offset = 0;
		
		while (remaining > 0) {
			int nRead = inputStream.read(buffer, offset, remaining);
			if (nRead == -1)
				return false; // EOF
			else {
				remaining -= nRead;
				offset += nRead;
			}				
		}
		
		return true;
	}
	
	/**
	 * Receive one complete SPLNoWifiMessage
	 * 
	 * @return message read (if successful), null if no more messages (normal EOF)
	 * @throws IOException
	 */
	private SPLNoWifiMessage receiveMessage() throws IOException {
	    InputStream fromReceiverInputStream;
	    synchronized(this) {
	        if (fromReceiver == null)
	            return null;
	        
	        fromReceiverInputStream = fromReceiver.getInputStream();
	    }
		
		byte[] buffer = new byte[SPLNoWifiMessage.DataPayload.SIZE];
		ByteBuffer byteBuffer = ByteBuffer.wrap(buffer);
		
		// read the header
        if (!readCompletely(fromReceiverInputStream, buffer,
                SPLNoWifiMessage.Header.SIZE))
			return null;
		
		SPLNoWifiMessage message = new SPLNoWifiMessage();
		message.header = new SPLNoWifiMessage.Header();
		
		message.header.fromByteArray(byteBuffer);
		
		// read the appropriate payload based on the header
		
		if (message.header.type == SPLNoWifiMessage.TYPE_LOCATION) {
			message.location = new SPLNoWifiMessage.LocationPayload();

            if (!readCompletely(fromReceiverInputStream, buffer,
                    SPLNoWifiMessage.LocationPayload.SIZE))
				throw new EOFException("EOF in location payload");

			byteBuffer.rewind();
			message.location.fromByteArray(byteBuffer);
			
		} else if (message.header.type == SPLNoWifiMessage.TYPE_DATA) {
			message.data = new SPLNoWifiMessage.DataPayload();
			
            if (!readCompletely(fromReceiverInputStream, buffer,
                    SPLNoWifiMessage.DataPayload.HEADER_SIZE))
				throw new EOFException("EOF in data payload header");

			byteBuffer.rewind();
			message.data.headerFromByteArray(byteBuffer);
			
			if (!readCompletely(fromReceiverInputStream, buffer, message.data.fragmentLength))
				throw new EOFException("EOF in data payload data");

			byteBuffer.rewind();
			message.data.dataFromByteArray(byteBuffer);
			
		} else {
			throw new IOException("Unrecognised header type");
		}
		
		return message;
	}
	
	/**
	 * receive messages within a single TCP connection from the receiver robot.
	 * 
	 * This function returns when there are no more messages to be received
	 * (i.e. the receiver closes the connection cleanly) or there is an
	 * IOException of some sort
	 * 
	 * By the time this function returns, the receiver connection has been
	 * closed.
	 */
	public void receiveMessages() {
	    logger.fine("receiveMessages");
	    
	    synchronized(this) {
    		if (fromReceiver != null)
    			throw new IllegalStateException("there is already an active receiver socket");
    		
    		if (server == null)
    		    return;
	    }
		
		try {
			setSocketFromReceiver(server.accept());
			
			logger.info("accepted connection from receiver");
			
			SPLNoWifiMessage message;
			
			while ((message = receiveMessage()) != null) {
				commsTester.processReceivedMessage(message);
				commsTester.processMessageComplete();
			}

		} catch (SocketException e) {
			// we assume socket was closed on this side as part of reset
            logger.fine(
                    "Socket exception (assumed UI close) of receiver: "
                            + e.getMessage());
            // disconnect probably already done, but just in case...
            disconnectReceiver(true);
        } catch (EOFException e) {
            logger.warning("EOF while receiving messages, close connection: " + e.getMessage());
            disconnectReceiver(true);
		} catch (IOException e) {
            logger.warning("Unexpected problem receiving messages: " + e.getMessage());
            disconnectReceiver(true);
		}
		
        // if we get this far with no exceptions, it means that we
        // got a clean shutdown of the connection from the remote side
        // so tidy up here also

		disconnectReceiver(false);
	}
	

    private void initMessageReceiver() {
        executor = Executors.newSingleThreadExecutor();
        
        final Runnable messageReceiver = new Runnable() {
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    receiveMessages();
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        break; // interrupted means we're done
                    } catch (Exception e) {
                        logger.severe("Unhandled exception while receiving messages: " + e.getMessage());
                        logger.log(Level.FINE, e.getMessage(), e);
                    }
                    logger.fine("Message receiver shutting down");
                }
            }
        };
        
        executor.execute(messageReceiver);
    }

    public void shutdown() {
        executor.shutdownNow();
        
        disconnectReceiver(true);
        disconnectServer();
        
        try {
            executor.awaitTermination(1, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            logger.warning("receiver connection timed out awaiting thread termination.");
            // nop
        }
        
        logger.fine("receiver connection shutdown");
    }
}
