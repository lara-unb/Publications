/**
 * @author rudi
 *
 */
package nowifichallengetester;

import java.awt.EventQueue;
import java.io.File;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.UnsupportedEncodingException;


/**
 *
 */
public class CommsTester {
	private static final Logger logger = Logger.getLogger(CommsTester.class.getName());
	
	final static long TIME_NOT_SET = 0;
	final static int ENCRYPTION_PAD_SIZE = 1000;
	
	// we tidy up after timeout by polling with this period
	// (Note: we prevent any messages that are later than the millisecond
	// accurate timeout from being accepted, so the period here can be quite coarse)
	// This is also the period used to provide logging feedback on the UI
	final static int TIMEOUT_POLL_PERIOD_MS = 1000;
	
	long rxLastReceiveTime = TIME_NOT_SET;
	
	SPLNoWifiMessage txMessage;
	long txMessageSentTime;
	boolean txMessageTimedOut;
	boolean rxError;	
    long rxLocationReceivedTime;
	long rxDataFirstFragmentReceivedTime;
	long rxDataLastFragmentReceivedTime;
	int rxDataTotalCount; // total num data bytes received (including duplicates, errors, etc)
	int rxDataUniqueCount; // number of unique bytes received (including unique errors, etc)
	int rxDataMatchCount; // total number of data bytes that were matched
	int rxDataDuplicateMatchCount; // total number of data bytes that were duplicate matches
	int rxDataErrorCount; // total number of mismatching bytes received
	boolean[] rxMessageMatches;
    boolean[] rxMessageReceived;
	
	byte[] encryptionPad = null;
	
	Random random = new Random(System.currentTimeMillis());

	ScheduledExecutorService scheduledExecutor;
	
	static CommsTester instance = null;
	static CommsTester getInstance() { return instance; }
	static CommsTester init() {
	    instance = new CommsTester();
	    return instance;
	}
	
	private CommsTester() {
	    rxMessageMatches = new boolean[SPLNoWifiConstants.DATA_PAYLOAD_MAX_LEN];
	    rxMessageReceived = new boolean[SPLNoWifiConstants.DATA_PAYLOAD_MAX_LEN];
	    
	    // if using encryption of data, uncomment the next lines
	    encryptionPad = new byte[1000];
	    for (int i=0; i<encryptionPad.length; i++)
	        encryptionPad[i] = (byte) random.nextInt(256);
	    
	    initMessageTimeoutPolling();
	}
	
	//
	// test messages
	//
	
	public boolean sendLocation() {
	    int x = random.nextInt(6000) - 3000; // centre line +/- 3m
	    int y = random.nextInt(6000) - 3000; // centre point +/- 3m
	    
	    // NOTE: the transmitterConnection will automatically call setTxMessage
	    // if appropriate
	    return TransmitterConnection.getInstance().sendLocation(x, y);	    
	}
	
	public boolean sendData() {
	    byte[] data = new byte[SPLNoWifiMessage.DataPayload.DATA_SIZE];
//        byte[] data = new byte[200];
	    
	    // FIXME: temporarily we'll just use this text
	    
	    try {
    	    byte[] src = new String("abcdefghijklmnopqrstuvwxyz\n").getBytes("US-ASCII");
    	    
    	    if (encryptionPad == null) {
                for (int i=0; i<data.length; i++) {
                    data[i] = src[i % src.length];              
                }
    	    } else {
    	        // there is a (simple) encryption key so use it
                for (int i=0; i<data.length; i++) {
                    data[i] = (byte) (src[i % src.length]
                            ^ encryptionPad[i % encryptionPad.length]);
                }
    	    }
    	    
	    } catch (UnsupportedEncodingException e) {
	        logger.severe("could not create data to send");
	        logger.log(Level.FINE, e.getMessage(), e);
	        return false;
	    }
	    
	    // data prepared
	    
	    return  TransmitterConnection.getInstance().sendData(data, data.length);
	}
	
	//
	// checking test messages
	//
	
	public synchronized void setTxMessage(SPLNoWifiMessage message, long timeSentMs) {
		txMessage = message;
		txMessageSentTime = timeSentMs;
		txMessageTimedOut = false;
		
		// reset everything about received messages
		rxError = false;
        rxLocationReceivedTime = TIME_NOT_SET;
		rxDataFirstFragmentReceivedTime = TIME_NOT_SET;
		rxDataLastFragmentReceivedTime = TIME_NOT_SET;
		rxDataTotalCount = 0;
		rxDataUniqueCount = 0;
		rxDataMatchCount = 0;
		rxDataDuplicateMatchCount = 0;
		rxDataErrorCount = 0;
		Arrays.fill(rxMessageMatches, false);
        Arrays.fill(rxMessageReceived, false);
		
		logger.info("------------------------------------------------------");
		logger.info("Transmitted message: " + txMessage);
	}
	
	public synchronized void clearTxMessage() {
		txMessage = null;
		txMessageSentTime = TIME_NOT_SET;
		
        logger.info("------------------------------------------------------");
	}
	
//	public synchronized boolean isMessageActive() {
//	    return (txMessage != null) && !isRxMessageLate();
//	}
	
	public synchronized boolean hasTxTimeoutExpired() {
		if (txMessage == null)
			return false;
		
		// OK, there is a txMessage awaiting a response
		long timeNowMs = ClockHelper.getAccurateTimeMs();
		
		logger.finest(String.format("hasTxExpired: now=%d, sent=%d, diff=%d",
		        timeNowMs, txMessageSentTime, timeNowMs - txMessageSentTime));
		
		if ((timeNowMs - txMessageSentTime) >= SPLNoWifiConstants.TIMEOUT_MS)
			return true;
		
		return false;
	}
	
    public synchronized void processReceivedMessage(SPLNoWifiMessage rxMessage) {
        rxLastReceiveTime = ClockHelper.getAccurateTimeMs();
        
        logger.info("Received message: " + rxMessage);

        if (txMessage == null) {
            logger.warning(
                    "No transmitted message or transmitted message expired.");
            return;
        }

        // we have a transmitted message to compare with
        if (!txMessage.header.equals(rxMessage.header)) {
            logger.warning(
                    "Received message header does not match transmitted.");
            logger.fine(String.format("txHeader %d, rxHeader %d", 
                    txMessage.header.type,
                    rxMessage.header.type));
            rxError = true;
            return;
        }
		
        // the header is OK and the type matches
        if (txMessage.header.type == SPLNoWifiMessage.TYPE_LOCATION) {
            if (rxMessage.location == null) {
                logger.warning("Received location message payload missing.");
                rxError = true;

            } else if (hasTxTimeoutExpired()) {
                logger.warning("Received location message too late - ignoring.");
                
            } else if (Math.abs(rxMessage.location.x
                    - txMessage.location.x) > SPLNoWifiConstants.LOCATION_TOLERANCE_MM) {
                logger.warning("Received location.x mismatch too big.");
                rxError = true;

            } else if (Math.abs(rxMessage.location.y
                    - txMessage.location.y) > SPLNoWifiConstants.LOCATION_TOLERANCE_MM) {
                logger.warning("Received location.y mismatch too big.");
                rxError = true;

            } else {
                logger.info("Received location matches transmitted (within tolerance).");
                
                rxLocationReceivedTime = ClockHelper.getAccurateTimeMs();
            }

        } else  if (txMessage.header.type == SPLNoWifiMessage.TYPE_DATA) {
            if (rxMessage.data == null) {
                logger.warning("Received data message payload missing.");
                rxError = true;

            } else if ((rxMessage.data.fragmentOffset < 0) 
                    || (rxMessage.data.fragmentOffset > txMessage.data.fragmentLength)) {
                logger.warning("Received data payload offset is invalid.");
                rxError = true;
                
            } else if ((rxMessage.data.fragmentLength < 0)
                    || ((rxMessage.data.fragmentLength
                            + rxMessage.data.fragmentOffset) > txMessage.data.fragmentLength)) {
                logger.warning("Received data payload length is invalid.");
                rxError = true;
                
            } else if (rxMessage.data.data == null) {
                logger.warning("Received data payload data is missing.");
                rxError = true;
                
            } else if (hasTxTimeoutExpired()) {
                logger.warning("Received data message too late - ignoring.");

            } else {
                if (rxDataFirstFragmentReceivedTime == TIME_NOT_SET) {
                    rxDataFirstFragmentReceivedTime = ClockHelper.getAccurateTimeMs();
                    rxDataLastFragmentReceivedTime = rxDataFirstFragmentReceivedTime;
                } else {
                    rxDataLastFragmentReceivedTime = ClockHelper.getAccurateTimeMs();
                }
                
                // everything is basically OK with the structure so check the
                // individual data bytes
                
                // now check the matches - 
                // Note that we don't want to double count matches if
                // multiple fragments overlap
                int nMatches = 0;
                int nNewMatches = 0;
                int nUniqueReceived = 0;
                
                int rxOffset = rxMessage.data.fragmentOffset;
                
                for (int i = 0; i < rxMessage.data.fragmentLength; i++) {
                    // have we already received this data byte?
                    if (!rxMessageReceived[rxOffset + i]) {
                        nUniqueReceived++;
                        rxMessageReceived[rxOffset + i] = true;
                    }
                    
                    // does the received data byte match what we sent?
                    if (rxMessage.data.data[i] == 
                            txMessage.data.data[rxOffset + i]) {
                        nMatches++;
                        
                        // is this the first time we've matched this data byte?
                        if (!rxMessageMatches[rxOffset + i]) {
                            nNewMatches++;
                            rxMessageMatches[rxOffset + i] = true;
                        }
                    }
                }

                int nDuplicates = nMatches - nNewMatches;
                int nErrors = rxMessage.data.fragmentLength - nMatches;
                
                rxDataTotalCount += rxMessage.data.fragmentLength;
                rxDataUniqueCount += nUniqueReceived;
                rxDataMatchCount += nNewMatches;
                rxDataDuplicateMatchCount += nDuplicates;
                rxDataErrorCount += nErrors;
                
                logger.info(String.format(
                        "  %d matches, %d duplicates, %d errors (cumulative %d matches])",
                        nNewMatches, nDuplicates, nErrors, 
                        rxDataMatchCount));
            }
        }
        
	}
	
    
    public synchronized void processMessageComplete() {
        if (txMessage == null)
            return;
        
        if (txMessage.header.type == SPLNoWifiMessage.TYPE_LOCATION) {
            if (rxError) {
                // details of error have already been logged
                clearTxMessage();
            } else if (rxLocationReceivedTime == TIME_NOT_SET) {
                // no message received yet
                
                if (hasTxTimeoutExpired()) {
                    logger.warning("No location message received within permitted time");
                    clearTxMessage();
                }
            } else {
                // message received (processReceivedMessage should ensure that
                // overdue message is never given a received time)
                long timeOfFlight = rxLocationReceivedTime - txMessageSentTime;
                if (timeOfFlight < SPLNoWifiConstants.TIMEOUT_MS)
                    logger.info(String.format("Received location message arrived on time in %d ms",
                            timeOfFlight));
                else
                    logger.warning("Location message received too late");
                
                clearTxMessage();
            }
            
        } else if (txMessage.header.type == SPLNoWifiMessage.TYPE_DATA) {
            if (rxError) {
                // details of error have already been logged
                clearTxMessage();
            } else if (rxDataFirstFragmentReceivedTime == TIME_NOT_SET) {
                // no message received yet

                if (hasTxTimeoutExpired()) {
                    logger.warning("No data message received within permitted time");
                    clearTxMessage();
                }
            } else {
                // some fragement of data message received within timeout 
                // (guaranteed by processReceivedMessage).
                
                // calculate latency only if we have a complete match already
                // or the message has timed out
                if ((rxDataMatchCount == txMessage.data.fragmentLength) ||
                        hasTxTimeoutExpired()) {

                    long delayFirst = rxDataFirstFragmentReceivedTime - txMessageSentTime;
                    long delayLast = rxDataLastFragmentReceivedTime - txMessageSentTime;
                    long duration;
                    
                    if (rxDataMatchCount == txMessage.data.fragmentLength) {
                        duration = rxDataLastFragmentReceivedTime - txMessageSentTime;
                        logger.info(String.format("Complete data received on time (%d ms)",
                                duration));
                    } else {                        
                        // if message is incomplete then data rate is averaged
                        // over entire timeout period
                        duration = SPLNoWifiConstants.TIMEOUT_MS;
                        
                        logger.info(String.format("Partial data received on time (duration considered to be %d ms)",
                                SPLNoWifiConstants.TIMEOUT_MS));
                    }
                    
                    logger.info(String.format("  first data received after %d ms",
                            delayFirst));
                    logger.info(String.format("  last data received after %d ms",
                            delayLast));
                    logger.info(String.format("  received %d unique bytes, matched %d", 
                            rxDataUniqueCount, rxDataMatchCount));
                    logger.info(String.format("  (received %d total bytes including %d duplicate matches and %d errors)", 
                            rxDataTotalCount, rxDataDuplicateMatchCount, rxDataErrorCount));
                    logger.info(String.format("  error free data rate = %.2f matched bytes/sec",
                            (float) rxDataMatchCount * 1000.0f / duration));
                    
                    clearTxMessage();
                }
                
            }
            
        }
    }
    
    
    public synchronized void pollMessageTimeout() {
        if (txMessage == null)
            return;
        
        
        if (hasTxTimeoutExpired()) {
            logger.finer("pollMessageTimeout - txMessage active - timed out");
            processMessageComplete();
        } else {
            long timeNow = ClockHelper.getAccurateTimeMs();
            long timeOfFlight = timeNow - txMessageSentTime;
            
            if ((timeNow - rxLastReceiveTime) >= TIMEOUT_POLL_PERIOD_MS) {
                logger.info(String.format(
                        "transmitted message still active, %d ms remaining before timeout",
                        SPLNoWifiConstants.TIMEOUT_MS - timeOfFlight));
            }
//            logger.finer("pollMessageTimeout - txMessage active");
        }
    }
    
    private void initMessageTimeoutPolling() {
        scheduledExecutor = Executors.newSingleThreadScheduledExecutor();
        
        final Runnable poller = new Runnable() {
            public void run() {
                pollMessageTimeout();
            }
        };
        
//        final ScheduledFuture<?> pollerHandle = 
        scheduledExecutor.scheduleAtFixedRate(poller, TIMEOUT_POLL_PERIOD_MS,
                TIMEOUT_POLL_PERIOD_MS, TimeUnit.MILLISECONDS);
        
        // the handle can be used to cancel the task later if needed
    }
	
	
    public void shutdown() {
        scheduledExecutor.shutdownNow();
        
        ReceiverConnection.getInstance().shutdown();
        
        
        logger.info("CommsChallengeTester shutdown");        
    }
    
    public static Formatter consoleFormat = new Formatter() {

        SimpleDateFormat df = new SimpleDateFormat("HH:mm:ss.SSS");
        
        @Override
        public String format(LogRecord rec) {
            StringBuffer sb = new StringBuffer();
            
            sb.append(df.format(rec.getMillis()));
            sb.append(' ');
            sb.append(rec.getLevel());
            sb.append(": ");
            
//          sb.append(rec.getMessage());
            String message = formatMessage(rec);
            sb.append(message);
            
            sb.append('\n');
            
            return sb.toString();
        }
        
    };
    
    private static Formatter detailFormat = new Formatter() {

        SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        
        @Override
        public String format(LogRecord rec) {
            StringBuffer sb = new StringBuffer();
            
            sb.append(df.format(rec.getMillis()));
            sb.append(' ');
            sb.append(rec.getLevel());
            sb.append(' ');
            sb.append(rec.getLoggerName());
            sb.append(": ");
            String message = formatMessage(rec);
            sb.append(message);
            sb.append('\n');
            

           if (rec.getThrown() != null) {
               try {
                   StringWriter sw = new StringWriter();
                   PrintWriter pw = new PrintWriter(sw);
                   rec.getThrown().printStackTrace(pw);
                   pw.close();
                   sb.append(sw.toString());
               } catch (Exception ex) {
                   // nop
               }
           }
            
            return sb.toString();
        }
        
    };
    
	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
//	    logger.info("test log before anything set up");
//        logger.finest("finest test log before everything set up");
        
	    final ConsoleHandler consoleHandler = new ConsoleHandler();
        consoleHandler.setLevel(Level.FINEST);
        consoleHandler.setFormatter(consoleFormat);

//        logger.info("test log before app logger set up");
        
        Logger appLogger = Logger.getLogger("");
        for (Handler h : appLogger.getHandlers())
            appLogger.removeHandler(h);
        
        appLogger.setLevel(Level.INFO);        
        appLogger.addHandler(consoleHandler);
        
        Logger.getLogger("comms_challenge_tester").setLevel(Level.INFO);


        logger.info("user.dir = " + System.getProperty("user.dir"));
        
        final SimpleDateFormat df = new SimpleDateFormat("yyyyMMdd");        
        final String suffix = df.format(System.currentTimeMillis()) + ".log";
        
        File logDir = new File(System.getProperty("user.dir"), "logs");
        if (!logDir.exists() && !logDir.mkdirs())
            logDir = new File(System.getProperty("user.dir"));
        
        final String consoleName = Paths.get(logDir.getAbsolutePath(), 
                "no_wifi_console_" + suffix).toString();
        final String detailName = Paths.get(logDir.getAbsolutePath(),
                "no_wifi_detail_" + suffix).toString();
        
        try {
            Handler fhConsole = new FileHandler(consoleName, true);            
            fhConsole.setLevel(Level.INFO);
            fhConsole.setFormatter(consoleFormat);
            appLogger.addHandler(fhConsole);

            Handler fhDetail = new FileHandler(detailName, true);
            fhDetail.setLevel(Level.ALL);
            fhDetail.setFormatter(detailFormat);
            appLogger.addHandler(fhDetail);
            
        } catch (Exception e) {
            logger.warning("Problem setting up file logs" + e.getMessage());
        }

//      Handler sh = new UnbufferedStreamHandler(System.out);//System.err, consoleFormat);
//      sh.setFormatter(consoleFormat);
//      sh.setLevel(Level.FINE);
//      appLogger.addHandler(sh);
        
//        StreamHandler sh = new StreamHandler();
//        sh.setLevel(Level.INFO);
//        sh.setFormatter(consoleFormat);
//        appLogger.addHandler(sh);
        
        logger.info("=========================== start =========================");
        
	    CommsTester commsTester = CommsTester.init();
	    TransmitterConnection.init(commsTester);
	    ReceiverConnection.init(commsTester);
	    
		// start the UI
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					MainFrame frame = new MainFrame();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}


}
