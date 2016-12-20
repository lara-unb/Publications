package nowifichallengetester;

import java.awt.Container;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.swing.JFrame;
import javax.swing.JButton;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.SwingWorker;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;

import layout.TableLayout;


public class MainFrame extends JFrame implements ActionListener, ConnectionChangedListener {
    private static final Logger logger = Logger.getLogger(MainFrame.class.getName());
	
	private static final long serialVersionUID = -5721770951403109972L;
	
	JTextField txHostTextField;
	JTextField txPortTextField;
	JLabel txStatusLabel;
	JButton txConnectButton;

	JTextField rxPortTextField;
	JLabel rxStatusLabel;
	JButton rxConnectButton;
	
	JButton locationMsgButton;
	JButton dataMsgButton;
	
	JTextArea logTextArea;
	
	boolean txConnected = false;
    boolean rxServerConnected = false;
	boolean rxConnected = false;
	
	final ExecutorService commsExecutor;

	/**
	 * Create the frame.
	 */
	public MainFrame() {
		super("Comms Tester");
		
		commsExecutor = Executors.newSingleThreadExecutor();
		
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		int width = 800;
		int height = 600;
		
		setBounds(100, 100, width, height);

		txHostTextField = new JTextField();
		txHostTextField.setText("localhost");
		txPortTextField = new JTextField();
		txPortTextField.setText(String.valueOf(SPLNoWifiConstants.DEFAULT_TRANSMITTER_ROBOT_PORT));
		txStatusLabel = new JLabel("TODO");
		
		txConnectButton = new JButton("Connect transmitter, T");
		txConnectButton.setActionCommand("TX_CONNECT");
		txConnectButton.addActionListener(this);

		rxPortTextField = new JTextField();
		rxPortTextField.setText(String.valueOf(SPLNoWifiConstants.COMMS_TESTER_PORT));
		rxStatusLabel = new JLabel("TODO");
		
		rxConnectButton = new JButton("Reset connection to receiver, R");
		rxConnectButton.setActionCommand("RX_RESET");
		rxConnectButton.addActionListener(this);
		
		locationMsgButton = new JButton("Send location message");
		locationMsgButton.setActionCommand("SEND_LOCATION");
		locationMsgButton.addActionListener(this);
		
		dataMsgButton = new JButton("Send data message");
		dataMsgButton.setActionCommand("SEND_DATA");
		dataMsgButton.addActionListener(this);
		
		logTextArea = new JTextArea();
//		logTextArea.setText("dummy text at first");
		JScrollPane logScrollPane = new JScrollPane(logTextArea);
		
		Container pane = getContentPane();
		
		final double PREF = TableLayout.PREFERRED;
		final double FILL = TableLayout.FILL;
		final double MARGIN = 10;
		final double HS = 5; // horizontal space
		final double VS = 5; // vertical space
		
		double size[][] = {
				{ MARGIN, PREF, HS, FILL, HS*2, PREF, HS, FILL, MARGIN }, // column widths 
				{ 
					MARGIN,
					PREF, // label
					VS,
					PREF, // host
					VS,
					PREF, // port
					VS,
					PREF, // status
					VS,
					PREF, // connection button
					VS*2,
					100, // main buttons
					VS*2,
					FILL, // text area
					MARGIN
				} // row heights
		};
		
		pane.setLayout(new TableLayout(size));
				
		// fields related to robot T
		
		pane.add(new JLabel("Connection to T"), "1,1,3,1");
		
		pane.add(new JLabel("Remote Host"), "1,3,l,f");
		pane.add(txHostTextField, "3,3");
		
		pane.add(new JLabel("Remote Port"), "1,5,l,f");
		pane.add(txPortTextField, "3,5");
		
		pane.add(new JLabel("Status"), "1,7,l,f");
		pane.add(txStatusLabel, "3,7,l,f");

		pane.add(txConnectButton, "1,9,3,9");
		
		// fields related to robot R
		
		pane.add(new JLabel("Connection from receiver, R"), "5,1,7,1");
		
		pane.add(new JLabel("Listen Port"), "5,3,l,f");
		pane.add(rxPortTextField, "7,3");

		pane.add(new JLabel("Status"), "5,7,l,f");
		pane.add(rxStatusLabel, "7,7,l,f");

		pane.add(rxConnectButton, "5,9,7,9");
		
		
		// message buttons and log
		
		pane.add(locationMsgButton, "1,11,3,11");
		pane.add(dataMsgButton, "5,11,7,11");
		
		pane.add(logScrollPane, "1,13,7,13");

		// Add the logger which will redirect to the console text area of
		// the app.
		TextAreaOutputStream os = new TextAreaOutputStream(logTextArea, 3000);
        Handler sh = new UnbufferedStreamHandler(os);
        sh.setFormatter(CommsTester.consoleFormat);
        sh.setLevel(Level.INFO);
        Logger.getLogger("").addHandler(sh);

		
		updateComponents();

		ReceiverConnection.getInstance().setConnectionListener(this);
		TransmitterConnection.getInstance().setConnectionListener(this);
		
		// somewhat graceful shutdown handling
		addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		        CommsTester.getInstance().shutdown();
		    }
		});
	}
	
	public void onConnectionChanged() {
	    SwingUtilities.invokeLater(new Runnable() {
	        public void run() {
	            updateComponents();
	        }
	    });
	}
	
	void updateComponents() {
		if (TransmitterConnection.getInstance().isConnected()) {
			txStatusLabel.setText("Connected to T");
			txConnectButton.setText("Disconnect T");
			
			locationMsgButton.setEnabled(true);
            dataMsgButton.setEnabled(true);
		} else {
			txStatusLabel.setText("Disconnected");
			txConnectButton.setText("Connect T");
            
            locationMsgButton.setEnabled(false);
            dataMsgButton.setEnabled(false);
		}
		
		if (ReceiverConnection.getInstance().isReceiverConnected()) {
			rxStatusLabel.setText("Connected to R");
		} else if (ReceiverConnection.getInstance().isListening()) {
			rxStatusLabel.setText("Listening for connection");
		} else {
		    rxStatusLabel.setText("Not listening for connection");
		}
		
	}
	
	public void actionPerformed(ActionEvent e) {
		String cmd = e.getActionCommand();
		
		if ("TX_CONNECT".equals(cmd)) {
//			JOptionPane.showMessageDialog(this, "TX_CONNECT");
		    if (!txConnected)
		        connectToTransmitter();
		    else
		        disconnectFromTransmitter();
		} else if ("RX_RESET".equals(cmd)) {
//			JOptionPane.showMessageDialog(this, "RX_RESET");
		    resetReceiverConnection();
		} else if ("SEND_LOCATION".equals(cmd)) {
//			JOptionPane.showMessageDialog(this, "SEND_LOCATION");
//            CommsChallengeTester.testSendLocation();
			sendLocation();
		} else if ("SEND_DATA".equals(cmd)) {
//            JOptionPane.showMessageDialog(this, "SEND_DATA");
//            CommsChallengeTester.testSendData();
		    sendData();
		}
	}
	
	private void connectToTransmitter() {
        logger.fine("connect to transmitter");
        
        final String host = txHostTextField.getText();
        int tmpPort = -1;
        
        try {
            tmpPort = Integer.parseInt(txPortTextField.getText());
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Port must be an integer", "Port error", JOptionPane.ERROR_MESSAGE);
            return;
        }
        final int port = tmpPort;

        SwingWorker<Boolean, Void> worker = new SwingWorker<Boolean,Void>() {
            protected Boolean doInBackground() {
                TransmitterConnection tx = TransmitterConnection.getInstance();
                
                return Boolean.valueOf(tx.connect(host, port));
            }
            
            protected void done() {
                try {
                    Boolean result = get();
                    if (result.booleanValue()) {
                        // connected OK
                        txConnected = true;
                        updateComponents();
                        
                        logger.info("transmitter connected");
                    }
                } catch (InterruptedException e) {
                    // nop
                } catch (ExecutionException e) {
                    logger.severe("failed to connect to transmitter: " + e.getMessage());
                    logger.log(Level.FINE, e.getMessage(), e);
                }
                
            }
        };
        
//        worker.execute();
        commsExecutor.execute(worker);
	}
	
	private void disconnectFromTransmitter() {
        logger.fine("disconnect transmitter");

        SwingWorker<Boolean,Void> worker = new SwingWorker<Boolean,Void>() {
            protected Boolean doInBackground() {
                TransmitterConnection tx = TransmitterConnection.getInstance();
                
                return Boolean.valueOf(tx.disconnect());
            }
            
            protected void done() {
                try {
                    Boolean result = get();
                    if (result.booleanValue()) {
                        // disconnected OK
                        txConnected = false;
                        updateComponents();
                        
                        logger.info("transmitter disconnected");
                    }
                } catch (InterruptedException e) {
                    // nop
                } catch (ExecutionException e) {
                    logger.severe("problem disconnecting transmitter: " + e.getMessage());
                    logger.log(Level.FINE, e.getMessage(), e);

                    // we'll still assume the connection was disconnected
                    txConnected = false;
                }
            }
        };
        
//      worker.execute();
      commsExecutor.execute(worker);
	}
	
    
    private void resetReceiverConnection() {
        logger.fine("reset receiver connection");

        int tmpPort = -1;
        
        try {
            tmpPort = Integer.parseInt(rxPortTextField.getText());
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Port must be an integer", "Port error", JOptionPane.ERROR_MESSAGE);
            return;
        }
        final int port = tmpPort;
        
        SwingWorker<Boolean,Void> worker = new SwingWorker<Boolean,Void>() {
            protected Boolean doInBackground() {
                ReceiverConnection rx = ReceiverConnection.getInstance();
                
                return Boolean.valueOf(rx.resetConnection(port));
            }
            
            protected void done() {
                try {
                    Boolean result = get();
                    if (result.booleanValue()) {
                        // disconnected OK                        
                        logger.info("receiver reset");
                    }
                } catch (InterruptedException e) {
                    // nop
                } catch (ExecutionException e) {
                    logger.severe("problem resetting receiver: " + e.getMessage());
                    logger.log(Level.FINE, e.getMessage(), e);

                    // we'll still assume the connection was disconnected
                    txConnected = false;
                } finally {
                    updateComponents();
                }
            }
        };
        
//      worker.execute();
      commsExecutor.execute(worker);
    }
	
	private void sendLocation() {
        logger.fine("send location");
	    
	    SwingWorker<Boolean,Void> worker = new SwingWorker<Boolean,Void>() {
	        protected Boolean doInBackground() {
	            CommsTester commsTester = CommsTester.getInstance();
	            
	            return Boolean.valueOf(commsTester.sendLocation());
	        }
	        
	        protected void done() {
	            // nop
	        }
	    };
	    
//      worker.execute();
      commsExecutor.execute(worker);
	}

    
    private void sendData() {
        logger.fine("send data");
        
        SwingWorker<Boolean,Void> worker = new SwingWorker<Boolean,Void>() {
            protected Boolean doInBackground() {
                CommsTester commsTester = CommsTester.getInstance();
                
                return Boolean.valueOf(commsTester.sendData());
            }
            
            protected void done() {
                // nop
            }
        };
        
//      worker.execute();
      commsExecutor.execute(worker);
    }
}
