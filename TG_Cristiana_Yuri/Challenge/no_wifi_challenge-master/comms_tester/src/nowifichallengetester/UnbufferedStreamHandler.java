package nowifichallengetester;

import java.io.OutputStream;
import java.util.logging.LogRecord;
import java.util.logging.StreamHandler;

public class UnbufferedStreamHandler extends StreamHandler {
    
    public UnbufferedStreamHandler(OutputStream os) {
        super();
        setOutputStream(os);
    }
    
    public void publish(LogRecord rec) {
        super.publish(rec);
        flush();
    }
    
    public void close() {
        flush();
    }

}
