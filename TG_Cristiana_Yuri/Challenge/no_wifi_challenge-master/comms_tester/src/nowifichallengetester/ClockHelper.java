package nowifichallengetester;

public class ClockHelper {
    static boolean initialized = false;
    static long initialOffset;

    public static long getAccurateTimeMs() {
        if (!initialized) {
            initialOffset = System.nanoTime();
            initialized = true;
            return 0;
        } else {
            // remove offset and convert nanosecs to ms
            return (System.nanoTime() - initialOffset) / 1000000;
        }
    }
}
