package pandaPathing.util;

public class Timer {
    private long startTime = 0;
    private boolean started = false;

    /**
     * Creates a new Timer with start time unset (not started yet).
     */
    public Timer() {
        // startTime is unset, waitMs will start timing on first call
    }

    /**
     * Resets the timer start time to now and marks it as started.
     */
    public void reset() {
        started = true;
        startTime = System.currentTimeMillis();
    }

    /**
     * Returns elapsed time in milliseconds since start.
     * Returns 0 if timer not started.
     */
    public long getElapsedTime() {
        if (!started) return 0;
        return System.currentTimeMillis() - startTime;
    }

    /**
     * Returns elapsed time in seconds since start.
     */
    public double getElapsedTimeSeconds() {
        return getElapsedTime() / 1000.0;
    }

    /**
     * Returns true if the specified time in milliseconds has elapsed.
     * Starts counting on first call (auto-start).
     * Automatically resets when the wait time has passed.
     *
     * @param milliseconds time to wait
     * @return true if wait time has passed
     */
    public boolean waitMs(long milliseconds) {
        if (!started) {
            started = true;
            startTime = System.currentTimeMillis();
            return false;
        }

        if (System.currentTimeMillis() - startTime >= milliseconds) {
            started = false; // auto-reset for next use
            return true;
        }

        return false;
    }
}
