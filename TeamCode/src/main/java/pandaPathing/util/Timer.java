package pandaPathing.util;

public class Timer {
    private long startTime;

    /**
     * This creates a new Timer with the start time set to its creation time.
     */
    public Timer() {
        resetTimer();
    }

    /**
     * This resets the Timer's start time to the current time using System.currentTimeMillis().
     */
    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    /**
     * This returns the elapsed time in milliseconds since the start time of the Timer.
     *
     * @return this returns the elapsed time in milliseconds.
     */
    public long getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }

    /**
     * This returns the elapsed time in seconds since the start time of the Timer.
     *
     * @return this returns the elapsed time in seconds.
     */
    public double getElapsedTimeSeconds() {
        return (getElapsedTime() / 1000.0);
    }
}
