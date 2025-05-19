package pandaPathing.util;

public class Timer {
    private long startTime = 0;
    public boolean timerDone;

    public Timer() {
        reset();
    }

    public void reset() {
        startTime = System.currentTimeMillis();
    }

    public long getElapsedTime() {
        return System.currentTimeMillis() - startTime;
    }

    public double getElapsedTimeSeconds() {
        return getElapsedTime() / 1000.0;
    }

    public boolean isBetween(long start, long stop){
        return getElapsedTime() > start && getElapsedTime() < stop;
    }
}
