package pandaPathing.util;

import java.util.ArrayList;
import java.util.List;

public class RingBuffer<T> {

    protected List<T> list;
    protected int index = 0;

    public RingBuffer(int length, T startingValue) {
        list = new ArrayList<>();
        for (int i = 0; i < length; i++) {
            list.add(startingValue);
        }
    }

    /**
     * Adds a new value, returns the overwritten (old) value.
     */
    public T add(T current) {
        T retVal = list.get(index);
        list.set(index, current);
        index = (index + 1) % list.size();
        return retVal;
    }

    public void fill(T overwriteVal) {
        for (int i = 0; i < list.size(); i++) {
            list.set(i, overwriteVal);
        }
        index = 0;  // Reset index on fill for consistency
    }

    public List<T> getList() {
        return list;
    }

    public void changeLength(int length, T overwriteVal) {
        list = new ArrayList<>();
        for (int i = 0; i < length; i++) {
            list.add(overwriteVal);
        }
        index = 0;  // Reset index after changing length
    }
}
