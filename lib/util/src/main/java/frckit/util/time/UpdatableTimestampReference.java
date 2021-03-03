package frckit.util.time;

/**
 * Extension of TimestampReference which exposes the method to update the timestamp.  Users should implement this class
 * and then pass instances to the base class (TimestampReference) in order to properly hide access to the "update" method.
 *
 * @see TimestampReference
 */
public interface UpdatableTimestampReference extends TimestampReference {
    /**
     * Updates the timestamp and dt.  Implementations should implement this such that after a call to this method,
     * "getTimestamp()" and "getDt()" return new values.
     */
    void update();
}
