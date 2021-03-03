package frckit.util.time;

/**
 * An object which can provide the current timestamp, in seconds, as well as the "dt", which is the time that has elapsed
 * between the last loop cycle.
 *
 * It is expected that the timestamp will change exactly once per loop cycle (at the beginning), and all
 * calls to "getTimestamp()" or "getDt()" within that loop cycle will return the same value, regardless of the time
 * that has elapsed since the loop started.  This helps to ensure that the code is running on a consistent timebase
 * and that measurements can be referenced with their timestamp without worrying about some minuscule time difference.
 *
 * Note that this class should probably not be implemented directly.  The intention is that the user implements
 * UpdatableTimestampReference, which exposes an additional "update" method.  Then, code which needs to consume the
 * timestamp can be passed an instance of TimestampReference, which hides away the "update" method preventing that code
 * from calling it accidentally (which would invalidate the timestamp measurements).
 *
 * @see UpdatableTimestampReference
 */
public interface TimestampReference {
    /**
     * Returns the current loop cycle's timestamp, in seconds.  This timestamp does not necessarily start from zero
     * (and in most cases will not) and is only guaranteed to move forwards consistently.
     * @return The current loop cycle's timestamp, in seconds
     */
    double getTimestamp();

    /**
     * Returns the number of seconds that have elapsed between the start of the last loop cycle and the start
     * of the current loop cycle.  Note that calls to this method may return zero (particularly the first call)
     * and thus care should be taken to handle this in any situation where the user is dividing by the dt.
     * @return The number of seconds that have elapsed between the start of the last loop cycle and the start
     *         of the current loop cycle.
     *
     */
    double getDt();
}
