package frckit.util;

import java.util.function.DoubleSupplier;

/**
 * A DoubleSupplier which provides the same value until it is updated with a new value.
 * This is useful for applications where we only want to update a value once per loop cycle, and use the value
 * in many places.
 */
public class StoredDoubleSupplier implements DoubleSupplier {
    private double value;
    private final DoubleSupplier source;

    /**
     * Creates a new StoredDoubleSupplier given a source
     * @param source Source to retrieve new values from.  This occurs when "update" is called.
     */
    public StoredDoubleSupplier(DoubleSupplier source) {
        this.source = source;
    }

    /**
     * Retrieves a new value from the source, and updates the stored value of this supplier to provide it on
     * future calls to "getAsDouble".
     */
    public void update() {
        value = source.getAsDouble();
    }

    /**
     * @return The value as of the last call to "update"
     */
    @Override
    public double getAsDouble() {
        return value;
    }
}
