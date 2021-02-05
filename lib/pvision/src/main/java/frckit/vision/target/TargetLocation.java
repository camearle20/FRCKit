package frckit.vision.target;

/**
 * Represents the location of a target in the camera's perspective in 3D space.
 */
public class TargetLocation {
    private final double x;
    private final double y;
    private final double z;
    private final double skew;

    /**
     * Creates a new TargetElement with y, z, and skew components.  The x value is set to 1.0
     *
     * This constructor is useful for the majority of cases when working with a camera in the
     * FRC coordinate system
     * @param y The y value
     * @param z The z value
     * @param skew The skew value
     */
    public TargetLocation(double y, double z, double skew) {
        this.x = 1.0;
        this.y = y;
        this.z = z;
        this.skew = skew;
    }

    /**
     * Creates a new TargetElement with x, y, z, and skew components.
     * @param x The x value
     * @param y The y value
     * @param z The z value
     * @param skew The skew value
     */
    public TargetLocation(double x, double y, double z, double skew) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.skew = skew;
    }

    /**
     * @return The x value
     */
    public double getX() {
        return x;
    }

    /**
     * @return The y value
     */
    public double getY() {
        return y;
    }

    /**
     * @return The z value
     */
    public double getZ() {
        return z;
    }

    /**
     * @return The skew value
     */
    public double getSkew() {
        return skew;
    }

    @Override
    public String toString() {
        return "TargetElement(" + x + ", " + y + ", " + z + ", skew=" + skew + ")";
    }
}