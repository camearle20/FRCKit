package frckit.vision.corner;

import java.util.Comparator;

/**
 * Represents the pixel coordinates of the corner of a bounding box from a vision system.
 */
public class Corner {
    /**
     * A comparator which can be used to sort lists of corners by their x coordinate, in ascending order.
     */
    public static final Comparator<Corner> SORT_BY_X = Comparator.comparingDouble(Corner::getX);

    /**
     * A comparator which can be used to sort lists of corners by their y coordinate, in ascending order.
     */
    public static final Comparator<Corner> SORT_BY_Y = Comparator.comparingDouble(Corner::getY);

    private final double x;
    private final double y;

    /**
     * Creates a new Corner object.
     * @param x The x coordinate of the corner
     * @param y The y coordinate of the corner
     */
    public Corner(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
