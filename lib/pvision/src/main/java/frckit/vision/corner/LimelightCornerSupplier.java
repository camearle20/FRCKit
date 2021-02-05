package frckit.vision.corner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/**
 * Supplies Corner objects from a Limelight camera running 2020+ firmware.  Must be supplied with
 * either the NetworkTables table key for the camera, or alternatively a double array supplier
 * which provides the raw corners array (useful for testing).
 *
 * The array should have 8 elements, and be of the following format:
 * [x1, y1, x2, y2, x3, y3, x4, y4]
 */
public class LimelightCornerSupplier implements Supplier<List<Corner>> {
    private final Supplier<double[]> xyCornersSupplier;

    /**
     * Creates a new LimelightCornerSupplier, using a double array supplier to provide the raw
     * corners array
     * @param xyCornersSupplier A supplier for the raw corners array.
     */
    public LimelightCornerSupplier(Supplier<double[]> xyCornersSupplier) {
        this.xyCornersSupplier = xyCornersSupplier;
    }

    /**
     * Creates a new LimelightCornerSupplier using the table key for the Limelight.
     * @param limelightTableName The table key of the Limelight.
     */
    public LimelightCornerSupplier(String limelightTableName) {
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightTableName);
        NetworkTableEntry tcornxy = limelightTable.getEntry("tcornxy");
        double[] defaultArray = {0, 0, 0, 0, 0, 0, 0, 0};
        this.xyCornersSupplier = () -> tcornxy.getDoubleArray(defaultArray);
    }

    @Override
    public List<Corner> get() {
        double[] xyCorners = xyCornersSupplier.get();

        //The corners array must be length 8.  If it isn't, something is wrong.
        if (xyCorners.length < 8) {
            return Collections.emptyList(); //Return an empty list if there are not the correct number of targets.
        }

        ArrayList<Corner> corners = new ArrayList<>(4);
        corners.add(new Corner(xyCorners[0], xyCorners[1]));
        corners.add(new Corner(xyCorners[2], xyCorners[3]));
        corners.add(new Corner(xyCorners[4], xyCorners[5]));
        corners.add(new Corner(xyCorners[6], xyCorners[7]));

        return corners;
    }
}