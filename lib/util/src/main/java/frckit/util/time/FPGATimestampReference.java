package frckit.util.time;

import edu.wpi.first.wpilibj.Timer;

/**
 * Timestamp reference which uses the RoboRIO FPGA (or simulated FPGA time) to provide the timestamp values.
 */
public class FPGATimestampReference implements UpdatableTimestampReference {
    private double timestamp;
    private double dt;

    @Override
    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public double getDt() {
        return dt;
    }

    @Override
    public void update() {
        //Read timestamp from the FPGA:
        double now = Timer.getFPGATimestamp(); //Read timestamp
        if (timestamp == 0.0) {
            //The timestamp has never been initialized, so the dt should be zero.
            dt = 0.0;
        } else {
            //The timestamp was initialized, so calculate dt as normal.
            dt = now - timestamp;
        }
        timestamp = now; //Update timestamp
    }
}
