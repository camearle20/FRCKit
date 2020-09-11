package frckit.simulation.modelconfig;

import java.util.HashMap;
import java.util.NoSuchElementException;

/**
 * Configuration for an individual transmission
 */
public class TransmissionConfig {
    private static final HashMap<String, Double[]> motorLookup = new HashMap<>();

    static {
        //Register known FRC motors
        //Speed per volt (rad/s / V), torque per volt (N*m / V), nominal voltage (V)
        motorLookup.put("falcon500",    new Double[]{55.67600313862, 0.39083333333, 12.0});
        motorLookup.put("neo550",       new Double[]{95.99310885969, 0.08083333333, 12.0});
        motorLookup.put("neo",          new Double[]{51.31268000863, 0.28000000000, 12.0});
        motorLookup.put("cim",          new Double[]{46.51302456565, 0.20083333333, 12.0});
        motorLookup.put("minicim",      new Double[]{50.96361415823, 0.11750000000, 12.0});
        motorLookup.put("bag",          new Double[]{115.01719770643, 0.03583333333, 12.0});
        motorLookup.put("775pro",       new Double[]{163.45008444927, 0.05916666667, 12.0});
        motorLookup.put("am-redline-a", new Double[]{183.43410438460, 0.05833333333, 12.0});
        motorLookup.put("am-9015",      new Double[]{124.52924212980, 0.03000000000, 12.0});
        motorLookup.put("am-neverest",  new Double[]{47.82202150464, 0.01416666667, 12.0});
        motorLookup.put("am-rs775-125", new Double[]{50.61454830784, 0.02333333333, 12.0});
        motorLookup.put("bb-rs550",     new Double[]{165.80627893946, 0.03166666667, 12.0});
    }

    //Motors can either be defined by a type name, or by defining Kv and Kt, and nominal voltage manually
    public int slot; //Protocol slot this transmission occupies
    public String motorType;
    public double motorSpeedPerVolt = -1; //Negative values here to check that they've actually been set by the user later
    public double motorTorquePerVolt = -1;
    public double nominalVoltage = 12.0; //Default this to 12 since it should always be 12

    public double efficiency = 0.9; //90% is a decent estimate for well built FRC systems
    public int numMotors = 1; //Default to 1 so user doesn't need to put this field in simple mechanisms

    public double motorToOutputRatio = 1.0; //Ratio between the motor and the output of the transmission on the actual hardware.
    public SensorConfig sensor = new SensorConfig(0.0, false);

    /**
     * Tries to load motor values from 'motorType' if it is present,
     * and in any case verifies that Kv and Kt have been set properly.
     */
    public void loadMotorValues() {
        if (motorType != null) {
            Double[] values = motorLookup.get(motorType);
            if (values == null) {
                throw new NoSuchElementException("Motor type '" + motorType + "' not found!  Available types are: " +
                        String.join(", ", motorLookup.keySet()));
            }
            motorSpeedPerVolt = values[0];
            motorTorquePerVolt = values[1];
            nominalVoltage = values[2];
        }
        //Check values
        if (motorSpeedPerVolt == -1 || motorTorquePerVolt == -1) {
            throw new IllegalStateException("Illegal motor parameters");
        }
        //Load sensor
        sensor.loadSensorValues();
    }
}
