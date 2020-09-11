package frckit.simulation.modelconfig;

import java.util.HashMap;
import java.util.NoSuchElementException;

public class SensorConfig {
    public String sensorType;
    public double resolution = 0.0;
    public int decoding = 4;
    public boolean inMotor = false;
    public double sensorToOutputRatio = 1.0;

    public SensorConfig(double resolution, boolean inMotor) {
        this.resolution = resolution;
        this.inMotor = inMotor;
    }

    /**
     * Returns the proper resolution of the sensor given the configured decoding multiplier
     * @return The true resolution of the sensor after decoding.
     */
    public double getDecodingResolution() {
        return resolution / decoding;
    }

    private static final HashMap<String, SensorConfig> sensorLookup = new HashMap<>();
    private static final double TWO_PI = 2.0 * Math.PI;


    static {
        //All of these are multiplied by 4.0 to put them in the unit of 1x decoding.  If decoding is set to 4,
        //which is the default, they will all be scaled back
        sensorLookup.put("falcon500", new SensorConfig(TWO_PI / 2048.0 * 4.0, true));
        sensorLookup.put("neo", new SensorConfig(TWO_PI / 42.0 * 4.0, true));
        sensorLookup.put("ctre-mag-encoder", new SensorConfig(TWO_PI / 4096.0 * 4.0, false));
        sensorLookup.put("rev-mag-encoder", new SensorConfig(TWO_PI / 8192.0 * 4.0, false));
    }

    public void loadSensorValues() {
        if (sensorType != null) {
            SensorConfig config = sensorLookup.get(sensorType);
            if (config == null) {
                throw new NoSuchElementException("Sensor type '" + sensorType + "' not found!  Available types are: " +
                        String.join(", ", sensorLookup.keySet()));
            }
            resolution = config.resolution;
            inMotor = config.inMotor;
        }
    }
}
