package frckit.simulation.webots;


import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

public class WebotsEncoder {
    private final PositionSensor positionSensor;
    private final double resolution;

    private double lastTimestamp; //Used to calculate velocity
    private double lastPosition;
    private double position;
    private double velocity;

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    /**
     * Gets the position of the sensor, scaled based on the resolution of the sensor.
     * @return The "ticked" position of the sensor
     */
    public double getPositionWithResolution() {
        if (position > 0) {
            return Math.floor(position / resolution) * resolution;
        } else if (position < 0) {
            return Math.ceil(position / resolution) * resolution;
        } else {
            return 0.0;
        }
    }

    /**
     * Updates values from the sensor.
     * @param timestamp The simulation timestamp
     */
    public void update(double timestamp) {
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        //Read sensor data
        position = positionSensor.getValue();
        velocity = (position - lastPosition) / dt;
        lastPosition = position;
    }

    public WebotsEncoder(PositionSensor positionSensor, double resolution) {
        this.positionSensor = positionSensor;
        this.resolution = resolution;
    }
}
