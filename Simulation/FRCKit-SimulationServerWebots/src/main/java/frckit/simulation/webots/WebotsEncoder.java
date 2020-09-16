package frckit.simulation.webots;


import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

import java.util.Arrays;

public class WebotsEncoder {
    private final PositionSensor positionSensor;
    private final double resolution;

    private double lastTimestamp; //Used to calculate velocity
    private double lastPosition;
    private double position;
    private double velocity;

    private double offset = 0.0;

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setNewPosition(double position) {
        offset = position - getPositionWithResolution();
    }

    public double getPositionWithResolution() {
        if (resolution == 0.0) {
            return position; //Infinite resolution configured
        } else if (position > 0) {
            return Math.floor(position / resolution) * resolution;
        } else if (position < 0) {
            return Math.ceil(position / resolution) * resolution;
        }
        return 0.0;
    }

    /**
     * Gets the position of the sensor, scaled based on the resolution of the sensor.
     * @return The "ticked" position of the sensor
     */
    public double getPositionWithResolutionAndOffset() {
        return getPositionWithResolution() + offset;
    }

    public void reset() {
        lastTimestamp = 0.0;
        lastPosition = 0.0;
        position = 0.0;
        velocity = 0.0;
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
