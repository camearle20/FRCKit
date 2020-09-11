package frckit.simulation.webots;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import frckit.simulation.modelconfig.TransmissionConfig;
import frckit.simulation.physics.DCTransmissionModel;

/**
 * Union of a webots motor, position sensor, and a motor model.  These 3 components allow
 * torque simulation of the motor
 *
 * At the beginning of a simulation timestep, the data is read from the sensors.
 * The user code is then presented with the data, executes a step, and provides voltages back.
 * After this is complete, the voltages are run through the model, turned to torques, and sent to Webots.
 */
public class WebotsMotorUnion {
    private final Motor motor;
    private final PositionSensor sensor;
    private final DCTransmissionModel model;

    private double lastTimestamp; //Used to calculate velocity
    private double lastPosition;

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    private double position;
    private double velocity;

    public WebotsMotorUnion(Motor motor, PositionSensor sensor, TransmissionConfig transmissionConfig) {
        transmissionConfig.loadMotorValues();
        this.motor = motor;
        this.sensor = sensor;
        this.model = new DCTransmissionModel(
                transmissionConfig.motorSpeedPerVolt,
                transmissionConfig.motorTorquePerVolt,
                transmissionConfig.efficiency,
                transmissionConfig.nominalVoltage,
                transmissionConfig.numMotors,
                transmissionConfig.motorToOutputRatio
        );
    }

    /**
     * Updates values from the sensor.
     * @param timestamp The simulation timestamp
     */
    public void updateSensors(double timestamp) {
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        //Read sensor data
        position = sensor.getValue();
        velocity = (position - lastPosition) / dt;
        lastPosition = position;
    }

    /**
     * Simulates torque applied to the motor.
     * @param voltage The voltage applied to the motor
     */
    public void applyTorque(double voltage) {
        double torque = model.simulate(voltage, velocity);
        motor.setTorque(torque);
    }
}
