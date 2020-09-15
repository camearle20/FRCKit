package frckit.simulation.webots;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import frckit.simulation.modelconfig.SensorConfig;
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
public class WebotsTransmission {
    private final Motor motor;
    private final DCTransmissionModel model;
    public final WebotsEncoder encoder;

    public WebotsTransmission(Motor motor, PositionSensor sensor, TransmissionConfig transmissionConfig) {
        transmissionConfig.loadMotorValues();
        this.motor = motor;
        this.model = new DCTransmissionModel(
                transmissionConfig.motorSpeedPerVolt,
                transmissionConfig.motorTorquePerVolt,
                transmissionConfig.efficiency,
                transmissionConfig.nominalVoltage,
                transmissionConfig.numMotors,
                transmissionConfig.motorToOutputRatio
        );
        this.encoder = new WebotsEncoder(sensor, transmissionConfig.sensor.resolution);
    }

    /**
     * Updates values from the sensor.
     * @param timestamp The simulation timestamp
     */
    public void updateSensors(double timestamp) {
        encoder.update(timestamp);
    }

    /**
     * Simulates torque applied to the motor.
     * @param voltage The voltage applied to the motor
     */
    public void applyTorque(double voltage) {
        double velocity = encoder.getVelocity();
        double torque = model.simulate(voltage, velocity);
        motor.setTorque(torque);
    }
}
