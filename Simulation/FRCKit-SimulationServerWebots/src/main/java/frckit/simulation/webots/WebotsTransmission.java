package frckit.simulation.webots;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import frckit.simulation.control.PIDController;
import frckit.simulation.modelconfig.TransmissionConfig;
import frckit.simulation.physics.DCTransmissionModel;
import frckit.simulation.protocol.RobotCycle;

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
    private final PIDController controller = new PIDController();
    private final double voltageLimit;

    private RobotCycle.MotorCommand command;

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
        this.voltageLimit = transmissionConfig.nominalVoltage;
    }

    /**
     * Updates values from the sensor.
     * @param timestamp The simulation timestamp
     */
    public void updateSensors(double timestamp) {
        encoder.update(timestamp);
    }

    public void updateCommand(RobotCycle.MotorCommand command) {
        this.command = command;
    }

    public void processPidConfigCommand(RobotCycle.PIDConfigCommand command) {
        switch (command.getSettingCase()) {
            case KP:
                controller.setkP(command.getKP());
                break;
            case KI:
                controller.setkI(command.getKI());
                break;
            case KD:
                controller.setkD(command.getKD());
                break;
            case KF:
                controller.setkF(command.getKF());
                break;
            case IZONE:
                controller.setiZone(command.getIZone());
                break;
            case MINOUTPUT:
                controller.setMinOutput(command.getMinOutput());
                break;
            case MAXOUTPUT:
                controller.setMaxOutput(command.getMaxOutput());
                break;
            case ENCODERPOSITION:
                //Set the offset of the encoder
                encoder.setNewPosition(command.getEncoderPosition());
        }
    }

    /**
     * Simulates torque applied to the motor.
     */
    public void updateMotor(boolean enabled) {
        if (command == null) motor.setTorque(0.0); //This occurs on the first cycle to reset all motors
        double velocity = encoder.getVelocity();
        double position = encoder.getPositionWithResolutionAndOffset();
        double voltage;
        if (!enabled) {
            voltage = 0.0; //Robot is disabled, motors get no voltage
        } else {
            //Robot is enabled, read last command and process PID values.
            voltage = command.getVoltage();
            RobotCycle.MotorCommand.ControlType controlType = command.getControlType();
            double setpoint = command.getCommand();

            //Add outputs from PID controller (if in smart control mode)
            switch (controlType) {
                case POSITION:
                    //Positional PIDF
                    voltage += controller.calculate(setpoint, position);
                    break;
                case VELOCITY:
                    //Velocity PIDF
                    voltage += controller.calculate(setpoint, velocity);
                    break;
            }
        }

        double voltageClamped = Math.min(Math.max(voltage, -voltageLimit), voltageLimit);

        double torque = model.simulate(voltageClamped, velocity);
        motor.setTorque(torque);
    }
}
