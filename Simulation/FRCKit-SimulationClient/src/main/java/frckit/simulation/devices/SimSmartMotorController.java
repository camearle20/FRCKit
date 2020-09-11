package frckit.simulation.devices;

import frckit.simulation.SimulationGlobals;
import frckit.simulation.protocol.RobotCycleMessage;

public class SimSmartMotorController {
    private final int slot;

    public SimSmartMotorController(int slot) {
        this.slot = slot;
        SimulationGlobals.registerMotor(slot);
    }

    public void setOutputVoltage(double voltage) {
        SimulationGlobals.cycleMessage.motors_controlTypes[slot] = RobotCycleMessage.MotorControlTypes.VOLTAGE;
        SimulationGlobals.cycleMessage.motors_setpoints[slot] = 0.0;
        SimulationGlobals.cycleMessage.motors_voltages[slot] = voltage;
    }

    public void setVelocitySetpoint(double velocityRadPerSec, double ffVolts) {
        SimulationGlobals.cycleMessage.motors_controlTypes[slot] = RobotCycleMessage.MotorControlTypes.VELOCITY;
        SimulationGlobals.cycleMessage.motors_setpoints[slot] = velocityRadPerSec;
        SimulationGlobals.cycleMessage.motors_voltages[slot] = ffVolts;
    }

    public void setVelocitySetpoint(double velocityRadPerSec) {
        setVelocitySetpoint(velocityRadPerSec, 0.0);
    }

    public void setPositionSetpoint(double positionRadians, double ffVolts) {
        SimulationGlobals.cycleMessage.motors_controlTypes[slot] = RobotCycleMessage.MotorControlTypes.POSITION;
        SimulationGlobals.cycleMessage.motors_setpoints[slot] = positionRadians;
        SimulationGlobals.cycleMessage.motors_voltages[slot] = ffVolts;
    }

    public void setPositionSetpoint(double positionRadians) {
        setPositionSetpoint(positionRadians, 0.0);
    }
}
