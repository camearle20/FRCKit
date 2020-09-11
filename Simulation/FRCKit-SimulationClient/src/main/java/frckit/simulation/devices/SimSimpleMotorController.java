package frckit.simulation.devices;

import frckit.simulation.SimulationGlobals;
import frckit.simulation.protocol.RobotCycleMessage;

public class SimSimpleMotorController {
    private final int slot;

    public SimSimpleMotorController(int slot) {
        this.slot = slot;
        SimulationGlobals.registerMotor(slot);
    }

    public void setOutputVoltage(double voltage) {
        SimulationGlobals.cycleMessage.motors_controlTypes[slot] = RobotCycleMessage.MotorControlTypes.VOLTAGE;
        SimulationGlobals.cycleMessage.motors_setpoints[slot] = 0.0;
        SimulationGlobals.cycleMessage.motors_voltages[slot] = voltage;
    }
}
