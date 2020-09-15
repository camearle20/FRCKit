package frckit.simulation.devices;

import frckit.simulation.SimulationClient;
import frckit.simulation.protocol.RobotCycle;

public class SimSimpleMotorController {
    private final int slot;

    public SimSimpleMotorController(int slot) {
        this.slot = slot;
    }

    public void setOutputVoltage(double voltage) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.putMotorCommands(slot,
                RobotCycle.MotorCommand.newBuilder()
                        .setControlType(RobotCycle.MotorCommand.ControlType.NONE)
                        .setCommand(0.0)
                        .setVoltage(voltage)
                        .build()
        );
    }
}
