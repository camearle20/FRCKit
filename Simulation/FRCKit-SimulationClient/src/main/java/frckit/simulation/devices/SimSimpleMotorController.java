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
        builder.addMotorCommands(
                RobotCycle.MotorCommand.newBuilder()
                        .setSlot(slot)
                        .setControlType(RobotCycle.MotorCommand.ControlType.NONE)
                        .setCommand(0.0)
                        .setVoltage(voltage)
                        .build()
        );
    }
}
