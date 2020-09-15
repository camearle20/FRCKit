package frckit.simulation.devices;

import frckit.simulation.SimulationClient;
import frckit.simulation.protocol.RobotCycle;

public class SimSmartMotorController {
    private final int slot;

    public SimSmartMotorController(int slot) {
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

    public void setVelocitySetpoint(double velocityRadPerSec, double ffVolts) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.putMotorCommands(slot,
                RobotCycle.MotorCommand.newBuilder()
                        .setControlType(RobotCycle.MotorCommand.ControlType.VELOCITY)
                        .setCommand(velocityRadPerSec)
                        .setVoltage(ffVolts)
                        .build()
        );
    }

    public void setVelocitySetpoint(double velocityRadPerSec) {
        setVelocitySetpoint(velocityRadPerSec, 0.0);
    }

    public void setPositionSetpoint(double positionRadians, double ffVolts) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.putMotorCommands(slot,
                RobotCycle.MotorCommand.newBuilder()
                        .setControlType(RobotCycle.MotorCommand.ControlType.POSITION)
                        .setCommand(positionRadians)
                        .setVoltage(ffVolts)
                        .build()
        );
    }

    public void setPositionSetpoint(double positionRadians) {
        setPositionSetpoint(positionRadians, 0.0);
    }
}
