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
        builder.addMotorCommands(
                RobotCycle.MotorCommand.newBuilder()
                        .setSlot(slot)
                        .setControlType(RobotCycle.MotorCommand.ControlType.NONE)
                        .setCommand(0.0)
                        .setVoltage(voltage)
                        .build()
        );
    }

    public void setVelocitySetpoint(double velocityRadPerSec, double ffVolts) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addMotorCommands(
                RobotCycle.MotorCommand.newBuilder()
                        .setSlot(slot)
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
        builder.addMotorCommands(
                RobotCycle.MotorCommand.newBuilder()
                        .setSlot(slot)
                        .setControlType(RobotCycle.MotorCommand.ControlType.POSITION)
                        .setCommand(positionRadians)
                        .setVoltage(ffVolts)
                        .build()
        );
    }

    public void setKp(double kP) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setKP(kP)
                        .build()
        );
    }


    public void setKi(double kI) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setKI(kI)
                        .build()
        );
    }


    public void setKd(double kD) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setKD(kD)
                        .build()
        );
    }


    public void setKf(double kF) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setKF(kF)
                        .build()
        );
    }


    public void setIZone(double iZone) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setIZone(iZone)
                        .build()
        );
    }


    public void setPIDMinOutput(double minOutput) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setMinOutput(minOutput)
                        .build()
        );
    }


    public void setPIDMaxOutput(double maxOutput) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setMaxOutput(maxOutput)
                        .build()
        );
    }

    public void setPositionSetpoint(double positionRadians) {
        setPositionSetpoint(positionRadians, 0.0);
    }
}
