package frckit.simulation.devices;

import frckit.simulation.SimulationClient;
import frckit.simulation.protocol.RobotCycle;

public class SimIMU {
    private final int slot;

    public SimIMU(int slot) {
        this.slot = slot;
    }

    public void setYaw(double yaw) {
        RobotCycle.RobotCycleMessage.Builder builder = SimulationClient.getInstance().getCurrentCycleBuilder();
        builder.addInertialCommands(
                RobotCycle.InertialCommand.newBuilder()
                        .setSlot(slot)
                        .setYawPosition(yaw)
                        .build()
        );
    }

    public double getYawRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getInertialStatesOrThrow(slot).getYaw();
    }


    public double getPitchRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getInertialStatesOrThrow(slot).getPitch();
    }

    public double getRollRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getInertialStatesOrThrow(slot).getRoll();
    }
}
