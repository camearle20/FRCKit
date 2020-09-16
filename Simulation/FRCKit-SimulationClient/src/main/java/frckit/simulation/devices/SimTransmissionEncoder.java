package frckit.simulation.devices;

import frckit.simulation.SimulationClient;
import frckit.simulation.protocol.RobotCycle;

public class SimTransmissionEncoder {
    private final int slot;

    public SimTransmissionEncoder(int slot) {
        this.slot = slot;
    }

    public double getPositionRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getTransmissionEncoderStatesOrThrow(slot).getPosition();
    }

    public double getVelocityRadPerSec() {
        return SimulationClient.getInstance().getLastWorldUpdate().getTransmissionEncoderStatesOrThrow(slot).getVelocity();
    }

    public void setPositionRadians(double position) {
        SimulationClient.getInstance().getCurrentCycleBuilder().addPidConfigCommands(
                RobotCycle.PIDConfigCommand.newBuilder()
                        .setSlot(slot)
                        .setEncoderPosition(position)
                        .build()
        );
    }
}
