package frckit.simulation.devices;

import frckit.simulation.SimulationClient;

public class SimTransmissionEncoder {
    private int slot;

    public SimTransmissionEncoder(int slot) {
        this.slot = slot;
    }

    public double getPositionRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getTransmissionEncoderStatesOrThrow(slot).getPosition();
    }

    public double getVelocityRadPerSec() {
        return SimulationClient.getInstance().getLastWorldUpdate().getTransmissionEncoderStatesOrThrow(slot).getVelocity();
    }
}
