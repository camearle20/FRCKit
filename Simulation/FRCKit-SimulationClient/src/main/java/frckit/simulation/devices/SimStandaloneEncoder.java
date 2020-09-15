package frckit.simulation.devices;


import frckit.simulation.SimulationClient;

public class SimStandaloneEncoder {
    private int slot;

    public SimStandaloneEncoder(int slot) {
        this.slot = slot;
    }

    public double getPositionRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getStandaloneEncoderStatesOrThrow(slot).getPosition();
    }

    public double getVelocityRadPerSec() {
        return SimulationClient.getInstance().getLastWorldUpdate().getStandaloneEncoderStatesOrThrow(slot).getVelocity();
    }
}
