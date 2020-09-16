package frckit.simulation.devices;


import frckit.simulation.SimulationClient;
import frckit.simulation.protocol.RobotCycle;

public class SimStandaloneEncoder {
    private final int slot;

    public SimStandaloneEncoder(int slot) {
        this.slot = slot;
    }

    public double getPositionRadians() {
        return SimulationClient.getInstance().getLastWorldUpdate().getStandaloneEncoderStatesOrThrow(slot).getPosition();
    }

    public double getVelocityRadPerSec() {
        return SimulationClient.getInstance().getLastWorldUpdate().getStandaloneEncoderStatesOrThrow(slot).getVelocity();
    }

    public void setPositionRadians(double position) {
        //TODO finish this (not in protocol yet)
    }
}
