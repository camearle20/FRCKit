package frckit.simulation.modelconfig;

/**
 * Full configuration for a simulation model.
 */
public class ModelConfiguration {
    public int robotCodeDtMs = 10;
    public int serverPort = 8889;
    public TransmissionConfig[] transmissions;
    public SensorConfig[] extraSensors;
}
