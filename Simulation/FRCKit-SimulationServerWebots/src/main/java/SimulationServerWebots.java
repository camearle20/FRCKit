import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Supervisor;
import frckit.simulation.SimulationServer;
import frckit.simulation.modelconfig.ConfigurationLoader;
import frckit.simulation.modelconfig.ModelConfiguration;
import frckit.simulation.modelconfig.TransmissionConfig;
import frckit.simulation.protocol.RobotCycle;
import frckit.simulation.protocol.WorldUpdate;
import frckit.simulation.webots.WebotsTransmission;

import java.io.IOException;
import java.util.*;

public class SimulationServerWebots {
    public static void main(String[] args) throws IOException, InterruptedException {
        int resetCount = 0;
        boolean firstRun = true;
        boolean enabled = false;
        long lastSend = 0;
        ModelConfiguration config = ConfigurationLoader.load();

        SimulationServer server = new SimulationServer(config.serverPort);
        Supervisor sv = new Supervisor();
        int timestepMs = (int) Math.round(sv.getBasicTimeStep());
        int clientTimestepMs = config.robotCodeDtMs;

        if (clientTimestepMs < timestepMs) {
            System.err.println("WARNING: Configured robotCodeDtMs of " + clientTimestepMs + " ms is less than world basicTimeStep of " + timestepMs + ".  basicTimeStep will be used.");
        } else if (clientTimestepMs % timestepMs != 0) {
            System.err.println("WARNING: Configured robotCodeDtMs of " + clientTimestepMs + " ms does not align with world basicTimeStep of " + timestepMs + ".  basicTimeStep will be used.");
        }

        HashMap<Integer, WebotsTransmission> transmissionsMap = new HashMap<>();

        //Begin looking for components from the config
        for (TransmissionConfig transmissionConfig : config.transmissions) {
            int slot = transmissionConfig.slot;
            Motor m = sv.getMotor("motor" + slot);
            PositionSensor s = sv.getPositionSensor("pos" + slot);
            m.setTorque(0.0); //Disable motor's internal PID
            s.enable(timestepMs); //Enable sensor to match robot's timestep
            transmissionsMap.put(slot, new WebotsTransmission(m, s, transmissionConfig));
        }

        //Main simulation loop
        while (sv.step(timestepMs) != -1) {
            if (resetCount > 0) {
                resetCount--;
                continue;
            }
            double timestamp = sv.getTime();
            boolean sendCycle = clientTimestepMs <= timestepMs || Math.round(timestamp * 1000) - lastSend >= clientTimestepMs;
            WorldUpdate.WorldUpdateMessage.Builder builder = WorldUpdate.WorldUpdateMessage.newBuilder();
            builder.setTimestamp(timestamp);
            //Start by reading all sensors
            for (Map.Entry<Integer, WebotsTransmission> entry : transmissionsMap.entrySet()) {
                int slot = entry.getKey();
                WebotsTransmission transmission = entry.getValue();

                transmission.updateSensors(timestamp);
                builder.putTransmissionEncoderStates(slot,
                        WorldUpdate.EncoderState.newBuilder()
                                .setPosition(transmission.encoder.getPositionWithResolutionAndOffset())
                                .setVelocity(transmission.encoder.getVelocity())
                                .build()
                );
            }

            if (sendCycle) {
                if (firstRun) {
                    builder.setSimulatorName(sv.getName()); //Send name to client only on first cycle
                    firstRun = false;
                }
                lastSend = Math.round(timestamp * 1000);
                server.sendWorldUpdate(builder.build());
                RobotCycle.RobotCycleMessage robotCycleMessage = server.getRobotCycleMessage();
                if (robotCycleMessage.getResetWorldFlag()) {
                    //Reset now
                    System.out.println("Resetting simulation");
                    for (WebotsTransmission transmission : transmissionsMap.values()) {
                        transmission.updateCommand(null);
                        transmission.encoder.reset();
                    }
                    sv.simulationResetPhysics();
                    sv.simulationReset();
                    resetCount = 1;
                    firstRun = true;
                    lastSend = 0;
                    continue; //Break this loop cycle now
                }

                //Read commands in
                for (RobotCycle.MotorCommand motorCommand : robotCycleMessage.getMotorCommandsList()) {
                    WebotsTransmission transmission = transmissionsMap.get(motorCommand.getSlot());
                    if (transmission != null) {
                        transmission.updateCommand(motorCommand);
                    } else {
                        System.out.println("WARNING: No transmission with slot #" + motorCommand.getSlot() + " for motor command.  Ignoring command.");
                    }
                }

                //TODO pneumatics

                for (RobotCycle.PIDConfigCommand pidConfigCommand : robotCycleMessage.getPidConfigCommandsList()) {
                    WebotsTransmission transmission = transmissionsMap.get(pidConfigCommand.getSlot());
                    if (transmission != null) {
                        transmission.processPidConfigCommand(pidConfigCommand);
                    } else {
                        System.out.println("WARNING: No transmission with slot #" + pidConfigCommand.getSlot() + " for PID config command.  Ignoring command.");
                    }
                }
                enabled = robotCycleMessage.getIsEnabled();
            }

            //Update transmissions
            boolean enabledCopy = enabled;
            transmissionsMap.values().forEach((t) -> t.updateMotor(enabledCopy));
        }
        server.stop();
    }
}
