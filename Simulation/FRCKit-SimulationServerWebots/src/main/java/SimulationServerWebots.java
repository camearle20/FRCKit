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
        RobotCycle.MotorCommand DEFAULT_COMMAND = RobotCycle.MotorCommand.newBuilder()
                .setVoltage(0.0)
                .setControlType(RobotCycle.MotorCommand.ControlType.NONE)
                .setCommand(0.0)
                .build();

        ModelConfiguration config = ConfigurationLoader.load();

        SimulationServer server = new SimulationServer(config.serverPort);
        Supervisor sv = new Supervisor();
        int timestepMs = (int) Math.round(sv.getBasicTimeStep());

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
            WorldUpdate.WorldUpdateMessage.Builder builder = WorldUpdate.WorldUpdateMessage.newBuilder();
            double timestamp = sv.getTime();
            builder.setTimestamp(timestamp);
            //Start by reading all sensors
            for (Map.Entry<Integer, WebotsTransmission> entry : transmissionsMap.entrySet()) {
                int slot = entry.getKey();
                WebotsTransmission transmission = entry.getValue();

                transmission.updateSensors(timestamp);
                builder.putTransmissionEncoderStates(slot,
                        WorldUpdate.EncoderState.newBuilder()
                                .setPosition(transmission.encoder.getPositionWithResolution())
                                .setVelocity(transmission.encoder.getVelocity())
                                .build()
                );
            }

            server.sendWorldUpdate(builder.build());
            RobotCycle.RobotCycleMessage robotCycleMessage = server.getRobotCycleMessage();
            if (robotCycleMessage.getResetWorldFlag()) {
                //Reset now
                System.out.println("Resetting simulation");
                sv.simulationReset();
                continue; //Break this loop cycle now
            }

            //Send torques to transmissions
            for (Map.Entry<Integer, WebotsTransmission> entry: transmissionsMap.entrySet()) {
                int slot = entry.getKey();
                WebotsTransmission transmission = entry.getValue();

                RobotCycle.MotorCommand message = robotCycleMessage.getMotorCommandsOrDefault(slot, DEFAULT_COMMAND);
                double voltage = message.getVoltage();
                transmission.applyTorque(voltage);
            }
        }
        server.stop();
    }
}
