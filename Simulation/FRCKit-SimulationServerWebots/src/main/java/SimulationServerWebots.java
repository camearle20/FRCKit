import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Supervisor;
import frckit.simulation.SimulationServer;
import frckit.simulation.modelconfig.ConfigurationLoader;
import frckit.simulation.modelconfig.ModelConfiguration;
import frckit.simulation.modelconfig.TransmissionConfig;
import frckit.simulation.protocol.RobotCycleMessage;
import frckit.simulation.protocol.WorldUpdateMessage;
import frckit.simulation.webots.WebotsMotorUnion;

import java.io.IOException;
import java.util.*;

public class SimulationServerWebots {

    public static void main(String[] args) throws IOException, InterruptedException {
        ModelConfiguration config = ConfigurationLoader.load();

        SimulationServer server = new SimulationServer(config.serverPort);
        Supervisor sv = new Supervisor();
        int timestepMs = (int) Math.round(sv.getBasicTimeStep());

        HashMap<Integer, WebotsMotorUnion> motorsMap = new HashMap<>();

        //Begin looking for components from the config
        for (TransmissionConfig transmissionConfig : config.transmissions) {
            int slot = transmissionConfig.slot;
            Motor m = sv.getMotor("motor" + slot);
            PositionSensor s = sv.getPositionSensor("pos" + slot);
            m.setTorque(0.0); //Disable motor's internal PID
            s.enable(timestepMs); //Enable sensor to match robot's timestep
            motorsMap.put(slot, new WebotsMotorUnion(m, s, transmissionConfig));
        }

        int numMotors = Collections.max(motorsMap.keySet()) + 1; //Size of motors array is idx of last motor + 1 (sparse array possible)

        //We will reuse the same instance of WorldUpdateMessage
        WorldUpdateMessage worldUpdateMessage = new WorldUpdateMessage(numMotors, 0); //TODO standalone encoder

        WebotsMotorUnion[] motors = new WebotsMotorUnion[numMotors]; //Load motors map into a (possibly) sparse array
        for (int i = 0; i < numMotors; i++) {
            WebotsMotorUnion motor = motorsMap.get(i);
            motors[i] = motor;
        }

        //Main simulation loop
        while (sv.step(timestepMs) != -1) {
            double timestamp = sv.getTime();
            worldUpdateMessage.timestamp = timestamp;
            //Start by reading all sensors
            for (int i = 0; i < motors.length; i++) {
                WebotsMotorUnion motor = motors[i];
                if (motor != null) {
                    motor.updateSensors(timestamp);
                    worldUpdateMessage.transmission_positions[i] = motor.getPosition();
                    worldUpdateMessage.transmission_velocities[i] = motor.getVelocity();
                }
            }

            server.sendWorldUpdate(worldUpdateMessage);
            RobotCycleMessage robotCycleMessage = server.getRobotCycleMessage();
            if (robotCycleMessage.resetWorldFlag) {
                //Reset now
                System.out.println("Resetting simulation");
                sv.simulationReset();
                continue;
            }

            //Send torques to motors
            for (int i = 0; i < motors.length; i++) {
                WebotsMotorUnion motor = motors[i];
                if (motor != null && i < robotCycleMessage.motors_setpoints.length) { //Check that robot code has defined motor of correct idx
                    double voltage = robotCycleMessage.motors_voltages[i];            //if not, ignore silently as this is not a problem (code for that motor could just not be implemented in user code yet)
                    motor.applyTorque(voltage);
                }
            }
        }
        server.stop();
    }
}
