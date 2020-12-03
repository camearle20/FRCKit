package frckit.simulation.webots;

import com.cyberbotics.webots.controller.InertialUnit;
import frckit.simulation.protocol.RobotCycle;

public class WebotsInertialUnit {
    private final InertialUnit inertialUnit;

    private double pitch;
    private double roll;
    private double yaw;
    private double lastYaw;

    private int yawRev;

    private double yawOffset;

    public void setNewYaw(double newYaw) {
        yawOffset = newYaw - yaw;
    }

    public double getYawWithOffset() {
        return yaw + yawOffset;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }

    public WebotsInertialUnit(InertialUnit inertialUnit) {
        this.inertialUnit = inertialUnit;
    }
    
    public void reset() {

        pitch = 0.0;
        roll = 0.0;
        yaw = 0.0;
        lastYaw = 0.0;

        yawRev = 0;

        yawOffset = 0.0;
    }

    public void update() {
        double[] rpy = inertialUnit.getRollPitchYaw();

        double yawDiff = rpy[2] - lastYaw;

        //Count rotations
        if (yawDiff < -Math.PI) {
            yawRev++;
        } else if (yawDiff > Math.PI) {
            yawRev--;
        }

        yaw = rpy[2] + (2.0 * Math.PI * yawRev);
        lastYaw = rpy[2];

        pitch = rpy[1];
        roll = rpy[0];
    }

    public void processCommand(RobotCycle.InertialCommand command) {
        switch (command.getSettingCase()) {
            case YAWPOSITION:
                setNewYaw(command.getYawPosition());
                break;
        }
    }
}
