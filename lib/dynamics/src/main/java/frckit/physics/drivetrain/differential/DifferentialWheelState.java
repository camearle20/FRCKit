package frckit.physics.drivetrain.differential;

public class DifferentialWheelState {
    public static final DifferentialWheelState ZERO = new DifferentialWheelState();

    private final double leftVelocity;
    private final double rightVelocity;
    private final double leftAcceleration;
    private final double rightAcceleration;
    private final double leftVoltage;
    private final double rightVoltage;


    private DifferentialWheelState() {
        leftVelocity = 0.0;
        rightVelocity = 0.0;
        leftAcceleration = 0.0;
        rightAcceleration = 0.0;
        leftVoltage = 0.0;
        rightVoltage = 0.0;
    }

    public DifferentialWheelState(double leftVelocity, double rightVelocity, double leftAcceleration, double rightAcceleration, double leftVoltage, double rightVoltage) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getLeftAcceleration() {
        return leftAcceleration;
    }

    public double getRightAcceleration() {
        return rightAcceleration;
    }

    public double getLeftVoltage() {
        return leftVoltage;
    }

    public double getRightVoltage() {
        return rightVoltage;
    }

    public double getLeftLinearVelocity(double wheelRadius) {
        return wheelRadius * leftVelocity;
    }

    public double getRightLinearVelocity(double wheelRadius) {
        return wheelRadius * rightVelocity;
    }

    public double getLeftLinearAcceleration(double wheelRadius) {
        return wheelRadius * leftAcceleration;
    }

    public double getRightLinearAcceleration(double wheelRadius) {
        return wheelRadius * rightAcceleration;
    }
}
