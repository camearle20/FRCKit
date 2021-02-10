package frckit.physics.drivetrain;

public class DifferentialWheelState {
    public static final DifferentialWheelState ZERO = new DifferentialWheelState();

    protected final double leftVelocity;
    protected final double rightVelocity;
    protected final double leftAcceleration;
    protected final double rightAcceleration;

    protected DifferentialWheelState() {
        leftVelocity = 0.0;
        rightVelocity = 0.0;
        leftAcceleration = 0.0;
        rightAcceleration = 0.0;
    }

    public DifferentialWheelState(double leftVelocity, double rightVelocity, double leftAcceleration, double rightAcceleration) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
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
