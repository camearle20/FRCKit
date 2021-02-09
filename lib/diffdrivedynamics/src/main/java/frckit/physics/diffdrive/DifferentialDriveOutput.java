package frckit.physics.diffdrive;

public class DifferentialDriveOutput {
    public static final DifferentialDriveOutput ZERO = new DifferentialDriveOutput();

    private final double leftVoltage;
    private final double rightVoltage;
    private final double leftVelocity;
    private final double rightVelocity;
    private final double leftAcceleration;
    private final double rightAcceleration;

    private DifferentialDriveOutput() {
        leftVoltage = 0.0;
        rightVoltage = 0.0;
        leftVelocity = 0.0;
        rightVelocity = 0.0;
        leftAcceleration = 0.0;
        rightAcceleration = 0.0;
    }

    public DifferentialDriveOutput(double leftVoltage, double rightVoltage, double leftVelocity, double rightVelocity, double leftAcceleration, double rightAcceleration) {
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
    }

    public double getLeftVoltage() {
        return leftVoltage;
    }

    public double getRightVoltage() {
        return rightVoltage;
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
}
