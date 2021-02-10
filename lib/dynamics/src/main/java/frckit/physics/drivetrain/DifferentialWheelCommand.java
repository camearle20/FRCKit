package frckit.physics.drivetrain;

public class DifferentialWheelCommand extends DifferentialWheelState {
    public static final DifferentialWheelCommand ZERO = new DifferentialWheelCommand();

    private final double leftVoltage;
    private final double rightVoltage;

    public DifferentialWheelCommand() {
        super();
        leftVoltage = 0.0;
        rightVoltage = 0.0;
    }

    public DifferentialWheelCommand(double leftVoltage, double rightVoltage, double leftVelocity, double rightVelocity, double leftAcceleration, double rightAcceleration) {
        super(leftVelocity, rightVelocity, leftAcceleration, rightAcceleration);
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    public static DifferentialWheelCommand fromState(DifferentialWheelState state, double leftVoltage, double rightVoltage) {
        return new DifferentialWheelCommand(
                leftVoltage,
                rightVoltage,
                state.leftVelocity,
                state.rightVelocity,
                state.leftAcceleration,
                state.rightAcceleration
        );
    }
}