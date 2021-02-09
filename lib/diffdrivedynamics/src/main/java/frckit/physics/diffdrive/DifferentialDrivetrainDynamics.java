package frckit.physics.diffdrive;

import frckit.physics.DCMotorDynamics;
import frckit.util.Epsilon;

public class DifferentialDrivetrainDynamics {
    private final double mass; //kg
    private final double moi; //kg*m^2
    private final double angularDrag; //N*m/rad/s
    private final double wheelRadius; //m
    private final double effectiveWheelbaseRadius; //m
    private final DCMotorDynamics leftTransmission;
    private final DCMotorDynamics rightTransmission;
    /**
     * Creates a new differential drivetrain model from parameters and DC motor models
     *
     * @param massKg                         The mass of the vehicle in kg
     * @param moiKgM2                        The rotational moment of inertia of the vehicle in Kg * m^2
     * @param angularDrag                    The angular drag factor of the vehicle in N*m / rad/s
     *                                       This is a "fudge factor" which can be used to account for additional drag
     *                                       caused while turning due to wheel scrub.
     * @param wheelRadiusMeters              The radius of the vehicle's drive wheel, in meters
     * @param effectiveWheelbaseRadiusMeters The radius of the vehicle's empirical track width, in meters
     * @param leftTransmission               The left transmission's dynamics model
     * @param rightTransmission              The right transmission's dynamics model
     * @see DifferentialDrivetrainDynamics#fromHybridCharacterization
     * @see DifferentialDrivetrainDynamics#fromCharacterization
     */
    public DifferentialDrivetrainDynamics(
            double massKg,
            double moiKgM2,
            double angularDrag,
            double wheelRadiusMeters,
            double effectiveWheelbaseRadiusMeters,
            DCMotorDynamics leftTransmission,
            DCMotorDynamics rightTransmission
    ) {
        this.mass = massKg;
        this.moi = moiKgM2;
        this.angularDrag = angularDrag;
        this.wheelRadius = wheelRadiusMeters;
        this.effectiveWheelbaseRadius = effectiveWheelbaseRadiusMeters;
        this.leftTransmission = leftTransmission;
        this.rightTransmission = rightTransmission;
    }

    /**
     * Creates a new differential drivetrain model from parameters and characterization data.
     * <p>
     * This is referred to as "hybrid" as instead of providing Ka (the characterization acceleration constant),
     * "torquePerVolt" is provided.  This is a theoretical constant of the motor(s) used in the transmission,
     * and will usually yield better results since the measurement process for Ka usually does not accurately
     * represent the true characteristics of the system due to measurement derivative errors.
     * </p>
     * <p>
     * A motor's torque per volt can be found by examining the stall torque of the motor as provided by the manufacturer,
     * and dividing it by the voltage at which the stall test was performed (12 volts for all currently legal FRC motors)
     * </p>
     * <pre>
     * An example calculation for the REV NEO:
     *      Stall torque (from REV website): 2.6 N*m
     *      Voltage for test: 12 (this is just assumed, specified as "Nominal voltage" usually)
     *
     *      torquePerVolt = 2.6 / 12 ~= 0.2167
     * </pre>
     * <p>
     * In a multi motor gearbox, this constant can simply be multiplied by the number of motors in the gearbox.
     * </p>
     * <pre>
     * For example, a two motor NEO gearbox:
     *      torquePerVolt = 2 * 0.2167 = 0.4334
     * </pre>
     * <p>
     * Finally, after accounting for motor type and count, the gear ratio must be factored in.  Since torque increases
     * as gear ratio (output revolutions:input revolutions) increases, we multiply by the ratio to achieve the
     * final result (or divide if you express gear ratio as output revolutions:input revolutions).
     * <pre>
     * For example, a 2 NEO gearbox with a 100:1 reduction would have a final torquePerVolt of:
     *      torquePerVolt = 0.4334 * 100 = 43.34
     * </pre>
     *
     * @param massKg                         The mass of the vehicle in kg
     * @param moiKgM2                        The rotational moment of inertia of the vehicle in Kg * m^2
     * @param angularDrag                    The angular drag factor of the vehicle in N*m / rad/s
     *                                       This is a "fudge factor" which can be used to account for additional drag
     *                                       caused while turning due to wheel scrub.
     * @param wheelRadiusMeters              The radius of the vehicle's drive wheel, in meters
     * @param effectiveWheelbaseRadiusMeters The radius of the vehicle's empirical track width, in meters
     * @param leftKs                         The Ks constant for the left side of the drive
     * @param leftKv                         The Kv constant for the left side of the drive
     * @param leftTorquePerVolt              The torque per volt of the left side motors, in N*m / V
     * @param rightKs                        The Ks constant for the right side of the drive
     * @param rightKv                        The Kv constant for the right side of the drive
     * @param rightTorquePerVolt             The torque per volt of the right side motors, in N*m / V
     * @return The DifferentialDrivetrainDynamics model representing these parameters
     */
    public static DifferentialDrivetrainDynamics fromHybridCharacterization(
            double massKg,
            double moiKgM2,
            double angularDrag,
            double wheelRadiusMeters,
            double effectiveWheelbaseRadiusMeters,
            double leftKs,
            double leftKv,
            double leftTorquePerVolt,
            double rightKs,
            double rightKv,
            double rightTorquePerVolt
    ) {
        return new DifferentialDrivetrainDynamics(
                massKg,
                moiKgM2,
                angularDrag,
                wheelRadiusMeters,
                effectiveWheelbaseRadiusMeters,
                new DCMotorDynamics(1.0 / leftKv, leftTorquePerVolt, leftKs), //Kv is volts per speed, invert
                new DCMotorDynamics(1.0 / rightKv, rightTorquePerVolt, rightKs)
        );
    }

    /**
     * Creates a new differential drivetrain model from parameters and characterization data.
     * <p>
     * This can be used when characterization data (specifically the Ka terms) can be trusted.  This is not usually
     * the case for FRC drivetrain characterizations, so usually the "hybrid" method should be used.
     * </p>
     *
     * @param massKg                         The mass of the vehicle in kg
     * @param moiKgM2                        The rotational moment of inertia of the vehicle in Kg * m^2
     * @param angularDrag                    The angular drag factor of the vehicle in N*m / rad/s
     *                                       This is a "fudge factor" which can be used to account for additional drag
     *                                       caused while turning due to wheel scrub.
     * @param wheelRadiusMeters              The radius of the vehicle's drive wheel, in meters
     * @param effectiveWheelbaseRadiusMeters The radius of the vehicle's empirical track width, in meters
     * @param leftKs                         The Ks constant for the left side of the drive
     * @param leftKv                         The Kv constant for the left side of the drive
     * @param leftKa                         The Ka constant for the left side of the drive
     * @param rightKs                        The Ks constant for the right side of the drive
     * @param rightKv                        The Kv constant for the right side of the drive
     * @param rightKa                        The Ka constant for the right side of the drive
     * @return The DifferentialDrivetrainDynamics model representing these parameters
     * @see DifferentialDrivetrainDynamics#fromHybridCharacterization
     */
    public static DifferentialDrivetrainDynamics fromCharacterization(
            double massKg,
            double moiKgM2,
            double angularDrag,
            double wheelRadiusMeters,
            double effectiveWheelbaseRadiusMeters,
            double leftKs,
            double leftKv,
            double leftKa,
            double rightKs,
            double rightKv,
            double rightKa
    ) {

        return fromHybridCharacterization(
                massKg,
                moiKgM2,
                angularDrag,
                wheelRadiusMeters,
                effectiveWheelbaseRadiusMeters,
                leftKs,
                leftKv,
                //m*r^2 / 2*Ka
                wheelRadiusMeters * wheelRadiusMeters * massKg / (2.0 * leftKa),
                rightKs,
                rightKv,
                wheelRadiusMeters * wheelRadiusMeters * massKg / (2.0 * rightKa)
        );
    }

    /**
     * @return The DC motor model for the left side transmission
     */
    public DCMotorDynamics getLeftTransmission() {
        return leftTransmission;
    }

    /**
     * @return The DC motor model for the right side transmission
     */
    public DCMotorDynamics getRightTransmission() {
        return rightTransmission;
    }

    public ChassisState forwardKinematics(WheelState wheelMotion) {
        return new ChassisState(
                wheelRadius * (wheelMotion.right + wheelMotion.left) / 2.0,
                wheelRadius * (wheelMotion.right - wheelMotion.left) / (2.0 * effectiveWheelbaseRadius)
        );
    }

    public WheelState inverseKinematics(ChassisState chassisMotion) {
        return new WheelState(
                (chassisMotion.linear - effectiveWheelbaseRadius * chassisMotion.angular) / wheelRadius,
                (chassisMotion.linear + effectiveWheelbaseRadius * chassisMotion.angular) / wheelRadius
        );
    }

    // Solve for torques and accelerations.
    public DriveDynamics solveForwardDynamics(ChassisState chassisVelocity, WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheelVelocity = inverseKinematics(chassisVelocity);
        dynamics.chassisVelocity = chassisVelocity;
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveForwardDynamics(WheelState wheelVelocity, WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheelVelocity = wheelVelocity;
        dynamics.chassisVelocity = forwardKinematics(wheelVelocity);
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and voltages provided.
    public void solveForwardDynamics(DriveDynamics dynamics) {
        final boolean left_stationary = Epsilon.equals(dynamics.wheelVelocity.left, 0.0) && Math.abs(dynamics
                .voltage.left) < leftTransmission.getFrictionVoltage();
        final boolean right_stationary = Epsilon.equals(dynamics.wheelVelocity.right, 0.0) && Math.abs(dynamics
                .voltage.right) < rightTransmission.getFrictionVoltage();
        if (left_stationary && right_stationary) {
            // Neither side breaks static friction, so we remain stationary.
            dynamics.wheelTorque.left = dynamics.wheelTorque.right = 0.0;
            dynamics.chassisAcceleration.linear = dynamics.chassisAcceleration.angular = 0.0;
            dynamics.wheelAcceleration.left = dynamics.wheelAcceleration.right = 0.0;
            dynamics.dcurvature = 0.0;
            return;
        }

        // Solve for motor torques generated on each side.
        dynamics.wheelTorque.left = leftTransmission.forwardDynamics(dynamics.wheelVelocity.left, dynamics
                .voltage.left);
        dynamics.wheelTorque.right = rightTransmission.forwardDynamics(dynamics.wheelVelocity.right, dynamics
                .voltage.right);

        // Add forces and torques about the center of mass.
        dynamics.chassisAcceleration.linear = (dynamics.wheelTorque.right + dynamics.wheelTorque.left) /
                (wheelRadius * mass);
        // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
        dynamics.chassisAcceleration.angular = effectiveWheelbaseRadius * (dynamics.wheelTorque.right - dynamics
                .wheelTorque.left) / (wheelRadius * moi) - dynamics.chassisVelocity.angular * angularDrag / moi;

        // Solve for change in curvature from angular acceleration.
        // total angular accel = linear_accel * curvature + v^2 * dcurvature
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;

        // Resolve chassis accelerations to each wheel.
        dynamics.wheelAcceleration.left = dynamics.chassisAcceleration.linear - dynamics.chassisAcceleration
                .angular * effectiveWheelbaseRadius;
        dynamics.wheelAcceleration.right = dynamics.chassisAcceleration.linear + dynamics.chassisAcceleration
                .angular * effectiveWheelbaseRadius;
    }

    // Solve for torques and voltages.
    public DriveDynamics inverseDynamics(ChassisState chassisVelocity, ChassisState chassisAcceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassisVelocity = chassisVelocity;
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.chassisAcceleration = chassisAcceleration;
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
        dynamics.wheelVelocity = inverseKinematics(chassisVelocity);
        dynamics.wheelAcceleration = inverseKinematics(chassisAcceleration);
        inverseDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics inverseDynamics(WheelState wheelVelocity, WheelState wheelAcceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassisVelocity = forwardKinematics(wheelVelocity);
        dynamics.curvature = dynamics.chassisVelocity.angular / dynamics.chassisVelocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.chassisAcceleration = forwardKinematics(wheelAcceleration);
        dynamics.dcurvature = (dynamics.chassisAcceleration.angular - dynamics.chassisAcceleration.linear * dynamics.curvature) /
                (dynamics.chassisVelocity.linear * dynamics.chassisVelocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
        dynamics.wheelVelocity = wheelVelocity;
        dynamics.wheelAcceleration = wheelAcceleration;
        inverseDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and accelerations provided, curvature and dcurvature computed.
    public void inverseDynamics(DriveDynamics dynamics) {
        // Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        dynamics.wheelTorque.left = wheelRadius / 2.0 * (dynamics.chassisAcceleration.linear * mass -
                dynamics.chassisAcceleration.angular * moi / effectiveWheelbaseRadius -
                dynamics.chassisVelocity.angular * angularDrag / effectiveWheelbaseRadius);
        dynamics.wheelTorque.right = wheelRadius / 2.0 * (dynamics.chassisAcceleration.linear * mass +
                dynamics.chassisAcceleration.angular * moi / effectiveWheelbaseRadius +
                dynamics.chassisVelocity.angular * angularDrag / effectiveWheelbaseRadius);

        // Solve for input voltages.
        dynamics.voltage.left = leftTransmission.inverseDynamics(dynamics.wheelVelocity.left, dynamics
                .wheelTorque.left);
        dynamics.voltage.right = rightTransmission.inverseDynamics(dynamics.wheelVelocity.right, dynamics
                .wheelTorque.right);
    }

    /**
     * Represents the state of the chassis as a whole.
     * This can be used to represent either velocity or acceleration of the chassis.
     */
    public static class ChassisState {
        private double linear;
        private double angular;

        public ChassisState() {
        }

        public ChassisState(double linear, double angular) {
            this.linear = linear;
            this.angular = angular;
        }

        public double getLinear() {
            return linear;
        }

        public double getAngular() {
            return angular;
        }
    }

    /**
     * Represents the state of the wheels of a differential drivetrain.
     * This can be used to represent velocity, acceleration, torque, voltage, etc. (context dependent)
     */
    public static class WheelState {
        private double left;
        private double right;

        public WheelState() {
        }

        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return left;
        }

        public double getRight() {
            return right;
        }
    }

    /**
     * Represents the full state dynamics of a differential drivetrain
     */
    public static class DriveDynamics {
        private double curvature = 0.0;  // m^-1
        private double dcurvature = 0.0;  // m^-1/m
        private ChassisState chassisVelocity = new ChassisState();  // m/s
        private ChassisState chassisAcceleration = new ChassisState();  // m/s^2
        private WheelState wheelVelocity = new WheelState();  // rad/s
        private WheelState wheelAcceleration = new WheelState();  // rad/s^2
        private WheelState voltage = new WheelState();  // V
        private final WheelState wheelTorque = new WheelState();  // N m
    }
}
