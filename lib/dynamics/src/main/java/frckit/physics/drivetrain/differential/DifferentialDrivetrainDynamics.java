package frckit.physics.drivetrain.differential;

import frckit.physics.DCMotorDynamics;
import frckit.physics.state.RigidBodyState2d;

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

    /**
     * Performs inverse dynamics, which given desired chassis speed and acceleration (linear and angular), sovles for
     * the wheel velocities, accelerations, and voltages required to produce the motion.
     * @param chassisState The desired chassis state
     * @return The command which can be applied to the drive transmissions
     */
    public DifferentialWheelState inverseDynamics(RigidBodyState2d chassisState) {
        //Unpack data
        double v = chassisState.getVx();
        double omega = chassisState.getOmega();
        double a = chassisState.getAx();
        double alpha = chassisState.getAlpha();

        //Do inverse kinematics
        double leftWheelOmega = (v - effectiveWheelbaseRadius * omega) / wheelRadius;
        double rightWheelOmega = (v + effectiveWheelbaseRadius * omega) / wheelRadius;
        double leftWheelAlpha = (a - effectiveWheelbaseRadius * alpha) / wheelRadius;
        double rightWheelAlpha = (a + effectiveWheelbaseRadius * alpha) / wheelRadius;

        //Compute required wheel torques
        double leftWheelTorque = wheelRadius / 2.0 * (a * mass -
                alpha * moi / effectiveWheelbaseRadius -
                omega * angularDrag / effectiveWheelbaseRadius);
        double rightWheelTorque = wheelRadius / 2.0 * (a * mass +
                alpha * moi / effectiveWheelbaseRadius +
                omega * angularDrag / effectiveWheelbaseRadius);

        //Solve for required voltages
        double leftVoltage = leftTransmission.inverseDynamics(leftWheelOmega, leftWheelTorque);
        double rightVoltage = rightTransmission.inverseDynamics(rightWheelOmega, rightWheelTorque);

        return new DifferentialWheelState(
                leftWheelOmega,
                rightWheelOmega,
                leftWheelAlpha,
                rightWheelAlpha,
                leftVoltage,
                rightVoltage
        );
    }
}
