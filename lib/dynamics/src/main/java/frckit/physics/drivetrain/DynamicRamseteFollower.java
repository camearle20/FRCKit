package frckit.physics.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frckit.physics.RigidBodyState;
import frckit.util.Epsilon;
import frckit.util.GeomUtil;

public class DynamicRamseteFollower {
    private final Trajectory trajectory;
    private final DifferentialDrivetrainDynamics model;
    private final boolean isInches;

    private double startTimestamp = -1.0;
    private double lastTimestamp = -1.0;

    private double kBeta = 2.0;
    private double kZeta = 0.7;

    private Trajectory.State setpoint = new Trajectory.State();
    private boolean done = false;
    private Twist2d prevVelocity = GeomUtil.IDENTITY_TWIST;

    private Pose2d error = GeomUtil.IDENTITY_POSE;

    public DynamicRamseteFollower(Trajectory trajectory, DifferentialDrivetrainDynamics model, double kBeta, double kZeta, boolean isInches) {
        this.trajectory = trajectory;
        this.model = model;
        this.isInches = isInches;
        this.kBeta = kBeta;
        this.kZeta = kZeta;
    }

    public DynamicRamseteFollower(Trajectory trajectory, DifferentialDrivetrainDynamics model, boolean isInches) {
        this.trajectory = trajectory;
        this.model = model;
        this.isInches = isInches;
    }

    public DynamicRamseteFollower(Trajectory trajectory, DifferentialDrivetrainDynamics model, double kBeta, double kZeta) {
        this.trajectory = trajectory;
        this.model = model;
        this.kBeta = kBeta;
        this.kZeta = kZeta;
        this.isInches = false;
    }

    public DynamicRamseteFollower(Trajectory trajectory, DifferentialDrivetrainDynamics model) {
        this(trajectory, model, false);
    }

    public Pose2d getError() {
        if (isInches) {
            return GeomUtil.metersToInches(error);
        } else {
            return error;
        }
    }

    public Trajectory.State getSetpoint() {
        return setpoint;
    }

    public boolean isDone() {
        return done;
    }

    public void reset() {
        startTimestamp = -1.0;
        lastTimestamp = -1.0;
        done = false;
        prevVelocity = GeomUtil.IDENTITY_TWIST;
        error = GeomUtil.IDENTITY_POSE;
        setpoint = trajectory.sample(0.0);
    }

    public DifferentialWheelCommand update(double timestamp, Pose2d currentPose) {
        if (isInches) {
            currentPose = GeomUtil.inchesToMeters(currentPose); //Convert currentPose to meters, since this class works in SI
        }

        if (lastTimestamp < 0.0 || startTimestamp < 0.0) {
            //This is the first run, set last timestamp and return a zero output
            lastTimestamp = timestamp;
            startTimestamp = timestamp;
            return DifferentialWheelCommand.ZERO;
        }

        //Calculate dt and update lastTimestamp (so that next cycle can calculate dt)
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double trajectoryTime = timestamp - startTimestamp;

        setpoint = trajectory.sample(trajectoryTime); //Sample the trajectory

        if (!done) {
            double linearVelocity, curvature;
            Pose2d setpointPose;
            if (isInches) {
                linearVelocity = Units.inchesToMeters(setpoint.velocityMetersPerSecond);
                curvature = Units.metersToInches(setpoint.curvatureRadPerMeter); //This is not a mistake, since curvature is in^-1 we need to use the inverse conversion function
                setpointPose = GeomUtil.inchesToMeters(setpoint.poseMeters);
            } else {
                linearVelocity = setpoint.velocityMetersPerSecond;
                curvature = setpoint.curvatureRadPerMeter;
                setpointPose = setpoint.poseMeters;
            }
            double angularVelocity = linearVelocity * curvature;

            error = setpointPose.relativeTo(currentPose);

            //Ramsete algorithm
            double k = 2.0 * kZeta * Math.sqrt(kBeta * linearVelocity * linearVelocity + angularVelocity * angularVelocity);
            double angleError = error.getRotation().getRadians();
            double sinc = Epsilon.equals(angleError, 0.0, 1e-9) ? 1.0 : error.getRotation().getSin() / angleError; //sin(theta) / theta = sinc(theta)

            Twist2d adjustedVelocity = new Twist2d(
                    linearVelocity * error.getRotation().getCos() + k * error.getTranslation().getX(),
                    0.0,
                    angularVelocity + k * angleError + linearVelocity * kBeta * sinc * error.getTranslation().getY()
            );

            //Compute acceleration from previous velocity (not the best method but it works well enough for this)
            Twist2d acceleration = new Twist2d(
                    Epsilon.equals(dt, 0.0) ? 0.0 : (adjustedVelocity.dx - prevVelocity.dx) / dt,
                    0.0,
                    Epsilon.equals(dt, 0.0) ? 0.0 : (adjustedVelocity.dtheta - prevVelocity.dtheta) / dt
            );

            prevVelocity = adjustedVelocity; //Store velocity for next acceleration calculation

            //Check if we are done
            done = trajectoryTime >= trajectory.getTotalTimeSeconds();

            //Solve dynamics
            return model.inverseDynamics(new RigidBodyState(adjustedVelocity, acceleration));
        } else {
            return DifferentialWheelCommand.ZERO; //Done following, return zero output
        }
    }
}
