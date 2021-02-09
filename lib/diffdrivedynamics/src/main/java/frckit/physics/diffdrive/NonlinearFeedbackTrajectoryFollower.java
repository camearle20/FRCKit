package frckit.physics.diffdrive;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frckit.util.GeomUtil;

public class NonlinearFeedbackTrajectoryFollower {
    private final Trajectory trajectory;
    private final DifferentialDrivetrainDynamics model;
    private final boolean isInches;

    private double startTimestamp = -1.0;
    private double lastTimestamp = -1.0;

    private Trajectory.State setpoint = new Trajectory.State();
    private boolean done = false;

    public DifferentialDriveOutput update(double timestamp, Pose2d currentPose) {
        if (isInches) {
            currentPose = GeomUtil.inchesToMeters(currentPose); //Convert currentPose to meters, since this class works in SI
        }

        if (lastTimestamp < 0.0 || startTimestamp < 0.0) {
            //This is the first run, set last timestamp and return a zero output
            lastTimestamp = timestamp;
            startTimestamp = timestamp;
            return DifferentialDriveOutput.ZERO;
        }

        //Calculate dt and update lastTimestamp (so that next cycle can calculate dt)
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double trajectoryTime = timestamp - startTimestamp;

        Trajectory.State sample = trajectory.sample(trajectoryTime);

        if (!done) {
            double velocity, curvature, dcurvatureDs, acceleration;

            if (isInches) {

            } else {
                velocity = setpoint.velocityMetersPerSecond;
                curvature = Units.metersToInches(setpoint.curvatureRadPerMeter)
            }

            // Generate feedforward voltages.
            final double velocityM = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
            final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state()
                    .getDCurvatureDs()));
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrivetrainDynamics.DriveDynamics dynamics = fullStateModel.getDriveDynamicsModel().solveInverseDynamics(
                    new DifferentialDrivetrainDynamics.ChassisState(velocity_m, velocity_m * curvature_m),
                    new DifferentialDrivetrainDynamics.ChassisState(acceleration_m,
                            acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

            mOutput = controller.update(dynamics, current_state, mError, fullStateModel.getDriveDynamicsModel(), mDt);
        }
    }
}
