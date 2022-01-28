package frckit.physics.drivetrain.follower;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frckit.physics.state.RigidBodyState2d;
import frckit.util.GeomUtil;

public class DrivetrainTrajectoryFollower {
    private final Trajectory trajectory;
    private final DrivetrainFeedback feedback;

    private double startTimestamp = -1.0;
    private double lastTimestamp = -1.0;

    private RigidBodyState2d setpoint = RigidBodyState2d.ZERO;
    private boolean done = false;
    private RigidBodyState2d targetState = RigidBodyState2d.ZERO;

    private Pose2d error = GeomUtil.POSE_ZERO;

    public DrivetrainTrajectoryFollower(Trajectory trajectory, DrivetrainFeedback feedback) {
        this.trajectory = trajectory;
        this.feedback = feedback;
    }

    public RigidBodyState2d getSetpoint() {
        return setpoint;
    }

    public boolean isDone() {
        return done;
    }

    public Pose2d getError() {
        return error;
    }

    public RigidBodyState2d update(double timestamp, Pose2d currentPoseMeters) {
        if (lastTimestamp < 0.0 || startTimestamp < 0.0) {
            //This is the first run, set last timestamp and return a zero output
            lastTimestamp = timestamp;
            startTimestamp = timestamp;
            return RigidBodyState2d.ZERO;
        }


        //Calculate dt and update lastTimestamp (so that next cycle can calculate dt)
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double trajectoryTime = timestamp - startTimestamp;
        Trajectory.State trajectoryState = trajectory.sample(trajectoryTime);

        if (!done) {
            setpoint = new RigidBodyState2d( //TODO this does not do anything with acceleration or y velocity.  This will NOT work with swerve!
                    trajectoryState.velocityMetersPerSecond,
                    0.0,
                    trajectoryState.velocityMetersPerSecond * trajectoryState.curvatureRadPerMeter,
                    0.0, 0.0, 0.0
            );

            error = trajectoryState.poseMeters.relativeTo(currentPoseMeters);

            //Do feedback
            targetState = feedback.calculate(dt, error, setpoint, targetState); //Pass back the previous targetState

            //Check if we are done
            done = trajectoryTime >= trajectory.getTotalTimeSeconds();

            return targetState;
        } else {
            return RigidBodyState2d.ZERO;
        }
    }
}
