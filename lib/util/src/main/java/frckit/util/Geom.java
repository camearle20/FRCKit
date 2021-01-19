package frckit.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Geometry utils implementing traditional (254 style) operations on WPILib geometry classes
 */
public class Geom {
    public static final Pose2d POSE_I = new Pose2d();
    public static final Transform2d XFORM_I = new Transform2d();
    public static final Translation2d TRANSLATION_I = new Translation2d();
    public static final Rotation2d ROT_I = new Rotation2d();

    public static Rotation2d inverse(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    public static Translation2d inverse(Translation2d translation) {
        return translation.unaryMinus();
    }

    public static Transform2d inverse(Transform2d transform) {
        Rotation2d rotationInverted = inverse(transform.getRotation());
        return new Transform2d(inverse(transform.getTranslation()).rotateBy(rotationInverted), rotationInverted);
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverted = inverse(pose.getRotation());
        return new Pose2d(inverse(pose.getTranslation()).rotateBy(rotationInverted), rotationInverted);
    }

    public static Pose2d poseTransform(Pose2d lhs, Pose2d rhs) {
        return new Pose2d(lhs.getTranslation().plus(rhs.getTranslation().rotateBy(lhs.getRotation())),
                lhs.getRotation().rotateBy(rhs.getRotation()));
    }
}
