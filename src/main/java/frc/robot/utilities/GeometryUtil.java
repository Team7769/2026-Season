package frc.robot.utilities;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.configuration.*;

/**
 * Utility class for various geometry related actions.
 */
public class GeometryUtil {
    /**
     * Determines if the robot or simulation is currently on the Red Alliance.
     * @return True if on the red alliance, false otherwise.
     */
    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
    
    /**
     * Mirrors a provided blue-side translation for the red alliance.
     * @param blueTranslation The blue-side translation.
     * @return The translation, mirrored for the red alliance.
     */
    public static Translation2d mirrorTranslationForRedAlliance(Translation2d blueTranslation) {
        return new Translation2d(
                blueTranslation.getX() +
                        FieldConstants.kFieldLength -
                        (2 * blueTranslation.getX()),

                blueTranslation.getY());
    }

    public static Translation2d rotateTranslationForRedAlliance(Translation2d blueTranslation) {
        var redTranslation = blueTranslation.rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg);
        return redTranslation;
    }
    
    public static Translation2d mirrorReef(Translation2d blueTranslation2d) {
        Translation2d reefTranslation = new Translation2d();
        reefTranslation = blueTranslation2d.rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg);
        return reefTranslation;
    }

    /**
     * Gets the angle to a target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @param isFollowingFront Boolean to determine if the angle should be determined from the front or back of the robot.
     * @return The angle, in degrees, to the target translation (point).
     */
    public static double getAngleToTarget(Translation2d targetTranslation, Supplier<Pose2d> currentPose, boolean isFollowingFront) {
        var currentPoseValue = currentPose.get();
        return targetTranslation
            .minus(currentPoseValue.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(isFollowingFront ? 0 : 180))
            .minus(currentPoseValue.getRotation())
            .getDegrees();
    }

    /**
     * Gets the distance to a target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @return The distance, in meters, to the target translation (point).
     */
    public static double getDistanceToTarget(Translation2d targetTranslation, Supplier<Pose2d> currentPose) {
        return currentPose.get()
            .getTranslation()
            .getDistance(targetTranslation);
    }

    /**
     * Gets the rotation difference between a provided angle.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @param angle The angle to evaluate the rotation to.
     * @return The difference between the supplied pose and the provided angle.
     */
    public static double getRotationDifference(Supplier<Pose2d> currentPose, double angle) {
        return Rotation2d
            .fromDegrees(angle)
            .minus(currentPose.get().getRotation())
            .getDegrees();
    }
    
    /**
     * Gets the rotation difference between a provided angle.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @param angle The angle to evaluate the rotation to.
     * @return The difference between the supplied pose and the provided angle.
     */
    public static double getRotationDifference(Pose2d currentPose, double angle) {
        return Rotation2d
            .fromDegrees(angle)
            .minus(currentPose.getRotation())
            .getDegrees();
    }

    /**
     * Gets the distance across the x-axis to the target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @return The x, in meters, to the target translation (point).
     */
    public static double getXDifference(Pose2d targetPose, Supplier<Pose2d> currentPose) {
        return currentPose.get().getTranslation().getX() - targetPose.getTranslation().getX();
    }

    /**
     * Gets the distance across the x-axis to the target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @return The x, in meters, to the target translation (point).
     */
    public static double getXDifference(Pose2d targetPose, Pose2d currentPose) {
        return currentPose.getTranslation().getX() - targetPose.getTranslation().getX();
    }

    /**
     * Gets the distance across the y-axis to the target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @return The y, in meters, to the target translation (point).
     */
    public static double getYDifference(Pose2d targetPose, Supplier<Pose2d> currentPose) {
        return currentPose.get().getTranslation().getY() - targetPose.getTranslation().getY();
    }
    
    /**
     * Gets the distance across the y-axis to the target translation (point).
     * @param targetTranslation The target translation (point) on the field.
     * @param currentPose Supplier for the current pose to evaluate from. This should usually be the pose from the pose estimator.
     * @return The y, in meters, to the target translation (point).
     */
    public static double getYDifference(Pose2d targetPose, Pose2d currentPose) {
        return currentPose.getTranslation().getY() - targetPose.getTranslation().getY();
    }
}
