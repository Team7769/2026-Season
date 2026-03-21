package frc.robot.configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.GeometryUtil;

public final class FieldConstants {
    // public static final double kFieldLength = 17.548;
    // public static final double kFieldWidth = 8.052;
    public static final double kFieldLength = 16.54;
    public static final double kFieldWidth = 8.069;
    public static final double kHalfFieldWidth = kFieldWidth / 2;
    public static final double kQuarterFieldWidth = kFieldWidth / 4;
    public static final double kHalfFieldLength = kFieldLength / 2;
    public static final Translation2d kFieldCenter = new Translation2d(kHalfFieldLength, kHalfFieldWidth);


    // Coral Source Coordinates:
    public static final int[] kRedTagIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    public static final int[] kBlueTagIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

    public static final double kRobotWidth = 0.622;
    public static final Translation2d kBlueHub = new Translation2d(4.6256, 4.0345);
    public static final Translation2d kBlueZoneA = new Translation2d(2.0172, 6.0518);
    public static final Translation2d kBlueZoneB = new Translation2d(2.0172, 2.0172);
    public static final Translation2d kBlueDepot = new Translation2d(0.3429, 5.9631);

    public static final Pose2d kRightTrench = new Pose2d(kHalfFieldLength, 0.810, Rotation2d.fromDegrees(90));
    public static final Pose2d kLeftTrench = new Pose2d(kHalfFieldLength, 7.271, Rotation2d.fromDegrees(270));
    
    public static final Pose2d kBlueClimbEngageLeftFront = new Pose2d(1.075 + kRobotWidth / 2, 4.17, Rotation2d.fromDegrees(180));
    public static final Pose2d kBlueClimbEngageRightFront = new Pose2d(1.075 + kRobotWidth / 2, 3.27, Rotation2d.fromDegrees(180));
    public static final Pose2d kBlueClimbStageLeftFront = new Pose2d(1.075 + 2 * kRobotWidth, 4.17, Rotation2d.fromDegrees(180));
    public static final Pose2d kBlueClimbStageRightFront = new Pose2d(1.075 + 2 * kRobotWidth, 3.27, Rotation2d.fromDegrees(180));
    
    public static final Pose2d kBlueClimbStageRightSide = new Pose2d(2, 2.000, Rotation2d.fromDegrees(325));
    public static final Pose2d kBlueClimbEngageRightSide = new Pose2d(1.065, 2.814, Rotation2d.fromDegrees(90));
    public static final Pose2d kBlueClimbStageLeftSide = new Pose2d(2, 5.501, Rotation2d.fromDegrees(35));
    public static final Pose2d kBlueClimbEngageLeftSide = new Pose2d(1.065, 4.687, Rotation2d.fromDegrees(270));

    public static final Translation2d kRedHub = GeometryUtil.rotateTranslationForRedAlliance(kBlueHub);
    public static final Translation2d kRedZoneA = GeometryUtil.rotateTranslationForRedAlliance(kBlueZoneA);
    public static final Translation2d kRedZoneB = GeometryUtil.rotateTranslationForRedAlliance(kBlueZoneB);
    public static final Translation2d kRedDepot = GeometryUtil.rotateTranslationForRedAlliance(kBlueDepot);

    public static final Pose2d kRedClimbEngageLeftFront = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbEngageLeftFront);
    public static final Pose2d kRedClimbEngageRightFront = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbEngageRightFront);
    public static final Pose2d kRedClimbStageLeftFront = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbStageLeftFront);
    public static final Pose2d kRedClimbStageRightFront = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbStageRightFront);

    public static final Pose2d kRedClimbStageRightSide = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbStageRightSide);
    public static final Pose2d kRedClimbEngageRightSide = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbEngageRightSide);
    public static final Pose2d kRedClimbStageLeftSide = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbStageLeftSide);
    public static final Pose2d kRedClimbEngageLeftSide = GeometryUtil.rotatePoseForRedAlliance(kBlueClimbEngageLeftSide);
  }