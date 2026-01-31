package frc.robot.configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.GeometryUtil;

public final class FieldConstants {
    public static final double kFieldLength = 17.548;
    public static final double kFieldWidth = 8.052;
    public static final double kHalfFieldWidth = 4.026;
    public static final double kQuarterFieldWidth = 4.026/2;
    public static final double kHalfFieldLength = 8.774;
    public static final Translation2d kFieldCenter = new Translation2d(kHalfFieldLength, kHalfFieldWidth);

    // Coral Source Coordinates:
    public static final int[] kRedTagIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    public static final int[] kBlueTagIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

    public static final Translation2d kBlueHub = new Translation2d(4.6256, 4.0345);
    public static final Translation2d kBlueZoneA = new Translation2d(2.0172, 6.0518);
    public static final Translation2d kBlueZoneB = new Translation2d(2.0172, 2.0172);
    public static final Translation2d kBlueDepot = new Translation2d(0.3429, 5.9631);

    public static final Translation2d kRedHub = GeometryUtil.rotateTranslationForRedAlliance(kBlueHub);
    public static final Translation2d kRedZoneA = GeometryUtil.rotateTranslationForRedAlliance(kBlueZoneA);
    public static final Translation2d kRedZoneB = GeometryUtil.rotateTranslationForRedAlliance(kBlueZoneB);
    public static final Translation2d kRedDepot = GeometryUtil.rotateTranslationForRedAlliance(kBlueDepot);
  }