package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.VisionMeasurement;

public class Vision extends SubsystemBase{
    private Pose2d _limelightFourPose = new Pose2d();

    private PoseEstimate _limelightFourPoseEstimate = new PoseEstimate();

    private static final double filterDistanceError = 2;
    private static final double filterAngleError = 5;
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * filterDistanceError), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * filterAngleError), 0.02);

    public Vision()
    {
    }

    @Override
    public void periodic() {
        _limelightFourPose = LimelightHelpers.getBotPose2d("limelight-four");
        // if (DriverStation.isDisabled() && DriverStation.getAlliance().isPresent()){
        //     if (DriverStation.getAlliance().get() == Alliance.Red){
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", FieldConstants.kRedTagIDs);
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-three", FieldConstants.kRedTagIDs);
        //     } else {    
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-three", FieldConstants.kBlueTagIDs);
        //         LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", FieldConstants.kBlueTagIDs);
        //     }
        // }
    }

    public Pose2d getLimelightFourPose() {
        return _limelightFourPose;
    }

    public double getDistance()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        var ty = table.getEntry("ty").getDouble(0) * 1.18; // constant
        var tx = table.getEntry("tx").getDouble(0);
        var tv = table.getEntry("tv").getDouble(0);
        
        if (tv != 0.0) {
            double distance = .905 / Math.tan(Math.toRadians(ty)); // constant
            double filterDistance = limelightDistanceFilter.calculate(distance);
            SmartDashboard.putNumber("VisionSystemGetDistance", distance);
            return filterDistance; 
        }
        else {
            return 0.0;
        }
    }

    public ArrayList<VisionMeasurement> getVisionMeasurements(Rotation2d rotation)
    {
        ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();
            LimelightHelpers.SetRobotOrientation(
                "limelight-four",
                rotation.getDegrees(),
                0, 
                0, 
                0, 
                0,
                0
            );
            LimelightHelpers.SetIMUMode("limelight-four", 0);

            PoseEstimate limelightFourPoseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    "limelight-four"
                );

            if (limelightFourPoseEstimate != null && limelightFourPoseEstimate.tagCount > 0) {
                _limelightFourPoseEstimate = limelightFourPoseEstimate;
                visionMeasurements.add(
                    new VisionMeasurement(
                        limelightFourPoseEstimate.pose,
                        limelightFourPoseEstimate.timestampSeconds
                    )
                );
            }
        return visionMeasurements;
    }

    public Pose2d getFrontLimelightPose(){
        return _limelightFourPoseEstimate.pose;
    }

    public Pose2d getRobotPoseInTargetSpace() {
        var botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight-four");

        return new Pose2d(botPoseTargetSpace[0], botPoseTargetSpace[2], new Rotation2d(botPoseTargetSpace[4]));
    }
}

