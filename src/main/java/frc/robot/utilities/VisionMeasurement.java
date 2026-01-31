package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    public Pose2d pose;
    public double timestamp;

    public VisionMeasurement(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
