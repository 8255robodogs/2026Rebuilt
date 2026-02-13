package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public Pose2d getBotPose2d() {
        double[] poseArray = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        return new Pose2d(
            poseArray[0],
            poseArray[1],
            new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(poseArray[5]))
        );
    }

    public double getLatencySeconds() {
        double latencyMs = table.getEntry("tl").getDouble(0)
                           + table.getEntry("cl").getDouble(0);
        return latencyMs / 1000.0;
    }
}
