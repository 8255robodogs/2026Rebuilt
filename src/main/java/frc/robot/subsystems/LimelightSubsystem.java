package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private final double minimumTagsForVisionToBeTrusted = 2;
    private final double maximumAverageTagDistanceToTrust = 4.0;

    // Base trust values
    private final double baseXYStdDev = 0.11;
    private final double baseRotStdDevDegrees = 4;

    //vision holding times
    private double lastGoodVisionTime = 0;
    private final double visionHoldTime = 0.25; // 250ms

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Returns the full botpose array or null if invalid.
     */
    private double[] getPoseArray() {
        double[] poseArray =
            table.getEntry("botpose_wpiblue")
                 .getDoubleArray(new double[11]);

        if (poseArray.length < 11) {
            return null;
        }

        return poseArray;
    }

    /**
     * Returns xy std deviation.
     */
    public double getXYStdDev() {
        return baseXYStdDev;
    }

    /**
     * Returns  rotation std deviation.
     */
    public double getRotStdDev() {
        return Units.degreesToRadians(baseRotStdDevDegrees);
    }

    /**
     * Reliability gate.
     */
    public boolean hasReliableTarget() {

        double[] poseArray = getPoseArray();
        if (poseArray == null) return false;

        double tagValid = table.getEntry("tv").getDouble(0);
        double tagCount = poseArray[7];
        double avgDist = poseArray[9];

        SmartDashboard.putNumber("LL TagCount", tagCount);
        SmartDashboard.putNumber("LL AvgDist", avgDist);
        SmartDashboard.putBoolean("LL HasTarget", tagValid == 1);

        boolean currentlyReliable =
        tagValid == 1
        && tagCount >= minimumTagsForVisionToBeTrusted
        && avgDist <= maximumAverageTagDistanceToTrust
        && poseArray[0] != 0
        && poseArray[1] != 0;

    if (currentlyReliable) {
        lastGoodVisionTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return true;
    }

    // Allow small dropout without disabling vision
    double timeSinceGood =
        edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastGoodVisionTime;

    return timeSinceGood < visionHoldTime;
    }

    public Pose2d getBotPose2d() {
    double[] poseArray = getPoseArray();
    if (poseArray == null) return null;

    double x = poseArray[0];
    double y = poseArray[1];

    // Reject obviously invalid pose
    if (Math.abs(x) < 0.001 && Math.abs(y) < 0.001) {
        return null;
    }

    return new Pose2d(
        x,
        y,
        Rotation2d.fromDegrees(poseArray[5])
    );
}

    public double getLatencySeconds() {
        double latencyMs = table.getEntry("tl").getDouble(0)
                           + table.getEntry("cl").getDouble(0);
        return latencyMs / 1000.0;
    }
}