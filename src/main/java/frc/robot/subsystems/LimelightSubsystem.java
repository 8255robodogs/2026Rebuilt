package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;



public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private final double minimumTagsForVisionToBeTrusted = 1;
    private final double maximumAverageTagDistanceToTrust = 4;




    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasReliableTarget() {
        boolean result = true;

        //make sure we actually see a tag
        double tagValid = table.getEntry("tv").getDouble(0);
        if(tagValid != 1){
            result = false;
        } 

        //make sure we are calculating with a certain number of tags
        double tagCount = table.getEntry("tagcount").getDouble(0);
        if (tagCount < minimumTagsForVisionToBeTrusted){
            result = false;
        }

        //don't trust tag readings if the average distance is too far away
        double avgDist = table.getEntry("avgdist").getDouble(0);
        if (avgDist > maximumAverageTagDistanceToTrust) {
            result = false;
        }

        

        //post check results to dashboard
        SmartDashboard.putNumber("LL TagCount", tagCount);
        SmartDashboard.putNumber("LL AvgDist", avgDist);
        SmartDashboard.putBoolean("LL HasTarget", tagValid == 1);



        return result;
    }

    public Pose2d getBotPose2d() {
    double[] poseArray =
        table.getEntry("botpose_wpiblue")
             .getDoubleArray(new double[6]);

    // Reject invalid data
    if (poseArray.length < 6 || (poseArray[0] == 0 && poseArray[1] == 0)) {
        return null;
    }

    return new Pose2d(
        poseArray[0],
        poseArray[1],
        Rotation2d.fromDegrees(poseArray[5])
    );
}


    public double getLatencySeconds() {
        double latencyMs = table.getEntry("tl").getDouble(0)
                           + table.getEntry("cl").getDouble(0);
        return latencyMs / 1000.0;
    }
}
