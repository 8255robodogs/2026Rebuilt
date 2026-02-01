package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SetPoseCmd extends Command {

    private final SwerveSubsystem swerveSys;

    private final Pose2d pose;

    public SetPoseCmd(Pose2d pose, SwerveSubsystem swerveSys) {
        this.pose = pose;
        this.swerveSys = swerveSys;
        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveSys.setPose(pose);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
