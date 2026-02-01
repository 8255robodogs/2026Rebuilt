package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefscapeClimbSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ClimberDownCmd extends Command {

    private final ReefscapeClimbSubsystem climber;
    
    public ClimberDownCmd(ReefscapeClimbSubsystem climber) {
        this.climber = climber;        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
            climber.climbPiston.set(Value.kReverse);
    }

    

    @Override
    public boolean isFinished() {
        return climber.climbPiston.get() == Value.kForward;
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
