package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefscapeClimbSubsystem;

public class ClimberUpCmd extends Command {

    private final ReefscapeClimbSubsystem climber;
    

    public ClimberUpCmd(ReefscapeClimbSubsystem climber) {
        this.climber = climber;        
    }

    @Override
    public void initialize() {
        climber.climbPiston.set(Value.kForward);

    }

    @Override
    public void execute() {


    }

    

    @Override
    public boolean isFinished() {

        return climber.climbPiston.get() == Value.kForward;

    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
