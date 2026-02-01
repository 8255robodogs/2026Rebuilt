package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReefscapeClimbSubsystem extends SubsystemBase{

    //pressure control module (PCM)
    private int pcmCanId = Constants.pcmCanId;

    //climbing piston
    public DoubleSolenoid climbPiston;
    private int climbPistonForwardPort = 2;
    private int climbPistonReversePort = 3;

    
    //Constructor
    public ReefscapeClimbSubsystem(){
        climbPiston = new DoubleSolenoid(
            pcmCanId,
            PneumaticsModuleType.CTREPCM,
            climbPistonForwardPort,
            climbPistonReversePort );

    }


    @Override
    public void periodic(){
        
    }




}



