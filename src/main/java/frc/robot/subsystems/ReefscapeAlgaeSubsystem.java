// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ReefscapeAlgaeSubsystem extends SubsystemBase{

//     //pressure control module (pcm)
//     private int pcmCanId = Constants.pcmCanId;

//     //algae collector rig
//     private DoubleSolenoid algaeCollectorPiston;
//     private int algaeCollectorPistonCanId = 21;
//     private int algaeCollectorPistonForwardPort = 5;
//     private int algaeCollectorPistonReversePort = 4;

//     private VictorSPX motor;
//     private int motorControllerCanId = 5;
//     private boolean invertedMotor = true;

//     //algae remover rig
//     private Solenoid algaeRemoverPneumatic;
//     private int algaeRemoverPneumaticCanId = 21;
//     private int algaeRemoverPneumaticPort = 1;
//     private boolean algaeRemoverPneumaticReversed = false;

//     //Constructor
//     public ReefscapeAlgaeSubsystem(){
//         motor = new VictorSPX(motorControllerCanId);
//         algaeCollectorPiston = new DoubleSolenoid(
//             pcmCanId, 
//             PneumaticsModuleType.CTREPCM,
//             algaeCollectorPistonForwardPort,
//             algaeCollectorPistonReversePort
//         );

//         algaeRemoverPneumatic = new Solenoid(
//             algaeRemoverPneumaticCanId,
//             PneumaticsModuleType.CTREPCM,
//             algaeRemoverPneumaticPort
//         );
//         algaeRemoverPneumatic.set(algaeRemoverPneumaticReversed);


//     }

//     public void setMotorSpeed(double speed){
//         double newSpeed = speed;
//         if(invertedMotor == true){
//             newSpeed = newSpeed *-1;
//         }
//         motor.set(VictorSPXControlMode.PercentOutput, newSpeed);
//     }


//     @Override
//     public void periodic(){
        
//     }

//     public Command setAlgaeCollectorMotorSpeed(double speed){
//         return Commands.runOnce(() -> setMotorSpeed(speed));
//     }

//     public Command setAlgaeCollectorPistonExtended(boolean trueOrFalse){
//         return Commands.runOnce(() -> {
//             if (trueOrFalse == true) {
//                 algaeCollectorPiston.set(Value.kForward);
//             } else {
//                 algaeCollectorPiston.set(Value.kReverse);
//             }
//         });
//     }

//     public Command toggleAlgaeRemover() {
//         return Commands.runOnce(() -> algaeRemoverPneumatic.set(!algaeRemoverPneumatic.get()));
//     }



// }



