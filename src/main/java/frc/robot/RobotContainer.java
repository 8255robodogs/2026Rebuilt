package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ReefscapeClimbSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.LeftFlywheelSubsystem;
import frc.robot.subsystems.RightFlywheelSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.ReefscapeElevatorSubsystem;
import frc.robot.subsystems.ReefscapeHeadSubsystem;
import frc.robot.subsystems.ReefscapeLEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;



public class RobotContainer {
  

  //used for DPAD driving
  public double creepSpeed = 0.2;


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  //private final ReefscapeElevatorSubsystem elevator = new ReefscapeElevatorSubsystem();
  //private final ReefscapeHeadSubsystem head = new ReefscapeHeadSubsystem();
  //private final ReefscapeAlgaeSubsystem algaeSystem = new ReefscapeAlgaeSubsystem();
  //private final ReefscapeClimbSubsystem climber = new ReefscapeClimbSubsystem();
  //private final ReefscapeLEDSubsystem leds = new ReefscapeLEDSubsystem();
  
  private final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final LeftFlywheelSubsystem flyleft = new LeftFlywheelSubsystem();
  private final RightFlywheelSubsystem flyright = new RightFlywheelSubsystem();
  private final HarvestorSubsystem harvestor = new HarvestorSubsystem();

  

  //declare the controller
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  //Setting up swerve drive
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY() *-1,
    () -> m_driverController.getLeftX() *-1)
    .withControllerRotationAxis(m_driverController::getRightX)
    .deadband(OperatorConstants.kDriverStickDeadband)
    .scaleTranslation(1)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(m_driverController::getRightX, 
    m_driverController::getRightY)
    .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  CameraServer cameraServer;
  
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();




  public RobotContainer() {
    //Explains the robot to pathplanner so it can be used to drive paths.
    drivebase.setupPathPlanner();
    
    //create the camera
    //CameraServer.addServer("cams");
    //CameraServer.startAutomaticCapture();

    

    //registers controls
    configureBindings();
    configureAutos();
  }
  
  private void configureBindings() {
    
    //DRIVER CONTROLLER (CONTROLLER ZERO)

    //driving controls

    //this makes the drivebase drive. Very important.
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //hold b to lock the wheels and resist being pushed
    m_driverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //press start to zero your heading
    m_driverController.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    

    //DPAD creeping
    m_driverController.povRight().whileTrue(drivebase.driveRobotRelativeCommand(0.0, creepSpeed, 0.0));
    m_driverController.povLeft().whileTrue(drivebase.driveRobotRelativeCommand(0.0, -creepSpeed, 0.0));
    m_driverController.povUp().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, 0, 0.0));
    m_driverController.povDown().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, 0, 0.0));
    
    m_driverController.povDownRight().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, creepSpeed, 0.0));
    m_driverController.povDownLeft().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, -creepSpeed, 0.0));
    m_driverController.povUpRight().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, creepSpeed, 0.0));
    m_driverController.povUpLeft().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, -creepSpeed, 0.0));




    //Driver CONTROLLER (CONTROLLER zero)

    
    
    //conveyor (2026)
    m_driverController.rightBumper().onTrue(conveyor.setHeadSpeed(1));
    m_driverController.rightBumper().onFalse(conveyor.setHeadSpeed(0));

    //kicker (2026)
    m_driverController.y().onTrue(kicker.setHeadSpeed(.9));
    m_driverController.y().onFalse(kicker.setHeadSpeed(0));
    m_driverController.rightBumper().onTrue(kicker.setHeadSpeed(-1));
    m_driverController.rightBumper().onFalse(kicker.setHeadSpeed(0));

    //flywheel
    m_driverController.leftBumper().onTrue(flyleft.setHeadSpeed(.7));
    m_driverController.leftBumper().onFalse(flyleft.setHeadSpeed(0));
    m_driverController.leftBumper().onTrue(flyright.setHeadSpeed(-.7));
    m_driverController.leftBumper().onFalse(flyright.setHeadSpeed(0));

    //Harvestor
    m_driverController.x().onTrue(harvestor.setHeadSpeed(.4));
    m_driverController.x().onFalse(harvestor.setHeadSpeed(0));


  } 

  private void configureAutos(){
    
    m_autoChooser.setDefaultOption("Left", "Left");
    m_autoChooser.addOption("Middle", "Middle");
    m_autoChooser.addOption("MiddleLeft", "MiddleLeft");
    m_autoChooser.addOption("MiddleRight", "MiddleRight");
    m_autoChooser.addOption("BlueLeft","BlueLeft");
    SmartDashboard.putData("Auto Choices", m_autoChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
/*
    if(m_autoChooser.getSelected() == "Left"){
      return 
      AutoBuilder.buildAuto("Test Auto")
      .alongWith(elevator.setLevel(4))
      .alongWith(head.setHeadSpeed(0))
      
      .andThen(head.setHeadSpeed(.7))
      .andThen(new WaitCommand(1))
      .andThen(head.setHeadSpeed(0))
      
      .andThen(elevator.setLevel(1))
      .andThen(AutoBuilder.buildAuto("Human Left"))
      
      .andThen(new WaitCommand(1))
      
      .andThen(AutoBuilder.buildAuto("Second Score"))
  
      //.andThen(head.setHeadSpeed(.2))
      
      ;
    }


    if(m_autoChooser.getSelected() == "BlueLeft"){
      return 
      AutoBuilder.buildAuto("M1BlueAuto")
      .alongWith(elevator.setLevel(4))
      .alongWith(head.setHeadSpeed(0))
      
      .andThen(head.setHeadSpeed(.7))
      .andThen(new WaitCommand(1))
      .andThen(head.setHeadSpeed(0))
      
      .andThen(elevator.setLevel(1))
      .andThen(AutoBuilder.buildAuto("Human Left"))
      
      .andThen(new WaitCommand(1))
      
      .andThen(AutoBuilder.buildAuto("Second Score"))
  
      //.andThen(head.setHeadSpeed(.2))
      
      ;
    }




    if(m_autoChooser.getSelected() == "Middle"){
      return
      AutoBuilder.buildAuto("center1auto")
      .alongWith(elevator.setLevel(4))

      .andThen(new WaitCommand(1))

      .andThen(head.setHeadSpeed(.7))
      .andThen(new WaitCommand(0.5))
      .andThen(head.setHeadSpeed(0))

      .andThen(AutoBuilder.buildAuto("moveback"))

      .andThen(elevator.setLevel(1))


      ;
    }


    //middlle Left
    if(m_autoChooser.getSelected() == "MiddleLeft"){
      return
      AutoBuilder.buildAuto("center1auto")
      .alongWith(elevator.setLevel(4))

      .andThen(head.setHeadSpeed(.7))
      .andThen(new WaitCommand(0.5))
      .andThen(head.setHeadSpeed(0))

      .andThen(elevator.setLevel(1))

      .andThen(new WaitCommand(0.25))
      .andThen(AutoBuilder.buildAuto("center2auto"))

      .andThen(new WaitCommand(2))
      .andThen(AutoBuilder.buildAuto("center3auto"));
    }

 
      //Move Right
      if(m_autoChooser.getSelected() == "MiddleRight"){
        return
        AutoBuilder.buildAuto("center1auto")
        .alongWith(elevator.setLevel(4))
  
        .andThen(head.setHeadSpeed(.7))
        .andThen(new WaitCommand(0.5))
        .andThen(head.setHeadSpeed(0))
  
        .andThen(elevator.setLevel(1))
  
        .andThen(new WaitCommand(0.25))
        .andThen(AutoBuilder.buildAuto("C2"))
  
        .andThen(new WaitCommand(2))
        .andThen(AutoBuilder.buildAuto("C3"));
    }
*/

   
    return null;
    
  }
}
