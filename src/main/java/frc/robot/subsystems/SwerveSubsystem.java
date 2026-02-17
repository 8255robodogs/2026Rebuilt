package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.zip.Inflater;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.subsystems.LimelightSubsystem;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


//things to make the field work in elastic
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class SwerveSubsystem extends SubsystemBase {
 
  double maximumSpeed = Units.feetToMeters(10);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

  //rebuilt specific positioning
  private final Translation2d blueHub = new Translation2d(4.611624,4.021328);
  private final Translation2d blueLeftShuttleTarget = new Translation2d(2.305812, 6);
  private final Translation2d blueRightShuttleTarget = new Translation2d(2.305812, 2);
  private final Pose2d blueOutpostRobotPose = new Pose2d(new Translation2d(2,2),new Rotation2d(0)); //TODO set the actual value

  private final Translation2d redLeftShuttleTarget = new Translation2d(4.33, 2);
  private final Translation2d redHub = new Translation2d(11.901424, 4.021328);
  private final Translation2d redRightShuttleTarget = new Translation2d(4.33, 6);
  private final Pose2d redOutpostRobotPose = new Pose2d(new Translation2d(2,2),new Rotation2d(180)); //TODO set the actual value


  SwerveDrive swerveDrive;
  LimelightSubsystem limelight;
  private final Field2d field = new Field2d();
  private final PIDController headingPID = new PIDController(4.0, 0.0, 0.2); //used for auto-lock on rotation





  public SwerveSubsystem(LimelightSubsystem limelight) {
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      this.limelight = limelight;
      

      //stddevs for limelight
      swerveDrive.setVisionMeasurementStdDevs(
          VecBuilder.fill(
              0.7,
              0.7,
              Units.degreesToRadians(10)
          )
      );

      //used to draw the field in the dashboard
      SmartDashboard.putData("Field", field);

      //used for auto-lock on rotation
      headingPID.enableContinuousInput(-Math.PI, Math.PI);
      headingPID.setTolerance(Units.degreesToRadians(2));




    }catch(Exception e){
      throw new RuntimeException(e);
    }
  }
 
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(()-> {
      swerveDrive.driveFieldOriented(velocity.get());
    }
    );
  }


  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }


/**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
    
  }

  public Command cmdResetOdometry(Pose2d initialHolonomicPose) {
    return this.runOnce(() -> swerveDrive.resetOdometry(initialHolonomicPose));
}


   /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }



/**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    //return false;
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }


/**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
      
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();

    }
  }

/**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }



public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    swerveDrive.addVisionMeasurement(visionPose, timestampSeconds);
}




/**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            var invertedSpeeds = new ChassisSpeeds(
              speedsRobotRelative.vxMetersPerSecond,
              speedsRobotRelative.vyMetersPerSecond,
              -speedsRobotRelative.omegaRadiansPerSecond);
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  invertedSpeeds,
                  swerveDrive.kinematics.toSwerveModuleStates(invertedSpeeds),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(invertedSpeeds);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1, 0.000000, 0),
              // Translation PID constants
              new PIDConstants(1.5, 0.0000, 0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }


  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      swerveDrive.getMaximumChassisVelocity(), 2.0,
      swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(90)
    );

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
      pose,
      constraints,
      edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }


  public void setPose(Pose2d pose2d){
    swerveDrive.resetOdometry(pose2d);
  }

  public Command SetPose(Pose2d pose){
    return this.runOnce(() -> setPose(pose));
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


/**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  public Command driveRobotRelativeCommand(double x, double y, double rot){
    return run(() -> drive(new ChassisSpeeds(x,y,rot)));
  }

  public void periodic(){

    //write out our encoder angles to the console
    //writeSwerveAnglesToConsole();


    //handle limelight
    if (limelight.hasTarget()) {
        Pose2d visionPose = limelight.getBotPose2d();
        double timestamp = 
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
            - limelight.getLatencySeconds();

        swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    //publish the field to elastic (dashboard)
    field.setRobotPose(swerveDrive.getPose());


  }

  //for debugging and tuning swerve drive. this should be displayed in the dashboard instead. will fix later
  private void writeSwerveAnglesToConsole(){
System.out.println(
    "Swerve angles: "+  
    Math.round(swerveDrive.getModules()[0].getAbsolutePosition()) + ", " +
  Math.round(swerveDrive.getModules()[1].getAbsolutePosition()) + ", " +
  Math.round(swerveDrive.getModules()[2].getAbsolutePosition()) + ", " +
  Math.round(swerveDrive.getModules()[3].getAbsolutePosition()) + ", " 

    );
  }


  public Command driveFacingTarget(Supplier<Double> xInput, Supplier<Double> yInput) {
  return run(() -> {

    // Field-relative translation from driver
    double xSpeed = xInput.get() * swerveDrive.getMaximumChassisVelocity();
    double ySpeed = yInput.get() * swerveDrive.getMaximumChassisVelocity();

    // Get target translation
    Translation2d target = getHubOrIdealShuttleTarget();

    // Current pose
    Pose2d pose = getPose();

    // Vector from robot to target
    Translation2d delta = target.minus(pose.getTranslation());

    // Desired heading (radians)
    double desiredHeading = Math.atan2(delta.getY(), delta.getX());

    // Current heading
    double currentHeading = pose.getRotation().getRadians();

    // PID output (angular velocity)
    double omega = headingPID.calculate(currentHeading, desiredHeading);

    // Clamp if needed
    omega = Math.max(Math.min(omega, swerveDrive.getMaximumChassisAngularVelocity()), 
                     -swerveDrive.getMaximumChassisAngularVelocity());

    // Drive field-relative
    swerveDrive.driveFieldOriented(
        new ChassisSpeeds(xSpeed, ySpeed, omega)
    );

  });
}





  //2026 Rebuilt specific
  public Command driveToMyAllianceOutpost(){
    if(isRedAlliance()){
      return run(() -> driveToPose(redOutpostRobotPose));
    }else{
      return run(() -> driveToPose(blueOutpostRobotPose));
    }
  }

  private double distanceToMyHub(){
    if(isRedAlliance()){
      return getPose().getTranslation().getDistance(redHub);
    }else{
      return getPose().getTranslation().getDistance(blueHub);
    }
  }


  //check if we are on the side of our hub where we can score
  private boolean inFriendlyScoringArea(){
    if(isRedAlliance()){
      return getPose().getTranslation().getX() > redHub.getX();
    }else{
      return getPose().getTranslation().getX() < blueHub.getX();
    }
  }


  private Translation2d getHubOrIdealShuttleTarget(){
    if(isRedAlliance()){
      if(inFriendlyScoringArea()){
        return redHub;
      }else{
        if(getPose().getY() >= redHub.getY()){
          return redRightShuttleTarget;
        }else{
          return redLeftShuttleTarget;
        }
      }
    }else{
      if(inFriendlyScoringArea()){
        return blueHub;
      }else{
        if(getPose().getY() >= blueHub.getY()){
          return blueLeftShuttleTarget;
        }else{
          return blueRightShuttleTarget;
        }
      }
    }



  }



}
