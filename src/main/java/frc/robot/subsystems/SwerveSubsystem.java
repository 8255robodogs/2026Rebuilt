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
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


//things to make the field work in elastic
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class SwerveSubsystem extends SubsystemBase {
 
  double maximumSpeed = Units.feetToMeters(10);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

  //rebuilt specific positioning
  private final Translation2d blueHub = new Translation2d(4.611624,4.021328);
  private final Translation2d blueLeftShuttleTarget = new Translation2d(1.305812, 5);
  private final Translation2d blueRightShuttleTarget = new Translation2d(1.305812, 3);

  private final Translation2d redLeftShuttleTarget = new Translation2d(15.2, 3);
  private final Translation2d redHub = new Translation2d(11.901424, 4.021328);
  private final Translation2d redRightShuttleTarget = new Translation2d(15.2, 5);


  SwerveDrive swerveDrive;
  LimelightSubsystem limelight;

  private final Field2d field = new Field2d();
  private final PIDController headingPID = new PIDController(8.0, 0.1, 1); //used for auto-lock on rotation

  //aim line to show current target
  Translation2d target = new Translation2d(0.5,0.5);
  private final FieldObject2d aimLine = field.getObject("AimLine");

  public SwerveSubsystem(LimelightSubsystem limelight) {
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      this.limelight = limelight;

      //stddevs for limelight
      swerveDrive.setVisionMeasurementStdDevs(
        VecBuilder.fill(
          limelight.getXYStdDev(),
          limelight.getXYStdDev(),
          limelight.getRotStdDev()
        )
      );

      //used to draw the field in the dashboard
      SmartDashboard.putData("Field", field);

      //used for auto-lock on rotation
      headingPID.enableContinuousInput(-Math.PI, Math.PI);
      headingPID.setTolerance(Units.degreesToRadians(4));


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
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
      swerveDrive.zeroGyro();
      if (isRedAlliance()){
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } 
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }


  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
      swerveDrive.addVisionMeasurement(visionPose, timestampSeconds);
  }




  /**
  * Setup AutoBuilder for PathPlanner.
  */
  public void setupPathPlanner(){
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry,// Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity,// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        (speedsRobotRelative, moduleFeedForwards) -> 
        {
          var speeds = new ChassisSpeeds(
          speedsRobotRelative.vxMetersPerSecond,
          speedsRobotRelative.vyMetersPerSecond,
          speedsRobotRelative.omegaRadiansPerSecond);
          if (enableFeedforward){
            swerveDrive.drive(
              speeds,
              swerveDrive.kinematics.toSwerveModuleStates(speeds),
              moduleFeedForwards.linearForces()
            );
          }else{
            swerveDrive.setChassisSpeeds(speeds);
          }
        },
        // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PPHolonomicDriveController(    
          new PIDConstants(1, 0.000000, 0), // Translation PID constants
          new PIDConstants(1.5, 0.0000, 0)  // Rotation PID constants
        ),
        config,// The robot configuration
        this::isRedAlliance, //Boolean supplier for if we are red alliance
        this // Reference to this subsystem to set requirements
      );

    } catch (Exception e){
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
  public Command driveToPose(Pose2d pose, double speedInMetersPerSecond)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      speedInMetersPerSecond, 0.2,
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

    //write out our encoder angles to the network tables
    double[] swerveAngles = new double[4];
    for (int i=0; i< swerveAngles.length;i++){
      swerveAngles[i]=Math.round(swerveDrive.getModules()[i].getAbsolutePosition());
    }
    SmartDashboard.putNumberArray("Swerve Angles", swerveAngles);

    

    //handle limelight
    if(!edu.wpi.first.wpilibj.RobotBase.isSimulation()){ //ensure we are not inside a simulation

      //dynamic standard deviations.
      if (limelight.hasReliableTarget()) {
      Pose2d visionPose = limelight.getBotPose2d();
      if (visionPose != null) {

        Pose2d currentPose = getPose();

        double translationError =
            currentPose.getTranslation()
                       .getDistance(visionPose.getTranslation());

        double xyStdDev = limelight.getXYStdDev();
        double rotStdDev = limelight.getRotStdDev();

        // Smooth adaptive scaling
        double errorScale = 1.0 + (translationError * 1.5);
        errorScale = Math.min(errorScale, 5.0);

        xyStdDev *= errorScale;
        rotStdDev *= errorScale;

        swerveDrive.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
        );

        double timestamp =
            Timer.getFPGATimestamp() - limelight.getLatencySeconds();

        swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }
}
      
    }


    //publish the field to elastic (dashboard)
    field.setRobotPose(swerveDrive.getPose());
    aimLine.setPoses(getPose(), new Pose2d(getHubOrIdealShuttleTarget(), new Rotation2d(0)));
    
    //publish our location
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y", getPose().getY());
    SmartDashboard.putNumber("Pose Deg", getPose().getRotation().getDegrees());

    //publish our speed
    ChassisSpeeds s = getRobotVelocity();
    SmartDashboard.putNumber("vx", Math.round(s.vxMetersPerSecond * 100.0) / 100.0);
    SmartDashboard.putNumber("vy",Math.round(s.vyMetersPerSecond * 100.0) / 100.0);
    SmartDashboard.putNumber("omega",Math.round(s.omegaRadiansPerSecond * 100.0) / 100.0);
    SmartDashboard.putNumber("omega (deg per sec)",Math.toDegrees(s.omegaRadiansPerSecond));



  }


    //simulation code for running a virtual robot
    @Override
    public void simulationPeriodic(){
      swerveDrive.updateOdometry();
    }




 


  public Command driveFacingTarget(Supplier<Double> xInput, Supplier<Double> yInput) {
  return this.run(() -> {

    // Field-relative translation from driver
    double xSpeed = xInput.get();
    double ySpeed = yInput.get();
    if (isRedAlliance()) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
    }
    xSpeed *= swerveDrive.getMaximumChassisVelocity();
    ySpeed *= swerveDrive.getMaximumChassisVelocity();

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


  public double distanceToMyTarget(){
      return getPose().getTranslation().getDistance(getHubOrIdealShuttleTarget());
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

  private Translation2d modifyATargetForMovement(Translation2d baseTarget){
    double projectileHangtime = 0.95;

    //get robot-relative speed
    ChassisSpeeds robotRelative = getRobotVelocity();

    //convert to field-relative speed
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelative, getPose().getRotation());

    double xDrift = -fieldRelative.vxMetersPerSecond * projectileHangtime;
    double yDrift = -fieldRelative.vyMetersPerSecond * projectileHangtime;

    Translation2d modifiedTarget = new Translation2d
    (
      baseTarget.getX() + xDrift,
      baseTarget.getY() + yDrift
    );

    return modifiedTarget;
  }


}
