// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class FlywheelSubsystem extends SubsystemBase {

//   private final SparkMax topMotor = new SparkMax(10, MotorType.kBrushless);
//   private final SparkMax bottomMotor = new SparkMax(11, MotorType.kBrushless);

//   private final RelativeEncoder encoder;
//   private final SparkPIDController pid;

//   // Example RPM — tune for your shooter
//   private static final double TARGET_RPM = 2500.0;

//   public FlywheelSubsystem() {

//     topMotor.restoreFactoryDefaults();
//     bottomMotor.restoreFactoryDefaults();

//     bottomMotor.follow(topMotor, true);

//     encoder = topMotor.getEncoder();
//     pid = topMotor.getPIDController();

//     // --- PID GAINS (START HERE) ---
//     pid.setP(0.0002);
//     pid.setI(0.0);
//     pid.setD(0.0);
//     pid.setFF(0.00018); // Feedforward is HUGE for flywheels

//     pid.setOutputRange(-1.0, 1.0);

//     //topMotor.burnFlash();
//     //bottomMotor.burnFlash();
//   }

//   public void setShooterRPM(double rpm) {
//     pid.setReference(rpm, SparkMax.ControlType.kVelocity);
//   }

//   public void stopShooter() {
//     topMotor.stopMotor();
//   }

//   public double getRPM() {
//     return encoder.getVelocity();
//   }
// }

