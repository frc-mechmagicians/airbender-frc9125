package frc.robot;
import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotPIDgyroPreSeason extends TimedRobot {
  static final int TEST_DISTANCE = 200;
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightSpark = new CANSparkMax(3, MotorType.kBrushless);

  //FIX - tune to run on floor
  final double kP = 0.1;
  final double kI = 0.001;
  final double kD = 0.01;
  final double iLimit = 5;

  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  private RelativeEncoder leftEncoder = driveLeftSpark.getEncoder();

  private RelativeEncoder rightEncoder = driveRightSpark.getEncoder();
  
  com.revrobotics.SparkMaxPIDController leftPid = driveLeftSpark.getPIDController();
  com.revrobotics.SparkMaxPIDController rightPid = driveRightSpark.getPIDController();

  DifferentialDrive drive = new DifferentialDrive(driveLeftSpark, driveRightSpark);
 
  @Override
  public void robotInit() {

  }

  double setpoint = getDistanceToPos(TEST_DISTANCE); // Dist in centi meters

  public void autonomousInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    leftPid.setP(kP);
    leftPid.setI(kI);
    leftPid.setD(kD);
        
    rightPid.setP(kP);
    rightPid.setI(kI);
    rightPid.setD(kD);

  
    //FIX - tune to run on floor
    double kIz = iLimit; 
    double kFF = 0.000156; 
    double kMaxOutput = 0.4; 
    double kMinOutput = -0.4;

    leftPid.setIZone(kIz);
    leftPid.setFF(kFF);
    leftPid.setOutputRange(kMinOutput, kMaxOutput);

    rightPid.setIZone(kIz);
    rightPid.setFF(kFF);
    rightPid.setOutputRange(kMinOutput, kMaxOutput);
    
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

    driveLeftSpark.set(0);
    driveLeftSpark.set(0);
    driveRightSpark.set(-0);
    driveRightSpark.set(-0);
    driveRightSpark.setInverted(true);
    drive.setSafetyEnabled(isAutonomousEnabled());
  }
  
  double getDistanceToPos(int cm) {
    //FIX 
    double oneRotationBT = 8.65;
    double circum = 2 * Math.PI * 3 * 2.54;
    double nRR = cm/circum;
    return nRR * oneRotationBT;
  }

  @Override
  public void autonomousPeriodic() { 
    leftPid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    rightPid.setReference(setpoint, CANSparkMax.ControlType.kPosition);

    if (Math.abs(leftEncoder.getPosition()) >= Math.abs(setpoint)) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      setpoint = - getDistanceToPos(TEST_DISTANCE); 
    }
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", leftEncoder.getPosition());
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}