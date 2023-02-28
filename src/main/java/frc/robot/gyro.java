// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotPID extends TimedRobot {

  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr,kPTurning;

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightSpark = new CANSparkMax(3, MotorType.kBrushless);
  RelativeEncoder left_encoder, right_encoder;
  SparkMaxPIDController left_pid, right_pid;
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();




  

 // CANSparkMax driveLeftSparktwo = new CANSparkMax(2, MotorType.kBrushed);
 // CANSparkMax driveRightSparktwo = new CANSparkMax(4, MotorType.kBrushed);

  


  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */

   // propotional turning constant

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
    Joystick j = new Joystick(0);

  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  DifferentialDrive drive = new DifferentialDrive(driveLeftSpark, driveRightSpark);
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //driveLeftSpark.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    //left_pid = driveLeftSpark.getPIDController();
    //left_encoder = driveLeftSpark.getEncoder();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(true);
    //driveLeftSparktwo.setInverted(true)
    driveRightSpark.setInverted(false);
    //driveRightSparktwo.setInverted(false);
    left_encoder = driveLeftSpark.getEncoder();
    right_encoder = driveRightSpark.getEncoder();
    left_pid = driveLeftSpark.getPIDController();
    right_pid = driveRightSpark.getPIDController();
    kP = 5e-4; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    maxVel = 2000; // rpm
    maxAcc = 1500;
    kPTurning=.05;

    // set PID coefficients
    left_pid.setP(kP);
    left_pid.setI(kI);
    left_pid.setD(kD);
    left_pid.setIZone(kIz);
    left_pid.setFF(kFF);
    left_pid.setOutputRange(kMinOutput, kMaxOutput);

    right_pid.setP(kP);
    right_pid.setI(kI);
    right_pid.setD(kD);
    right_pid.setIZone(kIz);
    right_pid.setFF(kFF);
    right_pid.setOutputRange(kMinOutput, kMaxOutput);
    int smartMotionSlot = 0;
    right_pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    right_pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    right_pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    right_pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    left_pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    left_pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    left_pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    left_pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
    SmartDashboard.putNumber("Set Angle",0);
    


    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */

  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftSpark.set(left);
    //driveLeftSparktwo.set(left);
    driveRightSpark.set(right);
    //driveRightSparktwo.set(right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  double heading= gyro.getAngle();;
  @Override

  public void autonomousInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    //driveLeftSparktwo.setIdleMode(IdleMode.kBrake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    //driveRightSparktwo.setIdleMode(IdleMode.kBrake);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);
  }

  double conversion = Math.PI/16;
  double setpoint=0;

  @Override
  public void autonomousPeriodic() {
    double error= heading- gyro.getAngle();
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    if((p != kP)) { right_pid.setP(p); kP = p; }
    if((i != kI)) { right_pid.setI(i); kI = i; }
    if((d != kD)) { right_pid.setD(d); kD = d; }
    if((iz != kIz)) { right_pid.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { right_pid.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      right_pid.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((p != kP)) { left_pid.setP(p); kP = p; }
    if((i != kI)) { left_pid.setI(i); kI = i; }
    if((d != kD)) { left_pid.setD(d); kD = d; }
    if((iz != kIz)) { left_pid.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { left_pid.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      left_pid.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { right_pid.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { right_pid.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { right_pid.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { right_pid.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    if((maxV != maxVel)) { left_pid.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { left_pid.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { left_pid.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { left_pid.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    double setPoint, processVariable, setPoint1, processVariable1;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    double kp = 1;
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      left_pid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = left_encoder.getVelocity();
      setPoint1 = SmartDashboard.getNumber("Set Velocity", 0);
      right_pid.setReference(setPoint1, CANSparkMax.ControlType.kVelocity);
      processVariable1 = right_encoder.getVelocity();
      drive.tankDrive( .1+ kp * error, .1 - kp * error);
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      setPoint1 = SmartDashboard.getNumber("Set Position", 0);

      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      left_pid.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = left_encoder.getPosition();
      right_pid.setReference(setPoint1, CANSparkMax.ControlType.kSmartMotion);
      processVariable1 = right_encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPointL", setPoint);
    SmartDashboard.putNumber("Process VariableL", processVariable);
    SmartDashboard.putNumber("OutputL", driveLeftSpark.getAppliedOutput());
    SmartDashboard.putNumber("SetPointR", setPoint1);
    SmartDashboard.putNumber("Process VariableR", processVariable1);
    SmartDashboard.putNumber("OutputR", driveRightSpark.getAppliedOutput());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
  
    double speedx = 0.6;
    

    //SmartDashboard.putNumber("Speed", left_encoder.getVelocity()/8);
    SmartDashboard.putNumber("Right Position:", right_encoder.getPosition());
    SmartDashboard.putNumber("Left Position:", left_encoder.getPosition());

    //(right_encoder.getPosition()+left_encoder.getPosition()<25)

    // if (m_autoSelected == kNothingAuto) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    //   return;
    // }

    // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // if (timeElapsed < ARM_EXTEND_TIME_S) {
    //   setArmMotor(ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
    //   setArmMotor(-ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    // } else {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLeftSpark.setIdleMode(IdleMode.kCoast);
    //driveLeftSparktwo.setIdleMode(IdleMode.kCoast);
    driveRightSpark.setIdleMode(IdleMode.kCoast);
    //driveRightSparktwo.setIdleMode(IdleMode.kCoast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    if(j.getRawButton(3)){

    }
    /*double armPower;
    if (j.getRawButton(7)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (j.getRawButton(5)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    */
    
    /* 
    double intakePower;
    int intakeAmps;
    
    if (j.getRawButton(8)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (j.getRawButton(6)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);
    */

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */



    
    setDriveMotors(-j.getRawAxis(1)*.2, -j.getRawAxis(5)*.5);

  }
}