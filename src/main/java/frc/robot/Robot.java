package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;



public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kLeftAuto = "left";
  private static final String kMiddleAuto = "middle";
  private static final String kRightAuto = "right";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  

  // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  static final double GYRO_CONTROLLED_MAX_OUTPUT = 0.10; // tune it (.25 to .5)
  static final double GYRO_TU = 1.0;  // Oscilatoin period - Tune it (0.5 to 2)
  static final double GYRO_KU = 1.0/15.0;  // ultimate gain at max angle 15
  static final double GYRO_TOLERANCE = 2.0; // Tolerance angle 

  // PI
  // static final double GYRO_KP = 0.45*GYRO_KU;
  // static final double GYRO_KI = 0.54*GYRO_KU/GYRO_TU;
  // static final double GYRO_KD = 0;

  // PD
  static final double GYRO_KP = 0.8*GYRO_KU;
  static final double GYRO_KI = 0;
  static final double GYRO_KD = 0.14*GYRO_KU*GYRO_TU;

  // Classis PID
  //static final double GYRO_KP = 0.6*GYRO_KU;
  //static final double GYRO_KI = 1.2*GYRO_KU/GYRO_TU;
  //static final double GYRO_KD = 0.075*GYRO_KU*GYRO_TU;
  
  // Passen Integral Rule
  //static final double GYRO_KP = 0.7*GYRO_KU;
  //static final double GYRO_KI = 1.75*GYRO_KU/GYRO_TU;
  //static final double GYRO_KD = 0.105*GYRO_KU*GYRO_TU

  // Some vershoot
  //static final double GYRO_KP = 0.33*GYRO_KU;
  //static final double GYRO_KI = 0.66*GYRO_KU/GYRO_TU;
  //static final double GYRO_KD = 0.11*GYRO_KU*GYRO_TU

  // No overshoot
  //static final double GYRO_KP = 0.2*GYRO_KU;
  //static final double GYRO_KI = 0.4*GYRO_KU/GYRO_TU;
  //static final double GYRO_KD = 0.066*GYRO_KU*GYRO_TU

  private final PIDController npid = new PIDController(GYRO_KP, GYRO_KI, GYRO_KD);
  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightSpark = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveLeftSpark2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveRightSpark2 = new CANSparkMax(4, MotorType.kBrushless);
  
  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftSpark,driveLeftSpark2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightSpark, driveRightSpark2);

  ADIS16470_IMU gyro = new ADIS16470_IMU();

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

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
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);
  
  public RelativeEncoder armEncoder = arm.getEncoder();
  public RelativeEncoder motorEncoder = driveLeftSpark.getEncoder();
  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  XboxController driverJoystick = new XboxController(0);
  PS4Controller operatorJoystick = new PS4Controller(1);
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 30;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 30;

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
  static int autoStage = 0;
  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 1.5;

  static double DROP_TIME_X=0;
  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.25;
  static final double ARM_MIDDLE_POS = 18;
  static final double ARM_END_POS = 30;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.35;
  static final double AUTO_FORWARD_DISTANCE = 5;
  static double autoBackupPosition = -30.0;

  /**
   * This method is run once when the robot is first started up.
   */

  @Override
  public void robotInit() {
    autoStage=0;
    armEncoder.setPosition(0);
    m_chooser.setDefaultOption("Left", kLeftAuto);
    m_chooser.addOption("Middle", kMiddleAuto);
    m_chooser.addOption("Right", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    CameraServer.startAutomaticCapture(0);
    
    /*
     * You will need to change some of these from false to true.
     * 
     * In the drive.curvatureDrive(0, timeElapsed, Encoder_Reset); method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(false);
    driveLeftSpark2.setInverted(false);
    driveRightSpark.setInverted(true);
    driveRightSpark2.setInverted(true);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
    drive.setDeadband(0);


    npid.setTolerance(GYRO_TOLERANCE);
    gyro_init_angle = gyro.getYComplementaryAngle();
    SmartDashboard.putNumber("Gyro Initial Angle", gyro_init_angle);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    if(percent>0){
      if(armEncoder.getPosition()<ARM_END_POS){
        arm.set(percent);
      }else if(armEncoder.getPosition()<ARM_END_POS+6){
        arm.set(.1);
      }else{
        arm.set(0);
      }
    }else{
      arm.set(percent);
    }
    
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps    current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("arm position",armEncoder.getPosition());
    
    if (armEncoder.getPosition() < 0) {
      armEncoder.setPosition(0);
    }
  }

  double autonomousStartTime;
  double autonomousIntakePower;
  

  static double gyro_init_angle = 0.0;
  public void levelRobot() {
    double speed = npid.calculate(gyro.getYComplementaryAngle(), gyro_init_angle);
    drive.setMaxOutput(GYRO_CONTROLLED_MAX_OUTPUT);
    setArmMotor(ARM_OUTPUT_POWER/4);
    drive.curvatureDrive(speed, 0, true);
    SmartDashboard.putNumber("Gyro controllred Speed", speed);
  }

  double robotPosition = 0;
  int robotStallInterval = 0;
  @Override
  public void autonomousInit() { 
    motorEncoder.setPosition(0);
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    driveLeftSpark2.setIdleMode(IdleMode.kBrake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    driveRightSpark2.setIdleMode(IdleMode.kBrake);
    robotPosition = 0;
    robotStallInterval = 0;

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    autonomousIntakePower = INTAKE_OUTPUT_POWER;

    if (m_autoSelected == kMiddleAuto) {
      autoBackupPosition = -20;
    } else {
      autoBackupPosition = -60;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
    autoStage = 0;
  }

  

  @Override
  public void autonomousPeriodic() { // If it is middle auton
    SmartDashboard.putNumber("drive position",motorEncoder.getPosition());
    SmartDashboard.putNumber("curr angle",gyro.getYComplementaryAngle());
   

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    // move forward
    // raise arm fully
    // turn intake motor drop
    // pull arm back
    // move backward

    /*
     * extend arm
     * pick the object
     * bring down arm ?
     * move forward
     * extend arm
     * release object
     * bring down arm ?
     * move backward
     * 
     */
    // extend arm out for two seconds
    //System.out.println(timeElapsed);
 
    switch (autoStage) {
      case 0:
        if (timeElapsed < ARM_EXTEND_TIME_S) {
          setArmMotor(-ARM_OUTPUT_POWER);
          setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
          drive.curvatureDrive(0.0, 0.0, false);
          // stop moving arm
          // if time is less that time it takes for arm to extend + time it takes to throw
          // piece
          // use the intake
        } else if (robotStallInterval < 5) {
          double curPos = motorEncoder.getPosition();
          if (curPos - robotPosition < 0.00001) {
            robotStallInterval++;
            System.out.println("Motor Encoder, Robot Position "+ motorEncoder.getPosition()+" "+robotPosition);
          } else {
            robotStallInterval = 0;
          }
  
          robotPosition=motorEncoder.getPosition();
          armEncoder.setPosition(0);
          setArmMotor(-ARM_OUTPUT_POWER/2);
          setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
          drive.curvatureDrive(-AUTO_DRIVE_SPEED / 4, 0.0,false);
        } else {
          autoStage++;
          autonomousStartTime = Timer.getFPGATimestamp();
          timeElapsed = 0;
        }
        break;

    case 1: // Throwing  piece and moving back to start position
        if (timeElapsed <  AUTO_THROW_TIME_S) {
          setArmMotor(-ARM_OUTPUT_POWER/2);
          setIntakeMotor(INTAKE_OUTPUT_POWER / 2, INTAKE_CURRENT_LIMIT_A);
          drive.curvatureDrive(0.0, 0.0,false);
        } else if (motorEncoder.getPosition() > 0) {
          setArmMotor(0.0);
          setIntakeMotor(0, 0);
          drive.curvatureDrive(AUTO_DRIVE_SPEED / 4, 0.0,false);
        } else {
          autoStage++;
          autonomousStartTime = Timer.getFPGATimestamp();
          timeElapsed = 0;
        }
        break;
        
    case 2: // Retracting arm and move out of community
        if (armEncoder.getPosition()<ARM_END_POS) {
           setArmMotor(ARM_OUTPUT_POWER);
           setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
           drive.curvatureDrive(0, 0, false);
          // drive backward
        } else if (motorEncoder.getPosition() > autoBackupPosition) { 
          setArmMotor(0.0);
          setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
          drive.curvatureDrive(0,0,false);
        } else {
          autoStage++;
          autonomousStartTime = Timer.getFPGATimestamp();
          timeElapsed = 0;
        }
        break;
    case 3: // charge station
        if (m_autoSelected != kMiddleAuto) {
          setArmMotor(0.0);
          setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
          drive.curvatureDrive(0.0, 0.0,false);
        } else {
          levelRobot();
        }
        break;
    }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;
  double intakePower = 0;
  int intakeAmps = 0;
  double OpenLoopRamp=0.25;
  @Override
  public void teleopInit() {
    motorEncoder.setPosition(0);
    driveLeftSpark.setIdleMode(IdleMode.kCoast);
    driveLeftSpark2.setIdleMode(IdleMode.kCoast);
    driveRightSpark.setIdleMode(IdleMode.kCoast);
    driveRightSpark2.setIdleMode(IdleMode.kCoast);
    driveLeftSpark.setOpenLoopRampRate(OpenLoopRamp);
   driveLeftSpark2.setOpenLoopRampRate(OpenLoopRamp);
    driveRightSpark.setOpenLoopRampRate(-OpenLoopRamp);
    driveRightSpark2.setOpenLoopRampRate(-OpenLoopRamp);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("drive position",motorEncoder.getPosition());

    // Arm control
    if (armEncoder.getPosition() < 0) {
      armEncoder.setPosition(0);
    }
    double currTime=Timer.getFPGATimestamp();
    if(currTime-DROP_TIME_X<3 && currTime-DROP_TIME_X>1) {
      setArmMotor(ARM_OUTPUT_POWER);
    } else if (driverJoystick.getRightTriggerAxis() != 0) {
      setArmMotor(ARM_OUTPUT_POWER);
    } else if(operatorJoystick.getCrossButton()) {
      if(armEncoder.getPosition()>ARM_MIDDLE_POS+1){
        setArmMotor(-ARM_OUTPUT_POWER/2);
      } else if(armEncoder.getPosition()<ARM_MIDDLE_POS-1){
        setArmMotor(ARM_OUTPUT_POWER/2);
      }
    } else if(armEncoder.getPosition() < ARM_MIDDLE_POS/2 &&
         (operatorJoystick.getCircleButton() || operatorJoystick.getTriangleButton())){
      setArmMotor(-ARM_OUTPUT_POWER);
    } else if (operatorJoystick.getLeftY() == 0) {
      setArmMotor(-.05);
    } else {
      setArmMotor(operatorJoystick.getLeftY() * ARM_OUTPUT_POWER);
    }

    // Intake control
    if (operatorJoystick.getCircleButtonPressed()) {
      setArmMotor(-ARM_OUTPUT_POWER);
      // cube in
      intakePower = INTAKE_OUTPUT_POWER / 2;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (operatorJoystick.getCircleButtonReleased()) {
      intakePower = 0;
      intakeAmps = 0;
    } else if (operatorJoystick.getTriangleButtonPressed()) {
      // cone in
      setArmMotor(-ARM_OUTPUT_POWER);
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
      
    } else if (operatorJoystick.getTriangleButtonReleased()) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (operatorJoystick.getSquareButtonPressed()) {
      if (lastGamePiece == CUBE) {
        intakePower = -INTAKE_OUTPUT_POWER / 2;
        intakeAmps = INTAKE_CURRENT_LIMIT_A;
      } else if (lastGamePiece == CONE) {
        intakePower = INTAKE_OUTPUT_POWER / 2;
        intakeAmps = INTAKE_CURRENT_LIMIT_A;
      }
    } else if (operatorJoystick.getSquareButtonReleased()) {
      DROP_TIME_X = Timer.getFPGATimestamp();
      intakeAmps = 0;
      intakePower = 0;
    }

      
    setIntakeMotor(intakePower, intakeAmps);

    if(driverJoystick.getBButton()){
      driveLeftSpark.setIdleMode(IdleMode.kBrake);
   driveLeftSpark2.setIdleMode(IdleMode.kBrake);
      driveRightSpark.setIdleMode(IdleMode.kBrake);
      driveRightSpark2.setIdleMode(IdleMode.kBrake);
    }
    if(driverJoystick.getBButtonReleased()){
     driveLeftSpark.setIdleMode(IdleMode.kCoast);
      driveLeftSpark2.setIdleMode(IdleMode.kCoast);
      driveRightSpark.setIdleMode(IdleMode.kCoast);
      driveRightSpark2.setIdleMode(IdleMode.kCoast);
    }
    // Drive control
    if(DROP_TIME_X!=0 && Timer.getFPGATimestamp()-DROP_TIME_X<1){
      drive.setMaxOutput(0.5);
      drive.curvatureDrive(AUTO_DRIVE_SPEED / 2, 0,false);
    } else if (driverJoystick.getBButton()) {
      levelRobot();
    } else {
      if (driverJoystick.getLeftTriggerAxis() != 0) {
        drive.setMaxOutput(.25);
      } else if (driverJoystick.getLeftBumper()) {
        drive.setMaxOutput( .45);
      }else if (driverJoystick.getRightTriggerAxis() != 0) {
        drive.setMaxOutput(1);
      } else {
        drive.setMaxOutput(0.5);
      }
      var xSpeed = m_speedLimiter.calculate(-driverJoystick.getLeftY());
      final var rot = m_rotLimiter.calculate(-driverJoystick.getRightX());

      drive.curvatureDrive(xSpeed, rot, driverJoystick.getLeftBumper());
    }
  }
}
