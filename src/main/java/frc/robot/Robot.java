package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotAuton extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftSpark, driveLeftSpark2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightSpark, driveRightSpark2);

  

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

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  Joystick j = new Joystick(1);
  Joystick j1 = new Joystick(0);

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
  // static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  // static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  // static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  // static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 1.5;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.5;

  /**
   * Time to drive back in auto
   */
  static double AUTO_DRIVE_TIME = 3.0;

  /**
   * Speed to drive backwards in auto
   */
   static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Left", kNothingAuto);
    m_chooser.addOption("Middle", kConeAuto);
    m_chooser.addOption("Right", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
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
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

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
    driveLeftSpark2.set(left);
    driveRightSpark.set(right);
    driveRightSpark2.set(right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
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
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    driveLeftSpark2.setIdleMode(IdleMode.kBrake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    driveRightSpark2.setIdleMode(IdleMode.kBrake);;

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    autonomousIntakePower = INTAKE_OUTPUT_POWER;

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  
  static final int INTAKE_CURRENT_LIMIT_A = 30;
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
  static final double INTAKE_OUTPUT_POWER = 1.0;
  static final double INTAKE_HOLD_POWER = 0.07;

  static double AUTO_BACK_TIME = 0.0;
  static double AUTO_WAIT_TIME = 0.0;
  static double SCORE_TIME = 1;


  @Override
  public void autonomousPeriodic() { // If it is middle auton
    if (m_autoSelected == kConeAuto) {
        System.out.println("Middle");
      AUTO_DRIVE_TIME = 3.0;
      AUTO_BACK_TIME = 2.0;
      AUTO_WAIT_TIME = 1.0;
    }
    if (m_autoSelected == kNothingAuto) {
        System.out.println("Left"); // If it is left auton
        AUTO_DRIVE_TIME = 3.0; 
        AUTO_BACK_TIME = 0.0;
        AUTO_WAIT_TIME = 0.0;
    }
    if (m_autoSelected == kCubeAuto) {
        System.out.println("Right"); // If it is right auton
        AUTO_DRIVE_TIME = 0.0;
        AUTO_BACK_TIME = 0.0;
        AUTO_WAIT_TIME = 0.0;
    }

    System.out.println(AUTO_DRIVE_TIME);
    System.out.println(AUTO_WAIT_TIME);
    System.out.println(AUTO_BACK_TIME);


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
    System.out.println(timeElapsed);
    if (timeElapsed < ARM_EXTEND_TIME_S) {
        System.out.println(true);
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(-20.0, INTAKE_HOLD_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    // stop moving arm
    // if time is less that time it takes for arm to extend + time it takes to throw piece
    // use the intake
    } else if (timeElapsed < ARM_EXTEND_TIME_S + SCORE_TIME) {
        setArmMotor(0.0);
        setIntakeMotor(-20.0, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(-AUTO_DRIVE_SPEED / 4, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S  + SCORE_TIME + AUTO_THROW_TIME_S) {
      setArmMotor(0.0);
      setIntakeMotor(20.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
      //bring arm down
    } else if (timeElapsed < ARM_EXTEND_TIME_S + SCORE_TIME + SCORE_TIME + AUTO_THROW_TIME_S) {
        setArmMotor(0.0);
        setIntakeMotor(-20.0, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(AUTO_DRIVE_SPEED / 4, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + SCORE_TIME + SCORE_TIME + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER); // bring down arm
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
      //drive backward
    } else if (timeElapsed < ARM_EXTEND_TIME_S + SCORE_TIME + SCORE_TIME + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0); 
    } else if (timeElapsed < ARM_EXTEND_TIME_S + SCORE_TIME + SCORE_TIME + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME + AUTO_BACK_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0); 
    } else if (timeElapsed < ARM_EXTEND_TIME_S+ SCORE_TIME  + SCORE_TIME + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME + AUTO_BACK_TIME + AUTO_WAIT_TIME) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0); 
    } else if (timeElapsed < ARM_EXTEND_TIME_S  + SCORE_TIME + SCORE_TIME + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME + AUTO_BACK_TIME + AUTO_WAIT_TIME + AUTO_BACK_TIME) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(-AUTO_DRIVE_SPEED, 0.0); 
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  double previous_y = j.getRawAxis(1);

  @Override
  public void teleopInit() {
    driveLeftSpark.setIdleMode(IdleMode.kCoast);
    driveLeftSpark2.setIdleMode(IdleMode.kCoast);
    driveRightSpark.setIdleMode(IdleMode.kCoast);
    driveRightSpark2.setIdleMode(IdleMode.kCoast);

    lastGamePiece = NOTHING;

    previous_y = j.getRawAxis(1);
  }

  @Override
  public void teleopPeriodic() {
    if(j1.getRawButton(8)){
      setArmMotor(.3);
    }
    setArmMotor(j1.getRawAxis(1)*.3);
    double intakePower;
    int intakeAmps;
    if (j1.getRawButton(3) || j1.getRawButton(6)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (j1.getRawButton(5) || j1.getRawButton(4)) {
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

    
    /* 
    if(j.getRawButton(3)){
      setIntakeMotor(1, 20);
    }
    if(j.getRawButton(4)){
      setIntakeMotor(-1, 20);
    }
    
    */

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    double axis = 1;
    // SmartDashboard.putNumber("Previous", previous_y);
    // if (j.getRawAxis(1) > 0.25) {
    //   setDriveMotors(-j.getRawAxis(1)*.3, -j.getRawAxis(0)*.3);
    //   previous_y = j.getRawAxis(1);    
    // }
    // else {
    //   previous_y = previous_y-0.03;
    //   if (previous_y < 0) {
    //     previous_y = 0;
    //   }
    //   setDriveMotors(-previous_y*.3,-j.getRawAxis(0)*.3);
    // }
    if (j.getRawAxis(3) != 0) {
        drive.setMaxOutput(1);
    } else {

        drive.setMaxOutput(0.5);
    }
     var xSpeed = m_speedLimiter.calculate(-j.getRawAxis(1));
    final var rot = m_rotLimiter.calculate(-j.getRawAxis(4));
    if (rot != 0 && xSpeed == 0) {
        xSpeed = 0.01;
    }
    
    drive.curvatureDrive(xSpeed, rot, j.getRawButton(6));
  }
}
