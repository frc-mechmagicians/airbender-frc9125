package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotEncoder extends TimedRobot {

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
  RelativeEncoder left_encoder, right_encoder;
  

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
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(true);
    driveRightSpark.setInverted(false);
    left_encoder = driveLeftSpark.getEncoder();
    right_encoder = driveRightSpark.getEncoder();
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);


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

    // display PID coefficients on SmartDashboard
    // display Smart Motion coefficients

    // button to toggle between velocity and smart motion modes

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;



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
    SmartDashboard.putNumber("RPMR", right_encoder.getVelocity());
    SmartDashboard.putNumber("RPML", left_encoder.getVelocity());
    SmartDashboard.putNumber("Right Position:", right_encoder.getPosition());
    SmartDashboard.putNumber("Left Position:", left_encoder.getPosition());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

    
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
   
    double speedx = 0.2;
    double position = 84.0;

    //SmartDashboard.putNumber("Speed", left_encoder.getVelocity()/8);
    

     if ((right_encoder.getPosition()+left_encoder.getPosition())/2 < position/(3*Math.PI/4)) {
      driveLeftSpark.set(speedx);
      driveRightSpark.set(speedx);
     }
     else{
      driveLeftSpark.set(0.0);
      driveRightSpark.set(0.0);
      //driveLeftSparktwo.set(0.0);
      //driveRightSparktwo.set(0.0);
    }

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
    setDriveMotors(-j.getRawAxis(1)*.2, -j.getRawAxis(5)*.5);

  }
}