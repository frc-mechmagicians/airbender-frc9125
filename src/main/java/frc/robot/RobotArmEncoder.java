package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotArmEncoder extends TimedRobot {

  static final int ARM_CURRENT_LIMIT_A = 30;
  static final double ARM_OUTPUT_POWER = 0.4;

  // Fix the Arm Position values from Encoder pos values after testing
  static final double ARM_CONE_MID_POSITION = 0.0;
  static final double ARM_CONE_TOP_POSITION = 0.0;
  static final double ARM_CUBE_MID_POSITION = 0.0;
  static final double ARM_CUBE_TOP_POSITION = 0.0;

  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  RelativeEncoder armEncoder = arm.getEncoder(); // Need to check whether it is Relative or Absolute
  Joystick j = new Joystick(1);
  Joystick j1 = new Joystick(0);
  double armEncoderPosition = 0; 
  double autonomousStartTime;

  @Override
  public void robotInit() {

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
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
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousInit() {
    armEncoderPosition = armEncoder.getPosition();
    SmartDashboard.putNumber("ArmEncoderPos", armEncoderPosition);
    autonomousStartTime = Timer.getFPGATimestamp();
  }
  
  @Override
  public void autonomousPeriodic() { // If it is middle auton
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
  }

  double previous_y = j.getRawAxis(1);

  @Override
  public void teleopInit() {
    armEncoderPosition = armEncoder.getPosition();
    SmartDashboard.putNumber("ArmEncoderPos", armEncoderPosition);
    previous_y = j.getRawAxis(1);
    // Map the Joystic buttons to ARM position for Cone & Cube mid and top positions

  }

  @Override
  public void teleopPeriodic() {

    // Fix - Set the Arm Postion depending on the joystic button. The position is calculated from Encode Position.

    if(j1.getRawButton(8)){ //ARM_CONE_MID_POSITION
      setArmMotor(.3);
      armEncoderPosition = armEncoder.getPosition();
      SmartDashboard.putNumber("ArmEncoderPos", armEncoderPosition);
      // armEncoder.setPosition(ARM_CONE_MID_POSITION);
    }
    setArmMotor(j1.getRawAxis(1)*.3);
  
  }
}
