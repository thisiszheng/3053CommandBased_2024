package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  
  // Climbing Arm Motors
  public static final int leftMotorID = 15;
  public static final int rightMotorID = 14;
  public CANSparkMax ClimberLeft = new CANSparkMax(leftMotorID, MotorType.kBrushless);
  public CANSparkMax ClimberRight = new CANSparkMax(rightMotorID, MotorType.kBrushless);

  // Motor Output Limit
  public double climbSpeedLimit = 0.3;

  // PS4 Joystick
  public Joystick driverJoystick = new Joystick(0);

  public Climber() {
    // Climbing Keybind + Power (remember to have the final output to be max -0.3 to 0.3)
    double Push = driverJoystick.getRawAxis(2) * climbSpeedLimit;
    double Pull = driverJoystick.getRawAxis(3) * climbSpeedLimit;

    // Climbing Arm Calculation
    double climbSpeed = Pull - Push; // when ClimbingArm is extending, Pull = 1

    // Climbing Arm Setting
    ClimberRight.set(0 + climbSpeed);
    ClimberLeft.set(0 + climbSpeed);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {}

}
