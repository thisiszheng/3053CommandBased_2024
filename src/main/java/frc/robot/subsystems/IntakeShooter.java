package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {

  // Shooting Wheel Motors
  public TalonSRX RightShooterWheel = new TalonSRX(4);
  public TalonSRX LeftShooterWheel = new TalonSRX(7);
  // Shooting Arm Motors
  public TalonSRX RightShooterArm = new TalonSRX(5);
  public TalonSRX LeftShooterArm = new TalonSRX(6);

  // PS4 Joystick
  public Joystick driverJoystick = new Joystick(0);

  // Shooting Arm Varible & Control
  double Raise = driverJoystick.getRawButton(2) ? 0.2: 0;
  double Lower = driverJoystick.getRawButton(4) ? 0.2: 0;
  // Shooting Wheel Variable & Calculation
  double Intake = driverJoystick.getRawButton(1) ? 0.5: 0;
  double Outtake = driverJoystick.getRawButton(3) ? 1: 0;

  // Shooting Arm Calculation
  double ArmSpeed = Raise - Lower;
  // Shooting Wheel Calculation
  double shootingSpeed = Intake - Outtake;

  public IntakeShooter() {
    // Shooting Arm Setting
    RightShooterArm.set(ControlMode.PercentOutput, 0 + ArmSpeed);
    LeftShooterArm.set(ControlMode.PercentOutput, 0 + ArmSpeed);

    // Shooting Wheel Setting
    RightShooterWheel.set(ControlMode.PercentOutput, 0 + shootingSpeed);
    LeftShooterWheel.set(ControlMode.PercentOutput, 0 + shootingSpeed);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

}
