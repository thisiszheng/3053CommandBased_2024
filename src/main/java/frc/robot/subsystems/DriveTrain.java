package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.motorcontrol.Spark <-- not in use currently
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveTrain extends SubsystemBase{
    // DriveTrain Motors
    public VictorSPX RightFront = new VictorSPX(0);
    public VictorSPX RightRear = new VictorSPX(1);
    public VictorSPX LeftFront = new VictorSPX(2);
    public VictorSPX LeftRear = new VictorSPX(3);

    // PS4 Joystick
    public Joystick driverJoystick = new Joystick(0);
    
    // DriveTrain Keybind + Power
    double turn = driverJoystick.getRawAxis(0) * 0.3;
    double speed = -driverJoystick.getRawAxis(5) * 0.3;

    /***** Calcuations *****/

    // DriveTrain Calcuation
    double left = speed + turn;
    double right = speed - turn;

    /*--PID stuff (Still in Progress)--*/
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    /* Remove after use
    if((p != kP)) { SparkPIDController.setP(p); kP = p; }
    if((i != kI)) { SparkPIDController.setI(i); kI = i; }
    if((d != kD)) { SparkPIDController.setD(d); kD = d; }
    if((iz != kIz)) { SparkPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { SparkPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      SparkPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    /* Remove after use
    SparkPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    */

  // Encoders & PID
  /*
  RelativeEncoder climberEncoder;
  SparkPIDController SparkPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  */

public void periodic() {
    /***** Motor Power Settings *****/

    // DriveTrain Setting
    RightFront.set(ControlMode.PercentOutput, 0 + right);
    RightRear.set(ControlMode.PercentOutput, 0 + right);
    LeftFront.set(ControlMode.PercentOutput, 0 + left);
    LeftRear.set(ControlMode.PercentOutput, 0 + left);
}
} 