package frc.robot.subsystems;

/*** FRC-Related Libraries ***/

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.motorcontrol.Spark <-- not in use currently
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveTrain extends SubsystemBase{
    /*---Part to ID Assignments---*/
    // DriveTrain Motors
    public VictorSPX RightFront = new VictorSPX(0);
    public VictorSPX RightRear = new VictorSPX(1);
    public VictorSPX LeftFront = new VictorSPX(2);
    public VictorSPX LeftRear = new VictorSPX(3);

    // Shooting Wheel Motors
    public TalonSRX RightShooterWheel = new TalonSRX(4);
    public TalonSRX LeftShooterWheel = new TalonSRX(7);

    // Shooting Arm Motors
    public TalonSRX RightShooterArm = new TalonSRX(5);
    public TalonSRX LeftShooterArm = new TalonSRX(6);

    // PS4 Joystick
    public Joystick driverJoystick = new Joystick(0);

    // Climbing Arm Motors
    public static final int leftMotorID = 15;
    public static final int rightMotorID = 14;
    public CANSparkMax ClimberLeft = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    public CANSparkMax ClimberRight = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    
    /***** Key-Binding Assignments *****/
    
    // DriveTrain Varible & Control
    double turn = driverJoystick.getRawAxis(0) * 0.3;
    double speed = -driverJoystick.getRawAxis(5) * 0.3;

    // Climbing Arm Varible & Control
    double Pull = driverJoystick.getRawAxis(3) * 0.3;
    double Push = driverJoystick.getRawAxis(2) * 0.3;

    // Shooting Arm Varible & Control
    double Raise = driverJoystick.getRawButton(2) ? 0.2: 0;
    double Lower = driverJoystick.getRawButton(4) ? 0.2: 0;

    // Shooting Wheel Variable & Calculation
    double Intake = driverJoystick.getRawButton(1) ? 0.5: 0;
    double Outtake = driverJoystick.getRawButton(3) ? 1: 0;

    /***** Calcuations *****/

    // DriveTrain Calcuation
    double left = speed + turn;
    double right = speed - turn;

    // Climbing Arm Calculation
    double climbSpeed = Pull - Push; // when ClimbingArm is extending, Pull = 1

    // Shooting Arm Calculation
    double ArmSpeed = Raise - Lower;

    // Shooting Wheel Calculation
    double shootingSpeed = Intake - Outtake;

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

    // Climbing Arm Setting
    ClimberRight.set(0 + climbSpeed);
    ClimberLeft.set(0 + climbSpeed);

    // Shooting Arm Setting
    RightShooterArm.set(ControlMode.PercentOutput, 0 + ArmSpeed);
    LeftShooterArm.set(ControlMode.PercentOutput, 0 + ArmSpeed);

    // Shooting Wheel Setting
    
    RightShooterWheel.set(ControlMode.PercentOutput, 0 + shootingSpeed);
    LeftShooterWheel.set(ControlMode.PercentOutput, 0 + shootingSpeed);
}
} 