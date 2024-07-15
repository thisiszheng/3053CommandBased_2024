// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeShoot;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Climb;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveTrain driveTrain = new DriveTrain();
  private final Climber climber = new Climber();
  private final IntakeShooter intakeShooter = new IntakeShooter();
  
  private final Joystick driverJoystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
    driveTrain.setDefaultCommand(
      new TankDrive(driveTrain, 0)
    );
    climber.setDefaultCommand(new Climb(climber, 0));
    intakeShooter.setDefaultCommand(new IntakeShoot(intakeShooter, 0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(driverJoystick, 3).whileTrue(new TankDrive(driveTrain, 1));
    new JoystickButton(driverJoystick, 4).whileTrue(new TankDrive(driveTrain, -1));
    new JoystickButton(driverJoystick, 5).whileTrue(new Climb(climber, 1));
    new JoystickButton(driverJoystick, 6).whileTrue(new Climb(climber, -1));
    new JoystickButton(driverJoystick, 7).whileTrue(new IntakeShoot(intakeShooter, 1));
    new JoystickButton(driverJoystick, 8).whileTrue(new IntakeShoot(intakeShooter, -1));
  }

  public Command getAutonomusCommand() {
    return null;
  }
}
