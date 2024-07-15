package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeShooter intakeShooter;
  private final double intakeShootSpeed;

  public IntakeShoot(IntakeShooter intakeShooter, double intakeShootSpeed) {
    this.intakeShooter = intakeShooter;
    this.intakeShootSpeed = intakeShootSpeed;
    addRequirements(intakeShooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeShooter.setMotor(intakeShootSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeShooter.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
