package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber climber;
  private final double climbSpeed;

  public Climb(Climber climber, double climbSpeed) {
    this.climber = climber;
    this.climbSpeed = climbSpeed;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.setMotor(climbSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
