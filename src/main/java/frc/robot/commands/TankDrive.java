package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDrive extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain driveTrain;
    private final double driveSpeed;

    public TankDrive(DriveTrain driveTrain, double driveSpeed) {
    this.driveTrain = driveTrain;
    this.driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {}
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

