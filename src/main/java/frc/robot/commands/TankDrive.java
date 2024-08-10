package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDrive extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    DriveTrain driveTrain;
    double driveSpeed;

    public TankDrive(DriveTrain driveTrain, double driveSpeed) {
    this.driveTrain = driveTrain;
    this.driveSpeed = driveSpeed;
    addRequirements(driveTrain);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        driveTrain.setMotor(driveSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        driveTrain.setMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

