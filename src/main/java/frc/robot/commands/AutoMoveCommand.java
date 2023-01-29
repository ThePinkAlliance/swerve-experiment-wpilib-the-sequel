package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoMoveCommand extends CommandBase {
    private final SwerveSubsystem driveSubsystem;
    private double veloXaxis = 0.0;
    private double veloYaxis = 0.0;
    private double radsPerSec = 0.0;

    
    public AutoMoveCommand(SwerveSubsystem driveSubsystem, double vx, double vy, double radsPerSec) {
        this.driveSubsystem = driveSubsystem;
        this.veloXaxis = vx;
        this.veloYaxis = vy;
        this.radsPerSec = radsPerSec;
        addRequirements(driveSubsystem);
        
    }

    @Override
    public void execute() {
        driveSubsystem.move(veloXaxis, veloYaxis, radsPerSec);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.move(0.0, 0.0, 0.0);
    }   
}
