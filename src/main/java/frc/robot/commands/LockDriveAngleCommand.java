package frc.robot.commands;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class LockDriveAngleCommand extends Command{
    private DriveSubsystem m_subsystem;

    public LockDriveAngleCommand(DriveSubsystem subsystem)
    {
        m_subsystem = subsystem;
    }

    double xSpeed;
    double ySpeed;

    @Override
    public void initialize() {
        xSpeed = -MathUtil.applyDeadband(m_subsystem.Controller.getLeftY(), OIConstants.kDriveDeadband);
        ySpeed = -MathUtil.applyDeadband(m_subsystem.Controller.getLeftX(), OIConstants.kDriveDeadband);
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        //m_subsystem.drive(
        //        xSpeed,
        //        ySpeed,
        //        -MathUtil.applyDeadband(m_subsystem.Controller.getRightX(), OIConstants.kDriveDeadband),
        //        true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}