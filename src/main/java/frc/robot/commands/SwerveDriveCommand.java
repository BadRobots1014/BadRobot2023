package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command{
    
    public final SwerveSubsystem swerveSubsystem;
    public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    public final Supplier<Boolean> fieldOrientedFunction;
    public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveDriveCommand(SwerveSubsystem subsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> turnSupplier, Supplier<Boolean> fieldOriented) {
        swerveSubsystem = subsystem;
        xSpdFunction = xSupplier;
        ySpdFunction = ySupplier;
        turningSpdFunction = turnSupplier;
        fieldOrientedFunction = fieldOriented;
        xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
        yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        //Get inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //Death
        xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

        //Slew soup
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleMaxMetersPerSec;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleMaxMetersPerSec;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleMaxRadiansPerSec;

        //I am speed
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            //Field oriented
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
        else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //Divide and conker
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Actually do the thing*/
        swerveSubsystem.setModuleStates(moduleStates);

        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
