package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase{

    //Modules
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftEncoderCanId,
        DriveConstants.kFrontLeftChassisAngularOffset,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightEncoderCanId,
        DriveConstants.kFrontRightChassisAngularOffset,
        DriveConstants.kFrontRightAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kRearLeftEncoderCanId,
        DriveConstants.kBackLeftChassisAngularOffset,
        DriveConstants.kBackLeftAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kRearRightEncoderCanId,
        DriveConstants.kBackRightChassisAngularOffset,
        DriveConstants.kBackRightAbsoluteEncoderReversed
    );

    // The gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    //Shuffleboard
    private final ShuffleboardTab m_tab;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(DriveConstants.kBootupDelay);
                zeroHeading();
            }
            catch (Exception e) {}
        }).start();
        m_tab = Shuffleboard.getTab("Swerve");
        m_tab.addNumber("Heading", this::getHeading);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * @param desiredStates The states the modules should move toward. In order, front left, front right, back left, back right.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
