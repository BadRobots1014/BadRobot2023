package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveModule;

public class SwerveSubsystem extends SubsystemBase{

    public SwerveSubsystem(XboxController controller)
    {
        this.Controller = controller;
        m_driveTab = Shuffleboard.getTab("Swerve");
        m_driveTab.addDouble("frontRightAngle", () -> frontRight.getState().angle.getDegrees());
        m_driveTab.addDouble("frontLeftAngle", () -> frontLeft.getState().angle.getDegrees());
        m_driveTab.addDouble("rearRightAngle", () -> backRight.getState().angle.getDegrees());
        m_driveTab.addDouble("rearLeftAngle", () ->  backLeft.getState().angle.getDegrees());

        m_driveTab.addDouble("frontRightSpeed", () -> frontRight.getState().speedMetersPerSecond);
        m_driveTab.addDouble("frontLeftSpeed", () -> frontLeft.getState().speedMetersPerSecond);
        m_driveTab.addDouble("rearRightSpeed", () -> backRight.getState().speedMetersPerSecond);
        m_driveTab.addDouble("rearLeftSpeed", () -> backLeft.getState().speedMetersPerSecond);

        m_driveTab.addDouble("controllerLeftX", () -> Controller.getLeftX());
        m_driveTab.addDouble("controllerLeftY", () -> Controller.getLeftY());

        m_driveTab.addDouble("controllerRightX", () -> Controller.getRightX());
        m_driveTab.addDouble("controllerRightY", () -> Controller.getRightY());
    }
    public XboxController Controller;
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
    private final ShuffleboardTab m_driveTab;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(DriveConstants.kBootupDelay);
                zeroHeading();
            }
            catch (Exception e) {}
        }).start();
        m_driveTab = Shuffleboard.getTab("Swerve");
        m_driveTab.addNumber("Heading", this::getHeading);
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

    public void testMotor()
    {
        double rx = Controller.getRightX();
        double ry = Controller.getRightY();
        double lx = Controller.getLeftX();
        double ly = Controller.getLeftY();

        if (Controller.getBackButton())
        {
            frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
            backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
        }
            
        double speedMultiplyer = !Controller.getRightBumper() ? 1 : .3;

        SwerveModule module = null;

        if (Controller.getYButton())
            module = frontRight;
        else if (Controller.getXButton())
            module = frontLeft;
        else if (Controller.getBButton())
            module = backRight;
        else if (Controller.getAButton())
            module = backLeft;
        
        if (module == null)
        {
            frontLeft.setDesiredState(new SwerveModuleState(Controller.getLeftY() * speedMultiplyer, Rotation2d.fromRotations(.5 * Controller.getLeftX())));
            frontRight.setDesiredState(new SwerveModuleState(Controller.getLeftY() * speedMultiplyer, Rotation2d.fromRotations(.5 * Controller.getLeftX())));
            backLeft.setDesiredState(new SwerveModuleState(Controller.getLeftY() * speedMultiplyer, Rotation2d.fromRotations(.5 * Controller.getLeftX())));
            backRight.setDesiredState(new SwerveModuleState(Controller.getLeftY() * speedMultiplyer, Rotation2d.fromRotations(.5 * Controller.getLeftX())));
            return;
        }

        frontLeft.setDesiredState(new SwerveModuleState());
        frontRight.setDesiredState(new SwerveModuleState());
        backLeft.setDesiredState(new SwerveModuleState());
        backRight.setDesiredState(new SwerveModuleState());
        
        module.setDesiredState(new SwerveModuleState(Controller.getLeftY() * speedMultiplyer, Rotation2d.fromRotations(.5 * Controller.getLeftX())));
    }//state.angle.getDegrees() + Controller.getLeftX() * 10
}
