// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // DONE Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
    public static final double MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

    // Max acceleration needed for TrajectoryConfig constructor. Using value from 0
    // to Autonomous example
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0; // original line of code
    // public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.5;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    Pigeon2 gyroscope = new Pigeon2(DRIVETRAIN_PIGEON_ID);

    private final SwerveDriveOdometry odometry;

    // Field object to keep track of location of bot.
    private final Field2d field = new Field2d();

    // These are our modules. We initialize them in the constructor.
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveModuleState[] states;

    /**
     * Constructor for the Drivetrain 
     */
    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        SmartDashboard.putData("Field", field);
        // zeroGyroscope();

        frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, "CANivore")
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_STEER_MOTOR, "CANivore")
                .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER, "CANivore")
                // .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
                // .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
                // .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();

        frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, "CANivore")
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, "CANivore")
                .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, "CANivore")
                // .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                // .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
                // .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

        backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, "CANivore")
                .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_STEER_MOTOR, "CANivore")
                .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER, "CANivore")
                // .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
                // .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
                // .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

        backRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, "CANivore")
                .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_STEER_MOTOR, "CANivore")
                .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER, "CANivore")
                // .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
                // .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
                // .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(getHeading()), getPositions());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroscope.setYaw(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        // Don't Remove Follwoing if you are using a Pigeon
        // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
        return Rotation2d.fromDegrees(gyroscope.getYaw());
    }

    public Rotation2d getGyroscopePitch() {
        return Rotation2d.fromDegrees(gyroscope.getPitch());
    }

    public Rotation2d getGyroscopeRoll() {
        return Rotation2d.fromDegrees(gyroscope.getRoll());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
        states = kinematics.toSwerveModuleStates(m_chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    /**
     * Drive method that can take in SwerveModuleState array. Needed for auton since
     * we have to provide a consumer of SwerveModuleState[]
     * 
     * @param states the states of the swerve modules
     */
    public void drive(SwerveModuleState[] states) {
        if (SmartDashboard.getBoolean("Debug Swerve", false)) {
            System.out.println("In drive SwerveModule[0]: " + states[0]);
            System.out.println("In drive SwerveModule[1]: " + states[1]);
            System.out.println("In drive SwerveModule[2]: " + states[2]);
            System.out.println("In drive SwerveModule[3]: " + states[3]);
        }
        // this.states = states;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Pitch", gyroscope.getPitch());
        SmartDashboard.putNumber("Robot Yaw", gyroscope.getYaw());
        SmartDashboard.putNumber("Robot Roll", gyroscope.getRoll());
        SmartDashboard.updateValues();  // I'm not sure this is needed. 

        //need to update Odometry
        odometry.update(Rotation2d.fromDegrees(gyroscope.getYaw()), getPositions());

        // Trying update robot position on field image on dashboard
        field.setRobotPose(odometry.getPoseMeters());
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swervePositions = {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
        return swervePositions;
    }

    /**
     * Gets the current pose of the robot
     * 
     * @return
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // I'm 100% sure I have this method doing what we want. Need to test it.
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(),
                getPositions(),
                pose);
    }

    public void stopModules() {
        frontLeftModule.set(0, frontLeftModule.getSteerAngle());
        frontRightModule.set(0, frontRightModule.getSteerAngle());
        backLeftModule.set(0, backLeftModule.getSteerAngle());
        backRightModule.set(0, backRightModule.getSteerAngle());
    }

    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getYaw(), 360);
    }

    public double getPitch() {
        return gyroscope.getPitch();
    }

    public double getRoll() {
        return gyroscope.getRoll();
    }

    public void driveStraightSlow() {
        frontLeftModule.set(2, 0);
        frontRightModule.set(2, 0);
        ;
        backLeftModule.set(2, 0);
        ;
        backRightModule.set(2, 0);
    }

    public void driveBackwardsSlow() {
        frontLeftModule.set(-2, 0);
        frontRightModule.set(-2, 0);
        ;
        backLeftModule.set(-2, 0);
        ;
        backRightModule.set(-2, 0);
    }

    public void sturdyBase() {
        backRightModule.set(0, 45);
        ;
        backLeftModule.set(0, -45);
        ;
        frontRightModule.set(0, -45);
        ;
        frontLeftModule.set(0, 45);
        ;
    }

    /**
     * Gets the Kinematics of the drivetrain
     * 
     * @return
     */
    public SwerveDriveKinematics geKinematics() {
        return kinematics;
    }

    // Assuming this method is part of a drivetrain subsystem that provides the
    // necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        this.kinematics, // SwerveDriveKinematics
                        new PIDController(Constants.kPXController, 0, 0), // X controller. Tune these values for your
                                                                          // robot. Leaving them 0 will only use
                                                                          // feedforwards.
                        new PIDController(Constants.kPXYController, 0, 0), // Y controller (usually the same values as X
                                                                           // controller)
                        new PIDController(Constants.kPThetaController, 0, 0), // Rotation controller. Tune these values
                                                                              // for your robot. Leaving them 0 will
                                                                              // only use feedforwards.
                        this::drive, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }
}