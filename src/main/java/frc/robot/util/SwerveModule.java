package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    
    //All the things
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final double absoluteEncoderOffsetRot;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        
        //Absolute encoder setup
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderOffsetRot = this.absoluteEncoderOffsetRad / (2 * Math.PI);
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        CANcoderConfigurator configer = absoluteEncoder.getConfigurator();
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetOffset = absoluteEncoderOffsetRot;
        config.SensorDirection = absoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configer.apply(config);

        //Motor setup
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //Relative encoder setup
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //Setup conversion factors
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //Setup PID controllers
        turningPidController = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        //Reset the encoders on start
        resetEncoders();
        
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad()); //This one gets reset to the actual position of the module
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < ModuleConstants.kModuleDeadband) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public double getDrivePosition() {return driveEncoder.getPosition();} //Returns position of drive encoder in meters traveled
    public double getTurningPosition() {return turningEncoder.getPosition();} //Returns position of turning encoder in radians
    public double getDriveVelocity() {return driveEncoder.getVelocity();} //Returns velocity of drive encoder in meters per second
    public double getTurningVelocity() {return turningEncoder.getVelocity();} //Returns velocity of drive encoder in radians per second
    public double getAbsoluteEncoderRot() {return absoluteEncoder.getAbsolutePosition().getValue();} //Returns position of absolute encoder in degrees
    public double getAbsoluteEncoderRad() {return getAbsoluteEncoderRot() * 2*Math.PI;} //Returns position of absolute encoder in radians
    public SwerveModuleState getState() {return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));} //Returns the above info in the form of a SwerveModuleState

}
