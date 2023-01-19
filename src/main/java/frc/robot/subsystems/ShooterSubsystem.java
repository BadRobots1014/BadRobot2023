package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*
     * Private constants ------------------------------------------------------
     */

    /**
     * Gravitational acceleration (little g) in m/s^2.
     */
    private static final double GRAVITATIONAL_ACCELERATION = 9.8;

    /**
     * The maximum tangential velocity of the shooter flywheel, in m/s.
     */
    private static final double MAX_FLYWHEEL_VELOCITY = 32.8;

    /*
     * Public constants -------------------------------------------------------
     */

    /**
     * The range of the ball from the target that determines whether to shoot from a
     * smaller or larger shooting angle.
     * 
     * If the range is lower than this value, use the smaller shooting angle,
     * otherwise, use the larger shooting angle.
     */
    public static final double RANGE_CUTOFF = 2.0;

    /**
     * The lower angle of the shooter, in degrees.
     */
    public static final double LOW_ANGLE = 65.0;

    /**
     * The higher angle of the shooter, in degrees.
     */
    public static final double HIGH_ANGLE = 75.0;

    /*
     * Private instance members -----------------------------------------------
     */

    /**
     * The speed controller for the shooter motor.
     */
    private final WPI_TalonFX motor;

    /*
     * Public utility methods -------------------------------------------------
     */

    /**
     * Calculates the initial velocity required for a ball launched
     * {@code initialAngle} degrees from the horizontal to hit a target
     * {@code range} meters away horizontally, and {@height} meters away vertically.
     * 
     * This method returns the value of v (velocity) that satisfies the following
     * system of physical equations:
     * range = cos(initialAngle) * v * t
     * height = sin(initialAngle) * v * t - (1/2) * GRAVITATIONAL_ACCELERATION * t^2
     * 
     * @param initialAngle the angle the ball is launched at, in degrees
     * @param range        the horizontal distance the ball must cover to hit the
     *                     target
     * @param height       the vertical distance the ball must cover to hit the
     *                     target
     * @return the initial velocity required for the ball to hit the target.
     */
    public double calculateInitialVelocity(double initialAngle, double range, double height) {
        initialAngle = Math.toRadians(initialAngle);
        double numerator = GRAVITATIONAL_ACCELERATION * Math.pow(range, 2.0);
        double denominator = (2 * Math.pow(Math.cos(initialAngle), 2)) *
                ((-1.0 * height) + (range * Math.tan(initialAngle)));

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Calculates the motor power output (as a fraction) required for a ball
     * launched at {@code initialAngle} degrees from the horizontal to hit a target
     * {@code range} meters away horizontally, and {@height} meters away vertically.
     * 
     * @param initialAngle the angle the ball is launched at, in degrees
     * @param range        the horizontal distance the ball must cover to hit the
     *                     target
     * @param height       the vertical distance the ball must cover to hit the
     *                     target
     * @return the motor power output required to launch a ball to hit the target.
     */
    public double calculatePower(double initialAngle, double range, double height) {
        return calculateInitialVelocity(initialAngle, range, height) / MAX_FLYWHEEL_VELOCITY;
    }

    /*
     * Constructor ------------------------------------------------------------
     */

    /**
     * No-argument constructor.
     */
    public ShooterSubsystem() {
        this.motor = new WPI_TalonFX(ShooterConstants.kShooterPort);
    }

    /*
     * Control methods --------------------------------------------------------
     */

    /**
     * Sets the flywheel motor to the given power.
     * 
     * @param power The motor power, as a percentage. This is positive for the
     *              forward direction and negative otherwise.
     */
    public void run(double power) {
        this.motor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Stops the flywheel motor.
     */
    public void stop() {
        this.motor.stopMotor();
    }
}
