package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.HardwareConfigConstants;
import frc.robot.vision.VisionSubsystem;

import javax.inject.Inject;
import javax.inject.Singleton;


@Singleton
public class ShooterSubsystem extends SubsystemBase {
    public static boolean atSetPoint = false;
    private static ShooterMode shooterMode;
    private final WPI_TalonFX shooterLeftKraken;
    private final WPI_TalonFX shooterRightKraken;
    private final MotorControllerGroup shooterKraken;
    private final PIDController shooterPID;
    private final SimpleMotorFeedforward feedForward;
    private final MoveAndShootController moveAndShootController;
    private final VisionSubsystem visionSubsystem;
    private double setPoint = 0.0;
    private double acceleration = 0.0;

    @Inject
    public ShooterSubsystem(MoveAndShootController moveAndShootController, VisionSubsystem visionSubsystem) {
        shooterLeftKraken = new WPI_TalonFX(HardwareConfigConstants.SHOOTER_LEFT_Kraken);
        shooterRightKraken = new WPI_TalonFX(HardwareConfigConstants.SHOOTER_RIGHT_Kraken);

        shooterKraken = new MotorControllerGroup(shooterLeftKraken, shooterRightKraken);

        shooterLeftKraken.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        shooterRightKraken.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        shooterLeftKraken.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        shooterLeftKraken.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        shooterRightKraken.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        shooterRightKraken.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

        shooterRightKraken.setInverted(true);
        shooterLeftKraken.follow(shooterRightKraken);
        shooterLeftKraken.setInverted(InvertType.OpposeMaster);

        shooterMode = ShooterMode.OFF;

        shooterLeftKraken.setNeutralMode(NeutralMode.Coast);
        shooterRightKraken.setNeutralMode(NeutralMode.Coast);

        shooterLeftKraken.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        shooterRightKraken.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

        shooterLeftKraken.enableVoltageCompensation(true);
        shooterRightKraken.enableVoltageCompensation(true);

        shooterPID = new PIDController(
                ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
        feedForward = new SimpleMotorFeedforward(
                ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA);
        this.moveAndShootController = moveAndShootController;
        this.visionSubsystem = visionSubsystem;
    }

    public static ShooterMode getShooterMode() {
        return shooterMode;
    }

    public void setShooterMode(ShooterMode shooterMode) {
        ShooterSubsystem.shooterMode = shooterMode;
    }

    @Override
    public void periodic() {
        atSetPoint = setPoint >= getMetersPerSecond() - ShooterConstants.VELOCITY_TOLERANCE
                && shooterMode != ShooterMode.OFF;

        switch (shooterMode) {
            case LIMELIGHT:
                if (visionSubsystem.getDistance() == 0.0) {
                    setPoint = 12;
                    shootFlywheel(setPoint);
                    break;
                }
                setSetPoint(2.564 * visionSubsystem.getDistance() + 12.787 + moveAndShootController.calculateShooterSpeed());
                setFlywheelVolts(
                        feedForward.calculate(setPoint, acceleration)
                                + shooterPID.calculate(getMetersPerSecond(), setPoint));
                break;
            case MANUAL:
                setFlywheelVolts(
                        feedForward.calculate(setPoint, acceleration)
                                + shooterPID.calculate(getMetersPerSecond(), setPoint));
                break;
            case LOW_GOAL:
                setSetPoint(10);
                setFlywheelVolts(feedForward.calculate(setPoint, acceleration)
                        + shooterPID.calculate(getMetersPerSecond(), setPoint));
                break;
            default:
                stopFlywheel();
                break;
        }
    }

    /**
     * @param setPoint in meters/second
     */
    public void setSetPoint(double setPoint) {
        this.acceleration = setPoint > 21.0 ? 17.0 : 0.8 * setPoint;
        this.setPoint = setPoint > ShooterConstants.MAX_VELOCITY ? 32 : setPoint;
    }

    private void shootFlywheel(double speed) {
        shooterKraken.set(speed);
    }

    private void setFlywheelVolts(double volts) {
        shooterKraken.setVoltage(volts);
    }

    public void stopFlywheel() {
        shooterKraken.stopMotor();
        setPoint = 0.0;
        shooterMode = ShooterMode.OFF;
    }

    public double getLeftEncoder() {
        return shooterLeftKraken.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return shooterRightKraken.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public double getFlywheelRPM() {
        return getLeftEncoder() * ShooterConstants.ENCODER_TIME_CONVERSION
                / ShooterConstants.ENCODER_RESOLUTION
                * ShooterConstants.PULLEY_RATIO;
    }

    public double getMetersPerSecond() {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (getFlywheelRPM() / 60.0);
    }

    public double getVelocityUnitsFromRPM() {
        return getFlywheelRPM() / ShooterConstants.PULLEY_RATIO
                * ShooterConstants.ENCODER_TIME_CONVERSION
                / ShooterConstants.ENCODER_RESOLUTION;
    }

    // returns in volts
    public double getFeedForward() {
        return (Constants.MOTOR_VOLTAGE_COMP / 8750.0) * setPoint;
    }

    public enum ShooterMode {
        LIMELIGHT,
        MANUAL,
        LOW_GOAL,
        OFF
    }
}