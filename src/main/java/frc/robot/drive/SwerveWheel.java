package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveWheel {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX azimuthMotor;

    private final WPI_CANCoder absoluteEncoder;
    private final PIDController drivePIDController = new PIDController(
            SwerveDriveConstants.DRIVE_MOTOR_P,
            SwerveDriveConstants.DRIVE_MOTOR_I,
            SwerveDriveConstants.DRIVE_MOTOR_D);
    private final ProfiledPIDController azimuthPIDController = new ProfiledPIDController(
            SwerveDriveConstants.AZIMUTH_MOTOR_P,
            SwerveDriveConstants.AZIMUTH_MOTOR_I,
            SwerveDriveConstants.AZIMUTH_MOTOR_D,
            new TrapezoidProfile.Constraints(3 * Math.PI, 6 * Math.PI));
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            SwerveDriveConstants.DRIVE_MOTOR_KS, SwerveDriveConstants.DRIVE_MOTOR_KV, SwerveDriveConstants.DRIVE_MOTOR_KA
    );

    private final SimpleMotorFeedforward azimuthFeedForward = new SimpleMotorFeedforward(
            SwerveDriveConstants.AZIMUTH_MOTOR_KS, SwerveDriveConstants.AZIMUTH_MOTOR_KV, SwerveDriveConstants.AZIMUTH_MOTOR_KA
    );

    public SwerveWheel(WPI_TalonFX driveMotor, WPI_TalonFX azimuthMotor, WPI_CANCoder wpiCanCoder) {
        this.driveMotor = driveMotor;
        this.azimuthMotor = azimuthMotor;
        this.absoluteEncoder = wpiCanCoder;
    }

    public static SwerveWheel from(int driveMotorPort, int azimuthMotorPort, int canCoderPort) {
        WPI_TalonFX dMotor = new WPI_TalonFX(driveMotorPort);
        WPI_TalonFX aMotor = new WPI_TalonFX(azimuthMotorPort, "canivoreBus");
        WPI_CANCoder coder = new WPI_CANCoder(canCoderPort, "canivoreBus");

        return new SwerveWheel(dMotor, aMotor, coder);
    }
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * 10 / 2048 *
                (SwerveDriveConstants.WHEEL_DIAMETER * Math.PI) * SwerveDriveConstants.SWERVE_X_REDUCTION;
    }
    public double getAzimuthPosition() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition());
    }
    public void setDesiredState(SwerveModuleState desiredState) {
        double driveVelocity = getDriveVelocity();
        double azimuthPosition = getAzimuthPosition();

        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(azimuthPosition));
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
                && Math.abs(desiredState.angle.getRadians() - azimuthPosition) < 0.05) {
            stop();
            return;
        }

        final double driveOutput =
                drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
                        + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        final double azimuthOutput =
                azimuthPIDController.calculate(azimuthPosition, desiredState.angle.getRadians())
                        + azimuthFeedForward.calculate(azimuthPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput);
        azimuthMotor.setVoltage(azimuthOutput);
    }

    public void stop() {
        driveMotor.setVoltage(0.0);
        azimuthMotor.set(0.0);
    }
}
