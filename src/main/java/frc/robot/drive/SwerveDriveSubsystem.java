package frc.robot.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.HardwareConfigConstants;

import javax.inject.Singleton;

@Singleton
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveWheel frontLeft = SwerveWheel.from(HardwareConfigConstants.FRONT_LEFT_DRIVE, HardwareConfigConstants.FRONT_LEFT_AZIMUTH, HardwareConfigConstants.FRONT_LEFT_ENCODER);
    private final SwerveWheel frontRight = SwerveWheel.from(HardwareConfigConstants.FRONT_RIGHT_DRIVE, HardwareConfigConstants.FRONT_RIGHT_AZIMUTH, HardwareConfigConstants.FRONT_RIGHT_ENCODER);
    private final SwerveWheel backLeft = SwerveWheel.from(HardwareConfigConstants.BACK_LEFT_DRIVE, HardwareConfigConstants.BACK_LEFT_AZIMUTH, HardwareConfigConstants.BACK_LEFT_ENCODER);
    private final SwerveWheel backRight = SwerveWheel.from(HardwareConfigConstants.BACK_RIGHT_DRIVE, HardwareConfigConstants.BACK_RIGHT_AZIMUTH, HardwareConfigConstants.BACK_RIGHT_ENCODER);

    private final WPI_Pigeon2 gyro = new WPI_Pigeon2(HardwareConfigConstants.GYRO);


    public void drive(double getLeft, double getRight) {
        SwerveModuleState swerveModuleState = new SwerveModuleState();
        swerveModuleState.speedMetersPerSecond = Math.sqrt(getLeft * getLeft + getRight * getRight);
        swerveModuleState = SwerveModuleState.optimize(swerveModuleState, new Rotation2d(getLeft, getRight));

        frontLeft.setDesiredState(swerveModuleState);
        frontRight.setDesiredState(swerveModuleState);
        backLeft.setDesiredState(swerveModuleState);
        backRight.setDesiredState(swerveModuleState);
    }
}
