package frc.robot.intake;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.common.HardwareConfigConstants;

import javax.inject.Singleton;

@Singleton
public class IntakeSubsystem {
    private WPI_TalonFX motor = new WPI_TalonFX(HardwareConfigConstants.INTAKE_MOTOR);
    boolean on = true;
    public void toggleMotor() {
        on = !on;
        motor.set(on ? 1 : -1);
        try {
            Thread.sleep(3000);
        } catch (Exception e) {
            motor.set(0);
        } finally {
            motor.set(0);
        }
    }
}
