// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.drive.SwerveDriveSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ShooterSubsystem;

import javax.inject.Inject;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {

    XboxController xboxController;
    SwerveDriveSubsystem swerveDriveSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    @Inject
    public RobotContainer(SwerveDriveSubsystem swerveDriveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        // Uses a lambda expression to set the default command of the TemplateDrivetrainSubsystem to drive
        // based on the joystick values of the Xbox controller


        // While the A button on the Xbox controller is pressed, the arm will move up at 50% power
        JoystickButton shooterButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        JoystickButton intakeButton = new JoystickButton(xboxController, XboxController.Button.kB.value);

        shooterButton.whileTrue(new ToggleShooterCommand(0.5, shooterSubsystem));
        intakeButton.whileTrue(new RunCommand(() -> {
            intakeSubsystem.toggleMotor();
        }));
        swerveDriveSubsystem.drive(xboxController.getRightX() - xboxController.getLeftX(), xboxController.getRightY() - xboxController.getLeftY());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new RunCommand(() -> swerveDriveSubsystem.drive(10, 10));
    }
}
