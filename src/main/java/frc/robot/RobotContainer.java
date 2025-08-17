// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Constants
import frc.robot.Constants.OperatorConstants;

// Import Subsystems
import frc.robot.actors.subsystems.Intake;
import frc.robot.actors.subsystems.Pivot;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.intake.*;
import frc.robot.commands.pivot.*;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake(1);
  private final Pivot pivot = new Pivot(2, 3);

  // Instantiate a Command Xbox Controller
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
  private void configureBindings() {
    // Schedule `IntakeCoralCommand` when the Xbox controller's rightTrigger button is pressed, cancel on release
    // Schedule `ScoreCoralCommand` when the Xbox controller's rightTrigger button is pressed, cancel on release
    driveController.rightTrigger().whileTrue(new IntakeCoralCommand(intake));
    driveController.leftTrigger().whileTrue(new ReleaseCoralCommand(intake));

    // Schedule `IntakeAlgaeCommand` when the Xbox controller's rightBumper button is pressed, cancel on release
    // Schedule `ScoreAlgaeCommand` when the Xbox controller's leftBumper button is pressed, cancel on release
    driveController.rightBumper().whileTrue(new IntakeAlgaeCommand(intake));
    driveController.leftBumper().whileTrue(new ReleaseAlgaeCommand(intake));

    // Schedule `PivotForwardCommand` when the Xbox controller's y button is pressed, cancel on release
    // Schedule `PivotBackwardCommand` when the Xbox controller's a button is pressed, cancel on release
    driveController.y().whileTrue(new PivotForwardCommand(pivot));
    driveController.a().whileTrue(new PivotBackwardCommand(pivot));

    // Schedule `PivotToPositionCommand` when the Xbox controller's b button is pressed. Will run until the isFinished condition is met.
    // Schedule `PivotToPositionCommand` when the Xbox controller's x button is pressed. Will run until the isFinished condition is met.
    driveController.b().onTrue(new PivotToPositionCommand(pivot, 50.0));
    driveController.x().onTrue(new PivotToPositionCommand(pivot, 90.0));

    // Schedule `PivotToPositionCommand` when the Xbox controller's D-Pad Up button is pressed. Will run until the isFinished condition is met.
    // Schedule `PivotToPositionCommand` when the Xbox controller's D-Pad Down button is pressed. Will run until the isFinished condition is met.
    driveController.povUp().onTrue(new PivotToPositionCommand(pivot, 175.0));
    driveController.povDown().onTrue(new PivotToPositionCommand(pivot, 5.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(this.pivot, this.intake);
  }
}
