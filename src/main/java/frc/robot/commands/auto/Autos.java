// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Import Subsystems
import frc.robot.actors.subsystems.Intake;
import frc.robot.actors.subsystems.Pivot;

// Import Custom Commands
import frc.robot.commands.intake.*;
import frc.robot.commands.pivot.*;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Pivot pivot, Intake intake) {
    return Commands.sequence(
      new PivotToPositionCommand(pivot, 175.0),
      new IntakeCoralCommand(intake)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
