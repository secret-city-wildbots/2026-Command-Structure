package frc.robot.commands;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.actors.subsystems.Intake;

public class ScoreCoralCommand extends Command {
    // Real Variables
    private final Intake intake;

    /**
     * Creates and sets up the ScoreCoralCommand
     * 
     * @param intake The subsystem to be controlled by the command ({@link Intake})
     */
    public ScoreCoralCommand(Intake intake) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Call the Intake subsystem releaseGamePiece Function
        intake.releaseGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the Intake subsystem
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when a beam break is not broken?
        return false;
    }
}
