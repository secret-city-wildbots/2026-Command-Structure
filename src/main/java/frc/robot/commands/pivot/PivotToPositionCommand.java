package frc.robot.commands.pivot;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.actors.subsystems.Pivot;

public class PivotToPositionCommand extends Command {
    // Real Variables
    private final Pivot pivot;
    private final Double position_deg;

    /**
     * Creates and sets up the PivotToPositionCommand
     * 
     * @param pivot The subsystem to be controlled by the command ({@link Pivot})
     * @param position_deg The position we want the pivot to move to in degrees
     */
    public PivotToPositionCommand(Pivot pivot, Double position_deg) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.pivot = pivot;
        this.position_deg = position_deg;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        // Call the pivot subsystem move to position function
        pivot.moveToPosition(this.position_deg);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the pivot subsystem
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command until the error is within an acceptable range
        double currentRotations = this.pivot.getPos(); // In rotations
        double currentDegrees = currentRotations * 360.0;
        double error = Math.abs(currentDegrees - this.position_deg);
        return error <= 0.01;
    }
}
