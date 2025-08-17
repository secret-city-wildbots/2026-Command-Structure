package frc.robot.commands.pivot;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.actors.subsystems.Pivot;

public class PivotForwardCommand extends Command {
    // Real Variables
    private final Pivot pivot;

    /**
     * Creates and sets up the PivotForwardCommand
     * 
     * @param pivot The subsystem to be controlled by the command ({@link Pivot})
     * @param position_deg The position in degrees we want the pivot to move to
     */
    public PivotForwardCommand(Pivot pivot) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        // Call the pivot subsystem foward function
        pivot.forward();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the pivot subsystem
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when a beam break is broken?
        return false;
    }
}
