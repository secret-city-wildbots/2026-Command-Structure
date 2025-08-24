package frc.robot.commands.pivot;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.actors.subsystems.Pivot2;

public class Pivot2ForwardCommand extends Command {
    // Real Variables
    private final Pivot2 pivot2;

    /**
     * Creates and sets up the PivotForwardCommand
     * 
     * @param pivot2 The subsystem to be controlled by the command ({@link Pivot2})
     */
    public Pivot2ForwardCommand(Pivot2 pivot2) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.pivot2 = pivot2;
        addRequirements(pivot2);
    }

    @Override
    public void execute() {
        // Call the pivot subsystem foward function
        pivot2.forward();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the pivot subsystem
        pivot2.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when a beam break is broken?
        return false;
    }
}
