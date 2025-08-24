package frc.robot.commands.pivot;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.actors.subsystems.Pivot2;

public class Pivot2ToPositionCommand extends Command {
    // Real Variables
    private final Pivot2 pivot2;
    private final Double position_deg;

    /**
     * Creates and sets up the Pivot2ToPositionCommand
     * 
     * @param pivot2 The subsystem to be controlled by the command ({@link Pivot2})
     * @param position_deg The position we want the pivot to move to in degrees
     */
    public Pivot2ToPositionCommand(Pivot2 pivot2, Double position_deg) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.pivot2 = pivot2;
        this.position_deg = position_deg;
        addRequirements(pivot2);
    }

    @Override
    public void execute() {
        // Call the pivot subsystem set target angle function
        pivot2.setTargetAngle(this.position_deg);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the pivot subsystem
        pivot2.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command until the error is within an acceptable range
        double currentDegrees = this.pivot2.getAngleDegrees(); // In degrees
        double error = Math.abs(currentDegrees - this.position_deg);
        return error <= 0.01;
    }
}
