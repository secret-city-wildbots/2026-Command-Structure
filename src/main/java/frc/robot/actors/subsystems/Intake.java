package frc.robot.actors.subsystems;

import frc.robot.actors.generic.Motor;
import frc.robot.utils.MotorType;
// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import WPILib, Math, and Unit Libraries - For Simulation
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// Phoenix 6 Library Imports
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Intake extends SubsystemBase {
    // Real Variables
    private Motor motor;
    private int canID;
    private double kGearRatio;

    // Simulation Variables
    private TalonFXSimState motorSim;
    private DCMotorSim motorSimModel;

    /**
     * Creates the CoralIntake Class
     * 
     * @param canID The canID number of the intake, used to set the CAN ID of the motor
     */
    public Intake(int canID) {
        // initialize the module number and motor
        this.canID = canID;
        this.motor = new Motor(this.canID, MotorType.TFX);
        this.kGearRatio = 10.0;

        // Initialize the simluation variables
        motorSim = motor.getSimStateTalonFX();
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, this.kGearRatio),
            DCMotor.getKrakenX60Foc(1),
            0.001,
            0.05
        );
    }

    /**
     * Sets the intake motor to spin spin in the direction to intake a coral piece
     */
    public void acquireCoral() {
        this.motor.dc(1.0);
    }

    /**
     * Sets the intake motor to spin spin in the direction to outtake a coral piece 
     */
    public void releaseCoral() {
        this.motor.dc(-1.0);
    }

    /**
     * Sets the intake motor to spin spin in the direction to intake an algae piece
     */
    public void acquireAlgae() {
        this.motor.dc(-1.0);
    }

    /**
     * Sets the intake motor to spin spin in the direction to outtake an algae piece 
     */
    public void releaseAlgae() {
        this.motor.dc(1.0);
    }

    /**
     * Sets the intake motor to 0 speed 
     */
    public void stop() {
        this.motor.dc(0.0);
    }

    /**
     * This function looks at the intake motor spin direction to determine if we are acquiring a gamepiece.
     * 
     * @return true if intaking, false otherwise
     */
    public boolean acquiringGamePiece() {
        // If the motor velocity is > 0 (this means we are intaking)
        if (this.motor.vel() > 0.0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * This function looks at the intake motor spin direction to determine if we are releasing a gamepiece.
     * 
     * @return true if intaking, false otherwise
     */
    public boolean releasingGamePiece() {
        // If the motor velocity is > 0 (this means we are intaking)
        if (this.motor.vel() < 0.0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * tells us whether the intake has a coral or not
     * 
     * @return true if the intake has a coral, false otherwise
     */
    public boolean get_hasPiece() {
        // TODO: create logic to use a sensor to detect if we have a gamepiece or not. Beam break?
        return false;
    }

    @Override
    public void simulationPeriodic() {
        // Supply voltage from battery
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Get actual voltage applied to motor
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();

        // Feed voltage into simulation model
        motorSimModel.setInputVoltage(motorVoltage.in(edu.wpi.first.units.Units.Volts));
        motorSimModel.update(0.020); // 20ms update

        // Update the simulated TalonFX state with output from DCMotorSim
        motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(this.kGearRatio));
        motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(this.kGearRatio));
    }

}
