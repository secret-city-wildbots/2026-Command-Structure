package frc.robot.actors.subsystems;

// Import custom actors and utils
import frc.robot.actors.generic.Motor;
import frc.robot.utils.MotorType;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import WPILib, Math, and Unit Libraries - For Simulation
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

// Import Phoenix 6 Sim Libraries - For Simulation
import com.ctre.phoenix6.sim.TalonFXSimState;

public class Pivot2 extends SubsystemBase {
    // Real Motor Variables
    private Motor motor;
    private int motor_canID;
    private double kGearRatio;

    // Real Encoder Variables
    private DutyCycleEncoder encoder;
    private int encoder_channelID;

    // Soft Limits
    private double minAngle_deg = 2.0;   // max front position
    private double maxAngle_deg = 180.0; // max back position

    // PID Controller
    private PIDController pid;

    // Simulation Variables
    private TalonFXSimState motorSim;
    private DCMotorSim motorSimModel;
    private DutyCycleEncoderSim encoderSim;

    // State
    private double targetAngle_deg = 2.0; // Degrees

    /**
     * Creates the CoralIntake Class
     * 
     * @param motor_canID The canID number of the intake, used to set the CAN ID of the motor
     * @param encoder_channelID The Channel ID of the encoder plugged into the RIO
     */
    public Pivot2(int motor_canID, int encoder_channelID) {
        // initialize the motor number and motor
        this.motor_canID = motor_canID;
        this.motor = new Motor(this.motor_canID, MotorType.TFX);
        this.kGearRatio = 10.0;

        // initialize the encoder channel and encoder
        this.encoder_channelID = encoder_channelID;
        this.encoder = new DutyCycleEncoder(this.encoder_channelID); // Absolute encoder

        // initialize the PID
        // TODO: This needs to be tuned for the real system
        this.pid = new PIDController(0.001, 0.001, 0.0);

        // Initialize the simluation variables for motor
        motorSim = motor.getSimStateTalonFX();
        
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, this.kGearRatio),
            DCMotor.getKrakenX60Foc(1),
            0.001,
            0.05
        );

        // Initialize the simulation variables for encoder
        encoderSim = new DutyCycleEncoderSim(this.encoder);
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot forwards
     */
    public void forward() {
        // Enforce soft limits: prevent movement beyond range
        if (isAtUpperLimit()) {
            this.motor.dc(0.0);
        }
        else {
            this.motor.dc(1.0);
        }
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot backwards 
     */
    public void backward() {
        // Enforce soft limits: prevent movement beyond range
        if (isAtLowerLimit()) {
            this.motor.dc(0.0);
        }
        else {
            this.motor.dc(-1.0);
        }
    }

    /**
     * Sets the target angle for the Pivot system.
     * This function will ensure it is clamped between the min and max angles.
     * 
     * @param angle_deg The angle that the pivot is requested to go to
     */
    public void setTargetAngle(double angle_deg) {
        // Clamp target angle within soft limits
        this.targetAngle_deg = Math.max(this.minAngle_deg, Math.min(this.maxAngle_deg, angle_deg));
    }

    /**
     * Returns the current pivot angle
     * @return the pivot angle in degrees
     */
    public double getAngleDegrees() {
        return this.encoder.get() * 360.0;
    }

    /**
     * Returns the current pivot angle
     * @return the pivot angle in radians
     */
    public double getAngleRadians() {
        return this.encoder.get() * 2.0 * Math.PI;
    }

    /**
     * Returns if the pivot is at the lower limit
     * @return true if at the lower limit, else false
     */
    public boolean isAtLowerLimit() {
        return getAngleDegrees() <= this.minAngle_deg;
    }

    /**
     * Returns if the pivot is at the upper limit
     * @return true if at the upper limit, else false
     */
    public boolean isAtUpperLimit() {
        return getAngleDegrees() >= this.maxAngle_deg;
    }

    /**
     * Sets the pivot motor to 0 speed 
     */
    public void stop() {
        this.motor.dc(0.0);
    }

    @Override
    public void periodic() {
        // Calculate motor output from PID controller
        double motorOutput = pid.calculate(getAngleDegrees(), this.targetAngle_deg);

        // Clamp output to safe range [-1.0, 1.0]
        motorOutput = Math.max(-1.0, Math.min(1.0, motorOutput));

        // Enforce soft limits: prevent movement beyond range
        if (isAtUpperLimit() && motorOutput > 0) {
            motorOutput = 0.0;
        }
        if (isAtLowerLimit() && motorOutput < 0) {
            motorOutput = 0.0;
        }

        // Send the output to the motor
        this.motor.dc(motorOutput);
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

        // Update simulated encoder value
        double motorRotations = motor.pos(); // Get the simulated motor shaft position (in rotations)
        double pivotRotations = motorRotations / this.kGearRatio; // Convert to mechanism rotations (e.g., arm rotation)
        encoderSim.set(pivotRotations);  // Simulate the encoder reading this arm position - Wrap around to [0, 1)
    }
}
