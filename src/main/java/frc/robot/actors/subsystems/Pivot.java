package frc.robot.actors.subsystems;

import frc.robot.Robot;

// Import custom actors and utils
import frc.robot.actors.generic.Motor;
import frc.robot.utils.MotorType;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Phoenix 6 Libraries
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

// Import WPILib, Math, and Unit Libraries - For Simulation
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// Import Phoenix 6 Sim Libraries - For Simulation
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.CANcoderSimState;

public class Pivot extends SubsystemBase {
    // Real Motor Variables
    private Motor motor;
    private int motor_canID;
    private double kGearRatio;

    // Real Encoder Variables
    private CANcoder encoder;
    private int encoder_canID;

    // Motion Magic Voltage Controller
    private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    // Simulation Variables
    private TalonFXSimState motorSim;
    private DCMotorSim motorSimModel;
    private CANcoderSimState encoderSim;

    /**
     * Creates the CoralIntake Class
     * 
     * @param motor_canID The canID number of the intake, used to set the CAN ID of the motor
     * @param encoder_canID The canID number of the encoder, used to sets the CAN ID of the encoder
     */
    public Pivot(int motor_canID, int encoder_canID) {
        // initialize the motor number and motor
        this.motor_canID = motor_canID;
        this.motor = new Motor(this.motor_canID, MotorType.TFX);
        this.kGearRatio = 10.0;

        // initialize the encoder number and encoder
        this.encoder_canID = encoder_canID;
        this.encoder = new CANcoder(this.encoder_canID);

        this.setupConfig();

        // Initialize the simluation variables for motor
        motorSim = motor.getSimStateTalonFX();
        
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, this.kGearRatio),
            DCMotor.getKrakenX60Foc(1),
            0.001,
            0.05
        );

        // Initialize the simulation variables for encoder
        encoderSim = encoder.getSimState();

        this.motor.resetPos(1.0);
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot forwards
     */
    public void forward() {
        this.motor.dc(1.0);
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot backwards 
     */
    public void backward() {
        this.motor.dc(-1.0);
    }

    public void moveToPosition(double degrees) {
        // Convert degrees to motor rotations
        double targetRotations = (degrees / 360.0) * this.kGearRatio;

        // Send Motion Magic command
        this.motor.applyControl(motionMagic.withPosition(targetRotations));
    }

    /**
     * Sets the pivot motor to 0 speed 
     */
    public void stop() {
        this.motor.dc(0.0);
    }

    public double getPos() {
        return this.encoder.getAbsolutePosition().getValueAsDouble();
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
        encoderSim.setRawPosition(pivotRotations);  // Simulate the encoder reading this arm position - Wrap around to [0, 1)
    }

    private void setupConfig() {
        // Motor Configuration
        // Configure the TalonFX to use CANcoder as its remote sensor
        TalonFXConfiguration motorConfig = this.motor.getConfig();

        if (Robot.isSimulation()) {
            motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        }else
        {
            motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            motorConfig.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
        }
    

       
        motorConfig.Feedback.SensorToMechanismRatio = this.kGearRatio; // Adjust for gearing if needed

        // Set PID values (tune these!)
        motorConfig.Slot0.kP = 15.0; // Start with P only and tune
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;

        // Motion Magic configs (set your cruise velocity and acceleration)
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 9999.0; // rotations/sec
        motorConfig.MotionMagic.MotionMagicAcceleration = 9999.0;   // rotations/sec^2

        // Convert degrees to rotations
        double forwardLimit = (180.0 / 360.0) * this.kGearRatio;
        double reverseLimit = (2.0 / 360.0) * this.kGearRatio;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        this.motor.applyTalonFXConfig(motorConfig);

        // Encoder Configuration
        MagnetSensorConfigs magnetCfg = new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(1.0)) // 0–1 range
            .withMagnetOffset(0.0); // optional zero offset

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(magnetCfg);

        this.encoder.getConfigurator().apply(encoderConfig);

    }

}
