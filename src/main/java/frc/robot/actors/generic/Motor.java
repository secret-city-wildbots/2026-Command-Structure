package frc.robot.actors.generic;

// Import Phoenix6 Hardware Libraries
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Import Phoenix6 Simluation Libraries
import com.ctre.phoenix6.sim.TalonFXSimState;

// Import Rev Robotics SparkMax Libraries
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Import Our Custom Utility Files
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Motor {
    // Generic Variables
    public int CanID;
    public MotorType type;
    public MotorConfig motorConfig;
    public String actuatorName = "not_set";

    // TalonFX Specific Variables
    public TalonFX motorTFX;
    public TalonFXConfiguration configTFX;
    public Slot0Configs slot0TFX;

    // SparkMax Specific Variables
    public SparkMax motorSPX;
    public SparkMaxConfig configSPX;
    

    /**
     * Basic contsructor for the motor.
     * 
     * @param CanID - ID of the motor on the CAN bus
     * @param type - The motor type. To see all the options look at {@link MotorType}
     */
    public Motor(int CanID, MotorType type) {
        // Initialize the generic variables
        this.CanID = CanID;
        this.type = type;

        // Depending on the type of the motor, we will configure the motor differently
        switch (type) {
            // SparkMax Motor
            case SPX:
                // Instantiate the motor and configuration for the motor
                this.motorSPX = new SparkMax(CanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
                this.configSPX = new SparkMaxConfig();
                break;
            // TalonFX Motor
            case TFX:
                // Instantiate the motor and configuration for the motor
                this.motorTFX = new TalonFX(CanID);
                this.configTFX = new TalonFXConfiguration();
                this.slot0TFX = new Slot0Configs();
                break;
            // None Type
            case None:
                System.err.println("Motor initialized with None type with CanID " + this.CanID);
        }

        // Get a new generic motor configuration and apply the configuration to the motor
        this.motorConfig = new MotorConfig();
        this.applyConfig();
    }

    /**
     * Advanced contsructor for the motor.
     * 
     * @param CanID - ID of the motor on the CAN bus
     * @param type - The motor type. To see all the options look at {@link MotorType}
     * @param actuatorName - The name for the motor we want to display on the dashboard
     */
    public Motor(int CanID, MotorType type, String actuatorName) {
        // Initialize the generic variables
        this.CanID = CanID;
        this.type = type;
        this.actuatorName = actuatorName;

        // Depending on the type of the motor, we will configure the motor differently
        switch (type) {
            // SparkMax Motor
            case SPX:
                this.motorSPX = new SparkMax(CanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
                this.configSPX = new SparkMaxConfig();
                break;
            // TalonFX Motor
            case TFX:
                this.motorTFX = new TalonFX(CanID);
                this.configTFX = new TalonFXConfiguration();
                this.slot0TFX = new Slot0Configs();
                break;
            // None
            case None:
                System.err.println("Motor initialized with None type with CanID " + this.CanID);
        }

        // Get a new generic motor configuration and apply the configuration to the motor
        this.motorConfig = new MotorConfig();
        this.applyConfig();
    }

    /**
     * Sets the duty cycle of the motor
     * 
     * @param dutyCycle from -1.0 to 1.0
     * <p>Below are the links to the code documentation for the motor types:
     * <ul>
     * <li><a href="https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double)">TalonFX</a></li>
     * <li><a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkbase#set(double)">REV Spark Max</a></li>
     * </ul>
     */
    public void dc(double dutyCycle) {
        // Depending on the type of the motor, we will call their respective functions
        switch (this.type) {
            // SparkMax
            case SPX:
                this.motorSPX.set(dutyCycle);
                break;
            // TalonFX 
            case TFX:
                this.motorTFX.set(dutyCycle);
                break;
            case None:
                System.err.println("tried to set dc on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the duty cycle of the motor
     * 
     * @return the duty cycle from -1.0 to 1.0
     * <p>Below are the links to the code documentation for the motor types:
     * <ul>
     * <li><a href="https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#get()">TalonFX</a></li>
     * <li><a href="https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkbase#get()">REV Spark Max</a></li>
     * </ul>
     */
    public double dc() {
        // Depending on the type of the motor, we will call their respective functions
        switch (this.type) {
            // SparkMax
            case SPX:
                return this.motorSPX.get();
            // TalonFX                
            case TFX:
                return this.motorTFX.get();
            default:
                return 0.0;
        }
    }

    /**
     * Sets the position of the motor without a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     */
    public void pos(double pos) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = 0;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Sets the position of the motor with a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     * @param ff  the feedforward
     */
    public void pos(double pos, double ff) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = ff;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the position of the motor
     * 
     * @return the pos
     */
    public double pos() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.getEncoder().getPosition();
            case TFX:
                return this.motorTFX.getPosition().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * tells the motor that wherever it is is the value passed to it
     * 
     * @param position the pos to set the encoder value to
     */
    public void resetPos(double position) {
        switch (this.type) {
            case SPX:
                this.motorSPX.getEncoder().setPosition(position);
            case TFX:
                this.motorTFX.setPosition(position);
            default:
                System.err.println("tried to reset pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * gives current rotor velocity in RpS
     * @return the motor velocity
     */
    public double vel() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.getEncoder().getVelocity();
            case TFX:
                return this.motorTFX.getVelocity().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * gets if there are currently any faults oon the motor
     * @return
     */
    public boolean isFault() {
        switch (this.type) {
            case SPX:
                return (short) 0 != motorSPX.getFaults().rawBits;
            case TFX:
                return 0 != motorTFX.getFaultField().getValueAsDouble();
            default:
                return false;
        }
    }

    /**
     * Sets the PID of the motor
     * 
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public void pid(double p, double i, double d) {
        switch (type) {
            case SPX:
                configSPX.closedLoop.pid(p, i, d);
                motorSPX.configure(configSPX, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters);
                break;
            case TFX:
                this.slot0TFX.kP = p;
                this.slot0TFX.kI = i;
                this.slot0TFX.kD = d;
                this.motorTFX.getConfigurator().apply(this.slot0TFX);
                break;
            case None:
                System.err.println("tried to set pid on None motor with CanID " + this.CanID);
        }
    }

    /**
     * set it to brake or coast, doesn't cause performance hit if the value isn't different
     * 
     * @param brake true for brake, false for coast
     */
    public void setBrake(boolean brake) {
        if (brake != this.motorConfig.brake) {
            this.motorConfig.brake = brake;
            this.applyConfig();
        }
    }

    public void applyConfig() {
        switch (this.type) {
            case SPX:
                this.configSPX.inverted(this.motorConfig.direction == RotationDir.Clockwise);
                this.configSPX.idleMode((this.motorConfig.brake) ? IdleMode.kBrake : IdleMode.kCoast);
                this.motorSPX.configure(configSPX, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                break;
            case TFX:
                this.configTFX.MotorOutput.PeakForwardDutyCycle = this.motorConfig.peakForwardDC;
                this.configTFX.MotorOutput.PeakReverseDutyCycle = this.motorConfig.peakReverseDC;
                this.configTFX.MotorOutput.Inverted = (this.motorConfig.direction == RotationDir.Clockwise)
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
                this.configTFX.MotorOutput.NeutralMode = (this.motorConfig.brake) ? NeutralModeValue.Brake
                        : NeutralModeValue.Coast;
                this.configTFX.HardwareLimitSwitch.ForwardLimitEnable = this.motorConfig.forwardLimitSwitchEnabled;
                this.configTFX.HardwareLimitSwitch.ReverseLimitEnable = this.motorConfig.reverseLimitSwitchEnabled;
                this.motorTFX.getConfigurator().apply(configTFX);
                break;
            case None:
                System.err.println("tried to apply motor config on None motor with CanID " + this.CanID);
        }
    }

    public TalonFXSimState getSimStateTalonFX() {
        return motorTFX.getSimState();
    }
}