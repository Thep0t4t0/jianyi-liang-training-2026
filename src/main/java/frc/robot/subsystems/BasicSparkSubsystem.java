package frc.robot.subsystems;

import java.lang.module.Configuration;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BasicSparkSubsystem {
    private final int sparkID = 0;
    private final boolean inverted = true;
    private final int currentList = 60;

    private final double slot0Kp = 0.1;
    private final double slot0Ki = 0;
    private final double slot0Kd = 0;

    private final double slot0Ks = 0.1;

    private final double slot1Kp = 0.1;
    private final double slot1Ki = 0;
    private final double slot1Kd = 0;

    private final double slot1Ks = 0.1;
    private final double slot1Kv = 0.1;


    private SparkMax motor;
    private SparkMaxConfig config;
    private SparkClosedLoopController controller;
    private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

    private boolean isBrakeMode = true;

    public BasicSparkSubsystem() {
        motor = new SparkMax(sparkID, MotorType.kBrushless);

        controller = motor.getClosedLoopController();

        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(currentList)
            .voltageCompensation(12);

        config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360/60);
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                slot0Kp,
                slot0Ki,
                slot0Kd,
                ClosedLoopSlot.kSlot0
            )
            .pid(
                slot1Kp,
                slot1Ki,
                slot1Kd,
                ClosedLoopSlot.kSlot1
            );
        
        config.closedLoop.feedForward
            .kS(slot0Ks, ClosedLoopSlot.kSlot0)
            .kS(slot1Ks, ClosedLoopSlot.kSlot1)
            .kS(slot1Kv, ClosedLoopSlot.kSlot1);
        
        config.signals
            .appliedOutputPeriodMs(20)
            .outputCurrentPeriodMs(100)
            .busVoltagePeriodMs(100)
            .faultsAlwaysOn(true)
            .iAccumulationAlwaysOn(true)
            .motorTemperaturePeriodMs(1000)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .warningsAlwaysOn(true)
            .warningsPeriodMs(1000);
        
        motor.clearFaults();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        public void runPercentOutput(double percentOutput) {
            motor.set(percentOutput);
        }

        public void runVoltage(double voltageVolts) {
            motor.setVoltage(voltageVolts);
        }

        public void runPosition(double position) {
            setPIDSlot(ClosedLoopSlot.kSlot0);
            controller.setSetpoint(position, ControlType.kPosition, closedLoopSlot);
        }
            
        public void runVelocity(double velocity) {
            setPIDSlot(ClosedLoopSlot.kSlot1);
            controller.setSetpoint(velocity, ControlType.kVelocity, closedLoopSlot);
        }

        public void stop() {
            motor.stopMotor();
        }

        public void setPIDSlot(ClosedLoopSlot slot) {
            closedLoopSlot = slot;
        }

        public void setBrakeMode(boolean brake) {
            if (isBrakeMode != brake) {
                SparkMaxConfig configuration = new SparkMaxConfig();
                configuration.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

                motor.configure(configuration, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

                isBrakeMode = brake;
            }
        }

        public void reset() {
            motor.clearFaults();
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

