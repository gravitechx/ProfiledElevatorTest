package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotState;
/*
 * COPY THIS FILE INTO PROJECT
 * Publishes values for tuning PID via DogLog
 */
public class MotorTuner extends DogLog{
    /**
     * Allows tuning only when robot is in test mode
     */
    public static void requireTestMode(){
        DogLog.setOptions(DogLog.getOptions().withNtTunables(() -> RobotState.isTest()));
    }

    // class for TalonFX motor controllers
    public static class TalonMotor {
        public static void tunablePID(String motorName, TalonFX motor, Slot0Configs defaultConfigs){
            // This make sure we are in test mode before chaning values
            TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            DogLog.tunable(motorName + " kP", defaultConfigs.kP, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKP(updatedValue));
            });
            DogLog.tunable(motorName + " kI", defaultConfigs.kI, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKI(updatedValue));
            });
            DogLog.tunable(motorName + " kD", defaultConfigs.kD, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKD(updatedValue));
            });
            
        }
        /*
         * Publishes values for tuning PID and Feedforward using DogLog
         */
        public static void tunablePIDFeedforward(String motorName, TalonFX motor, Slot0Configs defaultConfigs){
            TalonMotor.tunablePID(motorName, motor, defaultConfigs);

            TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            DogLog.tunable(motorName + " kG", defaultConfigs.kP, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKG(updatedValue));
            });
            DogLog.tunable(motorName + " kS", defaultConfigs.kS, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKS(updatedValue));
            });
            DogLog.tunable(motorName + " kV", defaultConfigs.kV, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKV(updatedValue));
            });
            DogLog.tunable(motorName + " kA", defaultConfigs.kA, updatedValue -> {
                motorConfigurator.apply(defaultConfigs.withKA(updatedValue));
            });
        }
    }
    public static class SparkMaxMotor{
        /*
         * This is a PID with a feedforward value
         * RevLib only offers one feedforward value
         * (Use the feedforward for static friction or gravity)
         */
        public static void tunablePIDF(String motorName, SparkMax motor){
            ClosedLoopConfigAccessor closedLoopAccessor = motor.configAccessor.closedLoop;
            ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
            SparkMaxConfig baseConfig = new SparkMaxConfig();
            
            DogLog.tunable(motorName + " kP", closedLoopAccessor.getP(), updatedValue -> {
                baseConfig.apply(
                    closedLoopConfig.p(updatedValue)
                );
                motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            });
            DogLog.tunable(motorName + " kI", closedLoopAccessor.getI(), updatedValue -> {
                baseConfig.apply(
                    closedLoopConfig.i(updatedValue)
                );
                motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            });
            DogLog.tunable(motorName + " kD", closedLoopAccessor.getD(), updatedValue -> {
                baseConfig.apply(
                    closedLoopConfig.d(updatedValue)
                );
                motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            });
            DogLog.tunable(motorName + " kV", closedLoopAccessor.getFF(), updatedValue -> {
                baseConfig.apply(
                    closedLoopConfig.velocityFF(updatedValue)
                );
                motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            });
        }
    }
    /*
     * "Native" is referring to WPILib controllers
     */
    public static class NativeController{
        /*
         * Note: WPILib feedforward can't be adjusted so you need to run a sysID routine to tune those values
         */
        public static void tunablePID(String motorName, PIDController controller){
            DogLog.tunable(motorName + " kP", controller.getP(), updatedValue -> controller.setP(updatedValue));
            DogLog.tunable(motorName + " kI", controller.getI(), updatedValue -> controller.setI(updatedValue));
            DogLog.tunable(motorName + " kD", controller.getD(), updatedValue -> controller.setD(updatedValue));
            DogLog.tunable(motorName + " tolerance", controller.getErrorTolerance(), updatedValue -> controller.setTolerance(updatedValue));
        }
        /*
         * Overloaded method to accept ProfiledPIDController
         */
        public static void tunablePID(String motorName, ProfiledPIDController controller){
            DogLog.tunable(motorName + " kP", controller.getP(), updatedValue -> controller.setP(updatedValue));
            DogLog.tunable(motorName + " kI", controller.getI(), updatedValue -> controller.setI(updatedValue));
            DogLog.tunable(motorName + " kD", controller.getD(), updatedValue -> controller.setD(updatedValue));
            DogLog.tunable(motorName + " position tolerance", controller.getPositionTolerance(), updatedValue -> controller.setTolerance(updatedValue));
            DogLog.tunable(motorName + " MaxVelocity", controller.getConstraints().maxVelocity, 
                updatedValue -> controller.setConstraints(
                    new Constraints(updatedValue, controller.getConstraints().maxAcceleration)
                )
            );
            DogLog.tunable(motorName + " MaxAcceleration", controller.getConstraints().maxAcceleration, 
                updatedValue -> controller.setConstraints(
                    new Constraints(controller.getConstraints().maxVelocity, updatedValue)
                )
            );
        }
        /*
         * Take in suppliers because WPILib feedforward objects can't be changed after initialization
         * We get around this by reassigning the variable storing the feedforward with a new object with updated values
         * This is specifically accomplished via suppliers to supply data about the current values of a feedforward
         * and consumers to modify the variable storing the feedforwards with updated values
         * NOTE: Feedforward variables cannot be final otherwise we cannot reassign them. 
         * ALSO: It is highly recommended to only use this initially for finding feedforwards manually. Then, set your variables to final to prevent them from being being reassigned.
         */
        public static void tunableMotorFeedforward(String motorName, Supplier<SimpleMotorFeedforward> feedforwardSupplier, Consumer<SimpleMotorFeedforward> feedforwardModifier){
            DogLog.tunable(motorName + " kS", feedforwardSupplier.get().getKs(), updatedValue -> feedforwardModifier.accept(new SimpleMotorFeedforward(
                updatedValue, 
                feedforwardSupplier.get().getKv(),
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kV", feedforwardSupplier.get().getKv(), updatedValue -> feedforwardModifier.accept(new SimpleMotorFeedforward(
                feedforwardSupplier.get().getKs(),
                updatedValue,
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kA", feedforwardSupplier.get().getKa(), updatedValue -> feedforwardModifier.accept(new SimpleMotorFeedforward(
                feedforwardSupplier.get().getKs(),
                feedforwardSupplier.get().getKv(),
                updatedValue
            )));
        }
        /*
         * Tunes an elevator feedforward with kS, kG, kV, kA
         */
        public static void tunableElevatorFeedforward(String motorName, Supplier<ElevatorFeedforward> feedforwardSupplier, Consumer<ElevatorFeedforward> feedforwardModifier){
            DogLog.tunable(motorName + " kS", feedforwardSupplier.get().getKs(), updatedValue -> feedforwardModifier.accept(new ElevatorFeedforward(
                updatedValue, 
                feedforwardSupplier.get().getKg(),
                feedforwardSupplier.get().getKv(),
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kG", feedforwardSupplier.get().getKg(), updatedValue -> feedforwardModifier.accept(new ElevatorFeedforward(
                feedforwardSupplier.get().getKs(),
                updatedValue, 
                feedforwardSupplier.get().getKv(),
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kV", feedforwardSupplier.get().getKv(), updatedValue -> feedforwardModifier.accept(new ElevatorFeedforward(
                feedforwardSupplier.get().getKs(),
                feedforwardSupplier.get().getKg(),
                updatedValue, 
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kA", feedforwardSupplier.get().getKa(), updatedValue -> feedforwardModifier.accept(new ElevatorFeedforward(
                feedforwardSupplier.get().getKs(),
                feedforwardSupplier.get().getKg(),
                feedforwardSupplier.get().getKv(),
                updatedValue
            )));
        }
        /*
         * Tunes an arm feedforward with kS, kG, kV, kA
         */
        public static void tunableArmFeedforward(String motorName, Supplier<ArmFeedforward> feedforwardSupplier, Consumer<ArmFeedforward> feedforwardModifier){
            DogLog.tunable(motorName + " kS", feedforwardSupplier.get().getKs(), updatedValue -> feedforwardModifier.accept(new ArmFeedforward(
                updatedValue, 
                feedforwardSupplier.get().getKg(),
                feedforwardSupplier.get().getKv(),
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kG", feedforwardSupplier.get().getKg(), updatedValue -> feedforwardModifier.accept(new ArmFeedforward(
                feedforwardSupplier.get().getKs(),
                updatedValue, 
                feedforwardSupplier.get().getKv(),
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kV", feedforwardSupplier.get().getKv(), updatedValue -> feedforwardModifier.accept(new ArmFeedforward(
                feedforwardSupplier.get().getKs(),
                feedforwardSupplier.get().getKg(),
                updatedValue, 
                feedforwardSupplier.get().getKa()
            )));
            DogLog.tunable(motorName + " kA", feedforwardSupplier.get().getKa(), updatedValue -> feedforwardModifier.accept(new ArmFeedforward(
                feedforwardSupplier.get().getKs(),
                feedforwardSupplier.get().getKg(),
                feedforwardSupplier.get().getKv(),
                updatedValue
            )));
        }
    }
}
