package frc.robot;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
/*
 * Publishes values for tuning PID via DogLog
 */
public class MotorTuner extends DogLog{
    /*
     * Use a boolean supplier to check if in test mode
     * otherwise don't allow tuning
     */
    //this returns true if in test mode
    private static BooleanSupplier isTestMode = () -> DriverStation.isTest();

    // class for TalonFX motor controllers
    public static class TalonMotor {
        public static void tunablePID(String motorName, TalonFX motor, Slot0Configs defaultConfigs){
            // This make sure we are in test mode before chaning values
            if(isTestMode.getAsBoolean()){
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
            
        }
        /*
         * Publishes values for tuning PID and Feedforward using DogLog
         */
        public static void tunablePIDFeedforward(String motorName, TalonFX motor, Slot0Configs defaultConfigs){
            if(isTestMode.getAsBoolean()){
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
    }
    public static class SparkMotor{
        /*
         * This is a PID with a feedforward value
         * RevLib only offers one feedforward value
         * (Use the feedforward for static friction or gravity)
         */
        public static void tunablePIDF(String motorName, SparkMax motor){
            if(isTestMode.getAsBoolean()){
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
    }
    public static class NativeController{
        /*
         * Note: WPIlib feedforward can't be adjusted so you need to run a sysID routine to tune those values
         */
        public static void tunablePID(String motorName, PIDController controller){
            if(isTestMode.getAsBoolean()){
                DogLog.tunable(motorName + " kP", controller.getP(), updatedValue -> controller.setP(updatedValue));
                DogLog.tunable(motorName + " kI", controller.getI(), updatedValue -> controller.setI(updatedValue));
                DogLog.tunable(motorName + " kD", controller.getD(), updatedValue -> controller.setD(updatedValue));
            }
        }
        /*
         * Overloaded method to accept ProfiledPIDController
         */
        public static void tunablePID(String motorName, ProfiledPIDController controller){
            if(isTestMode.getAsBoolean()){
                DogLog.tunable(motorName + " kP", controller.getP(), updatedValue -> controller.setP(updatedValue));
                DogLog.tunable(motorName + " kI", controller.getI(), updatedValue -> controller.setI(updatedValue));
                DogLog.tunable(motorName + " kD", controller.getD(), updatedValue -> controller.setD(updatedValue));
            }
        }
    }
}
