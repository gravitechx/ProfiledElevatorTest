package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorTuner;

import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase{
    private SparkMax m_elevatorLeaderMotor = new SparkMax(kLeaderMotorCANID, MotorType.kBrushless);
    private SparkMax m_elevatorFollowerMotor = new SparkMax(kFollowerMotorCANID, MotorType.kBrushless);
    private ProfiledPIDController elevatorController = new ProfiledPIDController(
        kControllerkP, 
        kControllerkI, 
        kControllerkD, 
        new Constraints(kMaxVelocity, kMaxAcceleration)
    );
    private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        kFeedForwardkS, 
        kFeedForwardkG, 
        kFeedForwardkV, 
        kFeedForwardkA
    );
    // enum Level {
    //     L1(0),
    //     L2(10),
    //     L3(20),
    //     L4(30);

    //     private double encoderValue;
    //     private Level(double encoderValue){
    //         this.encoderValue = encoderValue;
    //     }
    // }
    public Elevator(){
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SmartDashboard.putData(elevatorController);
        followerConfig.follow(this.m_elevatorLeaderMotor);
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.inverted(true); //TODO check if this is needed
        
        SoftLimitConfig softLimit = new SoftLimitConfig();
        softLimit.forwardSoftLimit(kForwardSoftLimit);
        softLimit.reverseSoftLimit(kReverseSoftLimit);
        leaderConfig.softLimit.apply(softLimit);
        leaderConfig.idleMode(IdleMode.kBrake);

        this.m_elevatorFollowerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.m_elevatorLeaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void setVoltage(double volts){
        this.m_elevatorLeaderMotor.setVoltage(volts);
    }

    public void setPercentOutput(double percentOutput){
        this.m_elevatorFollowerMotor.set(percentOutput);
    }

    public double getEncoderDistance(){
        return this.m_elevatorLeaderMotor.getEncoder().getPosition();
    }

    public boolean isAtGoalPosition(){
        return false;
    }

    public Command getPositionCommand(double goalEncoderPosition){
        return Commands.startRun(
            //intitialize
            () -> this.elevatorController.setGoal(goalEncoderPosition), 
            //execute
            () -> {
                double pidOutput = this.elevatorController.calculate(this.getEncoderDistance());
                double feedForwardOutput = this.elevatorFeedForward.calculate(
                    this.elevatorController.getSetpoint().velocity
                );
                this.setVoltage(
                    pidOutput + feedForwardOutput
                );
                DogLog.log("elevator output ", pidOutput + feedForwardOutput);
            },
            this
        );
    }

    public void tuneElevator(){
        MotorTuner.NativeController.tunablePID("Elevator Controller", elevatorController);
        MotorTuner.NativeController.tunableElevatorFeedforward("Elevator Feedforward", () -> this.elevatorFeedForward, updatedVal -> this.elevatorFeedForward = updatedVal);
    }
}
