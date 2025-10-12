package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase{
    private SparkMax m_elevatorLeaderMotor = new SparkMax(kLeaderMotorCANID, MotorType.kBrushless);
    private SparkMax m_elevatorFollowerMotor = new SparkMax(kFollowerMotorCANID, MotorType.kBrushless);
    public Elevator(){
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        SoftLimitConfig softLimit = new SoftLimitConfig();
        softLimit.forwardSoftLimit(kForwardSoftLimit);
        softLimit.reverseSoftLimit(kReverseSoftLimit);

        followerConfig.follow(this.m_elevatorLeaderMotor);
        followerConfig.idleMode(IdleMode.kBrake);
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.softLimit.apply(new SoftLimitConfig());

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
}
