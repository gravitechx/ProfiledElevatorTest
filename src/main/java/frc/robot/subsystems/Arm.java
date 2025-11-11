package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorTuner;

public class Arm extends SubsystemBase{
    private TalonFX pivotMotor = new TalonFX(0);
    private SparkMax wristMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();
    public Arm(){
        
    }

    public Command setPosition(double pivotPosition, double wristPosition){
        return Commands.run(() -> {
            this.pivotMotor.setPosition(pivotPosition);
            this.wristController.setReference(wristPosition, ControlType.kPosition);
        }, this);
    }

    public double getPivotPosition(){
        return this.pivotMotor.getPosition().getValueAsDouble();
    }

    public double getWristPosition(){
        return this.wristMotor.getEncoder().getPosition();
    }

    public void tuneArm(){
        MotorTuner.SparkMaxMotor.tunablePIDF("Wrist", wristMotor);
        MotorTuner.TalonMotor.tunablePID("Pivot", pivotMotor, new Slot0Configs());
    }
}
