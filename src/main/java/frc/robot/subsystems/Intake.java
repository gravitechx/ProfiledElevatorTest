package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(0);
    public Intake(){
        this.intakeMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        );
    }

    public void setOutput(double output){
        this.intakeMotor.set(output);
    }

    public double getEncoderRate(){
        return this.intakeMotor.getVelocity().getValueAsDouble();
    }

    public Command getOuttakeCommand(double outputPercentage){
        return new InstantCommand(() -> this.setOutput(outputPercentage));
    }
}
