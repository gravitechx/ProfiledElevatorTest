package frc.robot.commands;

import static frc.robot.Constants.Elevator.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveToLevel extends Command{
    private ProfiledPIDController elevatorController;
    private ElevatorFeedforward elevatorFeedForward;
    private Elevator elevatorSub;
    private double encoderGoalPosition = 0;
    
    public MoveToLevel(Elevator elevatorSub, double encoderGoalPosition){
        this.elevatorController = new ProfiledPIDController(
            kControllerkP, 
            kControllerkI, 
            kControllerkD, 
            new Constraints(kMaxVelocity, kMaxAcceleration)
        );
        this.elevatorFeedForward = new ElevatorFeedforward(
            kFeedForwardkS,
            kFeedForwardkG,
            kFeedForwardkV,
            kFeedForwardkA
        );
        this.elevatorSub = elevatorSub;
        this.encoderGoalPosition = encoderGoalPosition;
    }
    @Override
    public void initialize(){
        this.elevatorController.setGoal(encoderGoalPosition);
    }
    @Override
    public void execute(){
        double pidOutput = this.elevatorController.calculate(this.elevatorSub.getEncoderDistance());
        double feedforwardOutput = this.elevatorFeedForward.calculate(
            this.elevatorController.getSetpoint().velocity
        );
        this.elevatorSub.setVoltage(
            pidOutput + feedforwardOutput
        );
    }
}
