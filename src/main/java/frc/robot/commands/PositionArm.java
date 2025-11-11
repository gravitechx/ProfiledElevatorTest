package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class PositionArm extends SequentialCommandGroup{
    public PositionArm(
        Elevator elevator, double elevatorPosition,
        Arm arm, double armPosition, double wristPosition,
        Intake intake, double intakeSpeed
    ){
        addCommands(
            elevator.getPositionCommand(elevatorPosition),
            arm.getPositionCommand(armPosition, wristPosition),
            intake.getOuttakeCommand(intakeSpeed)
        );
    }
}
