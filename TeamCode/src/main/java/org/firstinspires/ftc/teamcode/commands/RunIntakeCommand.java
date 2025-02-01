package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RunIntakeCommand extends CommandBase {
    private final Intake intake;
    private final boolean isForward;

    public RunIntakeCommand(Intake intake, boolean isForward) {
        this.intake = intake;
        this.isForward = isForward;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setForward(isForward);
        intake.setRunning(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRunning(false);
    }
}
