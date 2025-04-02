package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftToggleCommand extends CommandBase {
    private final Lift lift;

    public LiftToggleCommand(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.toggle();
    }

    @Override
    public void execute() {
        lift.update();
    }

    @Override
    public boolean isFinished() {
        return lift.isAtTarget();
    }
}
