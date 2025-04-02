package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideToPosCommand extends CommandBase {
    private final Slide slide;
    private final double target;

    public SlideToPosCommand(Slide slide, double target) {
        this.slide = slide;
        addRequirements(slide);
        this.target = target;
    }

    @Override
    public void initialize() {
        slide.setHeight(target);
    }

    @Override
    public void execute() {
        slide.update();
    }

    @Override
    public void end(boolean interrupted) {
        slide.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return slide.isAtTarget();
    }
}
