package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideToBasketPosCommand extends CommandBase {
    private final Slide slide;

    public SlideToBasketPosCommand(Slide slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.setHeight(Slide.PAST_BASKET_POS);
    }

    @Override
    public boolean isFinished() {
        return slide.isAtTarget();
    }
}
