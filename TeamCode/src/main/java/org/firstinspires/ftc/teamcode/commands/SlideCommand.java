package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideCommand extends CommandBase {
    private final Slide slide;
    private final GamepadEx gamepad;

    public SlideCommand(Slide slide, GamepadEx gamepad) {
        this.slide = slide;
        this.gamepad = gamepad;
        addRequirements(slide);
    }

    @Override
    public void execute() {
        double left = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double right = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        slide.setSpeed(right - left);
        slide.update();
    }
}
