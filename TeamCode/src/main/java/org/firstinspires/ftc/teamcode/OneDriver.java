package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.SlideToBasketPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

/**
 * An op mode for 1 driver in the driver-controlled segment.
 */
@TeleOp
public class OneDriver extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepad = new GamepadEx(gamepad1);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Basket basket = new Basket(hardwareMap);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(new RunIntakeCommand(intake, false));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(new RunIntakeCommand(intake, true));
        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SlideToBasketPosCommand(slide).andThen(new InstantCommand(lift::toggle, lift)));
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(basket::toggle, basket));
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(drivetrain::resetImu, drivetrain));

        // The default commands will run whenever no other command requires the corresponding
        // subsystem, constantly updating the motor powers.
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, gamepad));
        lift.setDefaultCommand(new RunCommand(lift::update, lift));
        slide.setDefaultCommand(new SlideCommand(slide, gamepad));

        schedule(new RunCommand(telemetry::update));
    }
}