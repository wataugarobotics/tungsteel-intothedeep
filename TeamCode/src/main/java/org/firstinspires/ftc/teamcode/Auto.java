package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

import java.util.HashSet;

@Autonomous
public class Auto extends CommandOpMode {
    @Override
    public void initialize() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Basket basket = new Basket(hardwareMap);

        lift.setDefaultCommand(new RunCommand(lift::update, lift));

        schedule(new RunCommand(telemetry::update));

        schedule(new ActionCommand(drive.actionBuilder(beginPose).strafeTo(new Vector2d(5, -40)).build(), new HashSet<>()));
    }
}
