package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;

@Autonomous(name = "Blue Foundation Mover", group = "Blue")
public class BlueFoundationMover extends LinearOpMode
{

    private GoBuildaUtil robot = new GoBuildaUtil();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.initializeAuto(hardwareMap);
        robot.initializeSensors(hardwareMap);

        waitForStart();

        robot.foundation.setPosition(0.6);

        robot.moveX(-14, 0.2);
        robot.moveY(37,0.2);
        sleep(500);

        robot.foundation.setPosition(0.2);
        sleep(500);

        robot.moveY(-40, 0.15);
        sleep(500);

        robot.foundation.setPosition(0.6);
        robot.moveX(58, 0.3);
        robot.moveY(-2, 0.5);
    }
}
