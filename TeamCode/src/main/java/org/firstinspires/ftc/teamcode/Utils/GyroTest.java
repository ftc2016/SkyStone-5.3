package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "good")
public class GyroTest extends LinearOpMode
{
        GoBuildaUtil robot = new GoBuildaUtil();
        GyroCode gyro = new GyroCode();

    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.initializeTele(hardwareMap);
        gyro.initGyro(hardwareMap);

        waitForStart();

        telemetry.addData("MotorPow", gyro.getRotationCorrection(0, 0.3));
        telemetry.update();
        gyro.rotationCorrection(0, 0.3, 1);
        telemetry.addData("MotorPow", gyro.getRotationCorrection(0, 0.3));
        telemetry.addData("FIRST ANGLE:", gyro.angles.firstAngle);
        telemetry.update();
        sleep(1000);
//        gyro.rotationCorrection(0, 0.3);
//        sleep(1000);
//        gyro.rotationCorrection(0, 0.3);


    }
    }
