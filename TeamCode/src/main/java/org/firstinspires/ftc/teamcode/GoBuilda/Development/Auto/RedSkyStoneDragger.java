package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;

import java.util.Collection;

@Autonomous(name="Red SkyStoneDragger", group = "Red")
public class RedSkyStoneDragger extends LinearOpMode
{
    
    char blockPos = ' ';

    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gc = new GyroCode();


    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.initializeAuto(hardwareMap);
        robot.initServos(hardwareMap);
        robot.initializeSensors(hardwareMap);
        gc.initGyro(hardwareMap);

        waitForStart();

        detectBlock();

        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "chomp");

        if(blockPos == 'l')
        {
            robot.moveY(-10, 0.2);
            moveX(-56, 0.4);
        }

        if(blockPos == 'c')
        {
            robot.moveY(-10, 0.2);
            moveX(-48, 0.4);
        }

        if(blockPos == 'r')
        {
            robot.moveY(-10, 0.2);
            moveX(-40, 0.4);
        }

//        dragBlock("rot","down");
//        Thread.sleep(1500);
        dragBlock("grasp", "unchomp");
        Thread.sleep(700);
        dragBlock("rot","up");
        Thread.sleep(700);

        if(blockPos == 'l')
        {
            moveX(76, 0.4);
        }

        if(blockPos == 'c')
        {
            moveX(68, 0.4);
        }

        if(blockPos == 'r')
        {
            moveX(60, 0.4);
        }

        Thread.sleep(250);
        moveSetDistance();

        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "chomp");

        robot.moveY(-10, 0.2);
        Thread.sleep(500);
        moveX(-75, 0.5);

        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "unchomp");
        Thread.sleep(700);
        dragBlock("rot","up");

        moveX(15, 0.5);
    }
    

    private int inchesToCounts(double inches) {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void moveSetDistance()
    {
        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 51;

        while (current >= DESIRED_D)
        {
            current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            robot.MotorFrontY.setPower(0.3);
            robot.MotorBackY.setPower(0.3);
            robot.MotorFrontX.setPower(0);
            robot.MotorBackX.setPower(0);
        }

        robot.MotorFrontY.setPower(-0.09);
        robot.MotorBackY.setPower(-0.09);

        telemetry.addData("robot.MotorFrontX power", robot.MotorFrontX.getPower());
        telemetry.addData("robot.MotorFrontY power", robot.MotorFrontY.getPower());
        telemetry.addData("robot.MotorBackX power", robot.MotorBackX.getPower());
        telemetry.addData("MotorackY power", robot.MotorBackY.getPower());
        telemetry.update();

        robot.MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.MotorFrontY.setPower(0);
        robot.MotorBackY.setPower(0);
    }

    private void detectBlock() throws InterruptedException
    {
        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 52;

        while (current >= DESIRED_D)
        {
            current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            robot.MotorFrontY.setPower(0.3);
            robot.MotorBackY.setPower(0.3);
        }

        robot.MotorFrontY.setPower(-0.1);
        robot.MotorBackY.setPower(-0.1);

        Thread.sleep(2000);

        double leftNormalizedColors = (robot.leftColor.red()*robot.leftColor.green())/Math.pow(robot.leftColor.blue(),2);
        double rightNormalizedColors = (robot.rightColor.red()*robot.rightColor.green()/ Math.pow(robot.rightColor.blue(),2));

        Thread.sleep(500);

        telemetry.addData("Left ", leftNormalizedColors);
        telemetry.addData("Right", rightNormalizedColors);
        telemetry.update();

        // Yellow is greater than 0.5 and black is less than 0.5
        //Y[YB]
        if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
        {
            moveX(-3, 0.2);
            telemetry.addData("right", null);
            telemetry.update();
            blockPos = 'r';

        }

        //Y[BY]
        if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
        {
            moveX(0, 0.2);
            telemetry.addData("center", null);
            telemetry.update();
            blockPos = 'c';
        }

        //B[YY]
        if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
        {
            moveX(9, 0.2);
            telemetry.addData("left", null);
            telemetry.update();
           blockPos = 'l';

        }

        Thread.sleep(500);
    }

    void agaga(String position) {
        if (position.equals("release"))
        {
            robot.graspL.setPosition(1);
            robot.graspR.setPosition(0);
        }

        if (position.equals("grasp"))
        {
            robot.graspL.setPosition(0);
            robot.graspR.setPosition(1);
        }
    }

    private void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        robot.MotorFrontX.setPower(power);
        robot.MotorBackX.setPower(power);

        robot.MotorFrontX.setTargetPosition((robot.MotorFrontX.getCurrentPosition() + (counts)));
        robot.MotorBackX.setTargetPosition((robot.MotorBackX.getCurrentPosition() + (counts)));

        robot.MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.MotorBackX.isBusy() && robot.MotorFrontX.isBusy())
        {
            gc.rotationCorrection(0);
        }
    }


//    void armExtend(int counts, double power)
//    {
//        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorExtend.setPower(power);
//
//        motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + counts);
//
//        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (opModeIsActive() && motorExtend.isBusy()) {
//            telemetry.addData("extension error = ", motorExtend.getTargetPosition() - (motorExtend.getCurrentPosition() + counts));
//            telemetry.update();
//            if((motorExtend.getCurrentPosition()> motorExtend.getTargetPosition()-20) && (motorExtend.getCurrentPosition()<motorExtend.getTargetPosition()+20))
//                break;
//        }
//    }

    void dragBlock(String servo, String pos) throws InterruptedException
    {
        if(servo.equalsIgnoreCase("grasp"))
        {
            if (pos.equalsIgnoreCase("chomp"))
                robot.block_drag_grasp.setPosition(1);

            if (pos.equalsIgnoreCase("unchomp"))
                robot.block_drag_grasp.setPosition(0);
        }

        if(servo.equalsIgnoreCase("rot"))
        {
            if (pos.equalsIgnoreCase("down"))
                robot.block_drag.setPosition(1);

            if (pos.equalsIgnoreCase("up"))
                robot.block_drag.setPosition(0);
        }
    }
}