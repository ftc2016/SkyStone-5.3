package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto_Test;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utils.UtilTest;




@Autonomous(name="Red Test", group = "Red")
public class Red_Stone extends LinearOpMode
{

   private UtilTest robot = new UtilTest();

    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.initializeAuto(hardwareMap);
        robot.initializeSensors(hardwareMap);

        waitForStart();

        detectBlock();

        robot.armVertical(50,0.75);
        robot.armHorizontal(70,1);
        robot.armVertical(-50, 0.75);

    }

    private void detectBlock() throws InterruptedException {

        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 55;

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

        Log.i("LeftNormalizationValue", ""+leftNormalizedColors);
        Log.i("RightNormalizationValue", ""+rightNormalizedColors);

        Thread.sleep(500);

        telemetry.addData("Left ", leftNormalizedColors);
        telemetry.addData("Right", rightNormalizedColors);
        telemetry.update();

        // Yellow is greater than 0.5 and black is less than 0.5
        //Y[YB]
        if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
        {
            robot.moveX(3, 0.2);
            telemetry.addData("right", null);
            telemetry.update();
        }

        //Y[BY]
        if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
        {
            robot.moveX(-1, 0.2);
            telemetry.addData("center", null);
            telemetry.update();
        }

        //B[YY]
        if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
        {
            robot.moveX(-9, 0.2);
            telemetry.addData("left", null);
            telemetry.update();
        }

        Thread.sleep(500);
        telemetry.update();
        Thread.sleep(500);

        robot.moveY(1,0.275);

        armDeploy();
    }

    public void armDeploy()
    {
        robot.armVertical(5, 0.5);
        sleep(1000);
        robot.armHorizontal(2, 0.75);
        sleep(1000);
        robot.armVertical(5, 0.25);
    }

    public void armRetreat(double inches, double power)
    {
        robot.armVertical(2, 0.1);
        robot.armHorizontal(-2, 0.75);
        robot.armVertical(1, 0.1);
    }


}
