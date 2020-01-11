package org.firstinspires.ftc.teamcode.Utils;

import android.hardware.Sensor;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class UtilTest
{
    HardwareMap hardware;

    public ColorSensor leftColor, rightColor;
    public DistanceSensor leftDistance, rightDistance;
    //gyro init
    public BNO055IMU imu;
    public Orientation angles;
    public char blockPos = ' ';

    //initializing motors
    public static DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorVertical, motorExtend;
    public static Servo graspL, graspR, block_drag_grasp, block_drag;


    public UtilTest() { }

    public void initializeTele(HardwareMap hw)
    {
        hardware = hw;

        MotorFrontX = hardware.dcMotor.get("fx");
        MotorBackX = hardware.dcMotor.get("bx");
        MotorFrontY = hardware.dcMotor.get("fy");
        MotorBackY = hardware.dcMotor.get("by");
        motorVertical = hardware.dcMotor.get("vertical");
        motorExtend = hardware.dcMotor.get("extend");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
        motorVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        graspL = hardware.servo.get("graspL");
        graspR = hardware.servo.get("graspR");
        block_drag_grasp = hardware.servo.get("block_drag_grasp");
        block_drag = hardware.servo.get("block_drag");
    }

    public void initializeAuto(HardwareMap hw)
    {
        hardware = hw;

        MotorFrontX = hardware.dcMotor.get("fx");
        MotorBackX = hardware.dcMotor.get("bx");
        MotorFrontY = hardware.dcMotor.get("fy");
        MotorBackY = hardware.dcMotor.get("by");
        motorVertical = hardware.dcMotor.get("vertical");
        motorExtend = hardware.dcMotor.get("extend");


        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);
        motorVertical.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        graspL = hardware.servo.get("graspL");
//        graspR = hardware.servo.get("graspR");
//        block_drag_grasp = hardware.servo.get("block_drag_grasp");
//        block_drag = hardware.servo.get("block_drag");
//        foundation1 = hardwareMap.servo.get("foundation1");
//        foundation2 = hardwareMap.servo.get("foundation2");
    }

    public void initializeSensors(HardwareMap hw)
    {
         hardware = hw;

        leftColor = hardware.get(ColorSensor.class, "left");
       rightColor = hardware.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardware.get(DistanceSensor.class, "left");
       rightDistance = hardware.get(DistanceSensor.class, "right");
    }



    public void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (MotorBackX.isBusy() && MotorFrontX.isBusy())
        { }
    }

    public void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (MotorBackY.isBusy() && MotorFrontY.isBusy()) { }
    }

    void botRotate(int distance, int power)
    {
        MotorBackY.setPower(power);
        MotorBackX.setPower(power);
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);

        int COUNTS = inchesToCounts(distance);

        MotorFrontX.setTargetPosition(MotorFrontX.getCurrentPosition() + (COUNTS));
        MotorBackX.setTargetPosition(MotorBackX.getCurrentPosition() - (COUNTS));
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() + COUNTS);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - COUNTS);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(MotorBackY.isBusy() && MotorFrontY.isBusy() && MotorBackX.isBusy() && MotorFrontX.isBusy()) { }
    }

    public void armVertical(int counts, double power)
    {
        motorVertical.setPower(-power);

        motorVertical.setTargetPosition((motorVertical.getCurrentPosition() + counts));

        motorVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorVertical.isBusy())
        { }
    }

    public void armHorizontal(int counts, double power)
    {
        motorExtend.setPower(-power);

        motorExtend.setTargetPosition((MotorFrontY.getCurrentPosition() + counts));

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorExtend.isBusy())
        { }
    }

    public void armDeploy()
    {
        armVertical(1, 0.1);
        armHorizontal(2, 0.75);
        armVertical(2, 0.1);
    }

    public void armRetreat(double inches, double power)
    {
        armVertical(2, 0.1);
        armHorizontal(-2, 0.75);
        armVertical(1, 0.1);
    }

    public int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void detectBlock() throws InterruptedException {

        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 55;

        while (current >= DESIRED_D)
        {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
        }

        MotorFrontY.setPower(-0.1);
        MotorBackY.setPower(-0.1);

        Thread.sleep(2000);

        double leftNormalizedColors = (leftColor.red()*leftColor.green())/Math.pow(leftColor.blue(),2);
        double rightNormalizedColors = (rightColor.red()*rightColor.green()/ Math.pow(rightColor.blue(),2));

        Log.i("LeftNormalizationValue", ""+leftNormalizedColors);
        Log.i("RightNormalizationValue", ""+rightNormalizedColors);

        Thread.sleep(500);

        // Yellow is greater than 0.5 and black is less than 0.5
        //Y[YB]
        if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
        {
            moveX(3, 0.2);
        }

        //Y[BY]
        if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
        {
            moveX(-1, 0.2);
        }

        //B[YY]
        if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
        {
            moveX(-9, 0.2);
        }

        Thread.sleep(500);

        moveY(1,0.275);
    }




}
