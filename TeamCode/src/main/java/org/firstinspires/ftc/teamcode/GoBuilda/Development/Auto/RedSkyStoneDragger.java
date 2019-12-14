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

import java.util.Collection;

@Autonomous(name="Red SkyStoneDragger", group = "Red")
public class RedSkyStoneDragger extends LinearOpMode
{

    //initalizing sensors
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
    //gyro init
    BNO055IMU imu;
    Orientation angles;
    char blockPos = ' ';

    //initializing motors
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;


    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeMotors();
        initSensors();

        waitForStart();

        detectBlock();

        moveY(1,0.2);

        collectBlock("right","down");
        Thread.sleep(1500);

        moveY(-13, 0.2);
        moveX(-66, 0.2);

       collectBlock("right","up");
       moveX(74, 0.2);

        moveSetDistance();

        collectBlock("left","down");
        Thread.sleep(2000);

        moveY(-15, 0.2);
        Thread.sleep(500);
        moveX(-78, 0.5);

        collectBlock("left", "up");
        Thread.sleep(500);

        moveX(15, 0.2);
    }


    private void initializeMotors() {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");
        motorExtend = hardwareMap.dcMotor.get("extend");
        motorRotate = hardwareMap.dcMotor.get("rotate");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);


        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grasp1 = hardwareMap.servo.get("grasp1");
        grasp2 = hardwareMap.servo.get("grasp2");
        rightCollection = hardwareMap.servo.get("rightCollection");
        leftCollection = hardwareMap.servo.get("leftCollection");
        foundation = hardwareMap.servo.get("foundation");
        angle1 = hardwareMap.servo.get("angle1");
        angle2 = hardwareMap.servo.get("angle2");

//        grasp1.setPosition(1);
//        grasp2.setPosition(0);

    }

    private void initSensors()
    {

        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
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
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 50;

        while (current >= DESIRED_D)
        {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
            MotorFrontX.setPower(0);
            MotorBackX.setPower(0);
        }

        MotorFrontY.setPower(-0.1);
        MotorBackY.setPower(-0.1);

        telemetry.addData("MotorFrontX power", MotorFrontX.getPower());
        telemetry.addData("MotorFrontY power", MotorFrontY.getPower());
        telemetry.addData("MotorBackX power", MotorBackX.getPower());
        telemetry.addData("MotorackY power", MotorBackY.getPower());
        telemetry.update();

        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFrontY.setPower(0);
        MotorBackY.setPower(0);
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

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
        }

        MotorFrontY.setPower(-0.1);
        MotorBackY.setPower(-0.1);

        Thread.sleep(2000);

        double leftNormalizedColors = (leftColor.green() + leftColor.red() + leftColor.blue()) / Math.pow(leftDistance.getDistance(DistanceUnit.MM), 2);
        double rightNormalizedColors = (rightColor.green() + rightColor.red() + rightColor.blue()) / Math.pow(rightDistance.getDistance(DistanceUnit.MM), 2);

        Log.i("LeftNormalizationValue", ""+leftNormalizedColors);
        Log.i("RightNormalizationValue", ""+rightNormalizedColors);

        Thread.sleep(500);

        telemetry.addData("Left ", leftNormalizedColors);
        telemetry.addData("Right", rightNormalizedColors);
        telemetry.update();

        // Yellow is greater than 0.5 and black is less than 0.5
        //Y[YB]
        if (leftNormalizedColors > 0.5 && rightNormalizedColors < 0.5)
        {
            blockPos = 'r';
            moveX(2, 0.2);
            telemetry.addData("right", null);
            telemetry.update();
        }

        //Y[BY]
        if (leftNormalizedColors < 0.5 && rightNormalizedColors > 0.5)
        {
            blockPos = 'c';
            moveX(9, 0.2);
            telemetry.addData("center", null);
            telemetry.update();
        }

        //B[YY]
        if (leftNormalizedColors > 0.5 && rightNormalizedColors > 0.5)
        {
            blockPos = 'l';
            moveX(18, 0.2);
            telemetry.addData("left", null);
            telemetry.update();
        }

        Thread.sleep(500);
        telemetry.update();
        Thread.sleep(500);
    }

    void agaga(String position) {
        if (position.equals("release"))
        {
            angle1.setPosition(1);
            angle2.setPosition(0);

            grasp1.setPosition(1);
            grasp2.setPosition(0);

            angle1.setPosition(0);
            angle1.setPosition(1);
        }

        if (position.equals("grasp"))
        {
            grasp1.setPosition(0);
            grasp2.setPosition(1);
        }
    }

    void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy()) {
            telemetry.addData("Move Y method error = ", MotorFrontY.getTargetPosition() - (MotorFrontY.getCurrentPosition() + counts));
        }
    }

    private void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Move X method error = ", MotorFrontX.getTargetPosition() - (MotorFrontX.getCurrentPosition() + counts));
            telemetry.update();
        }
    }

    void botRotate(int distance, double power)
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
    }


    void armExtend(int counts, double power)
    {
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setPower(power);

        motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + counts);

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorExtend.isBusy()) {
            telemetry.addData("extension error = ", motorExtend.getTargetPosition() - (motorExtend.getCurrentPosition() + counts));
            telemetry.update();
            if((motorExtend.getCurrentPosition()> motorExtend.getTargetPosition()-20) && (motorExtend.getCurrentPosition()<motorExtend.getTargetPosition()+20))
                break;
        }
    }

    void armRotate(int counts, double power)
    {

        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setPower(power);

        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() + counts);

        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorRotate.isBusy())
        {
            telemetry.addData(" rotation error = ", motorRotate.getTargetPosition() - (motorRotate.getCurrentPosition() + counts));
            telemetry.update();
            if((motorRotate.getCurrentPosition()> motorRotate.getTargetPosition()-20) && (motorRotate.getCurrentPosition()<motorRotate.getTargetPosition()+20))
                break;
        }
    }

    void collectBlock(String servo, String pos) throws InterruptedException
    {
        if(servo.equalsIgnoreCase("right"))
        {
            if (pos.equalsIgnoreCase("down"))
                rightCollection.setPosition(0.6);

            if (pos.equalsIgnoreCase("up"))
                rightCollection.setPosition(0);
        }

        if(servo.equalsIgnoreCase("left"))
        {
            if (pos.equalsIgnoreCase("down"))
                leftCollection.setPosition(0.35);

            if (pos.equalsIgnoreCase("up"))
                leftCollection.setPosition(1);
        }
    }
}