package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

@Autonomous(name = "Blue DoesItAll", group = "Blue")
public class BlueDoesItAll extends LinearOpMode
{

    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;

    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp, angle, foundation1, foundation2;


    @Override
    public void runOpMode() throws InterruptedException
    {

    }

    private void initializeMotors()
    {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");

        MotorFrontX.setDirection(DcMotor.Direction.REVERSE);
        MotorBackX.setDirection(DcMotor.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotor.Direction.FORWARD);
        MotorBackY.setDirection(DcMotor.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFrontY.setZeroPowerBehavior(BRAKE);
        MotorBackY.setZeroPowerBehavior(BRAKE);
        MotorBackX.setZeroPowerBehavior(BRAKE);
        MotorFrontX.setZeroPowerBehavior(BRAKE);


        motorExtend = hardwareMap.dcMotor.get("extend");
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate = hardwareMap.dcMotor.get("rotate");
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRotate.setZeroPowerBehavior(BRAKE);

        grasp = hardwareMap.servo.get("grasp");
        foundation1 = hardwareMap.servo.get("foundation1");
        foundation2 = hardwareMap.servo.get("foundation2");
        angle = hardwareMap.servo.get("angle");

        grasp.setPosition(1);
        angle.setPosition(0.65);
        foundation1.setPosition(0);
        foundation2.setPosition(0);
    }

    private void initSensors()
    {
        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
    }

    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;

        return (int)(COUNTS_PER_REVOLUTION*inches);
    }

    private void detectBlock()
    {
        char blockPos = ' ';
        MotorFrontY.setZeroPowerBehavior(BRAKE);
        MotorBackY.setZeroPowerBehavior(BRAKE);

        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM))/2;
        final int DESIRED_D = 50;

        while(current >= DESIRED_D)
        {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM))/2;

            MotorFrontY.setPower(0.15);
            MotorBackY.setPower(0.15);
        }

        MotorFrontY.setPower(0);
        MotorBackY.setPower(0);

        double leftNormalizedColors = (leftColor.green()+leftColor.red()+leftColor.blue())/Math.pow(leftDistance.getDistance(DistanceUnit.MM),2);
        double rightNormalizedColors = (rightColor.green()+rightColor.red()+rightColor.blue())/Math.pow(rightDistance.getDistance(DistanceUnit.MM),2);

        if (rightNormalizedColors<0.5&&leftNormalizedColors>0.5)
        {
            blockPos = 'r';
            telemetry.addData("right", null);
            sleep(1000);
        }

        if(leftNormalizedColors<0.5&&rightNormalizedColors>0.5)
        {
            blockPos = 'c';
            telemetry.addData("center", null);
            sleep(1000);
        }

        if(leftNormalizedColors<0.5&&rightNormalizedColors<0.5)
        {
            blockPos = 'l';
            telemetry.addData("left", null);
            sleep(1000);
        }
        sleep(1000);
        telemetry.update();
        sleep(1000);

        switch(blockPos)
        {
            case 'r': moveX(-4, 0.2);
                break;
            case 'c': moveX(4, 0.2);
                break;
            case 'l': moveX(12, 0.2);
                break;
            default:
                moveY(-2, 0.7);
                detectBlock();
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

        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running motor Y front and back", "Encoders");
            telemetry.update();
        }
    }

    void moveX(double inches, double power)
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
            telemetry.addData("Running motor X front and back", "Encoders");
            telemetry.update();
        }
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
    }


    void armExtend(int counts, double power)
    {
        motorExtend.setPower(power);

        motorExtend.setTargetPosition(motorExtend.getCurrentPosition()+counts);

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && motorExtend.isBusy())
        {
            telemetry.addData("arm is extending to position", counts);
        }
    }

    void armRotate(int counts, double power)
    {
        motorRotate.setPower(power);

        motorRotate.setTargetPosition(motorRotate.getCurrentPosition()+counts);

        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && motorRotate.isBusy())
        {
            telemetry.addData("arm is rotating to position", counts);
        }
    }

    void collectBlock()
    {
        moveY(-6, 0.3);
        armRotate(600, 0.75);
        moveY(7, 0.6);
    }
}
