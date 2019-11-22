package org.firstinspires.ftc.teamcode.GoBuilda.Development;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensor")
public class SensorTest extends LinearOpMode
{
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;

    private final int redThreshold = 1000, greenThreshold = 2000;

    private float hsvValues[] = {0F, 0F, 0F};
    private final float values[] = hsvValues;
    private boolean isLeft = false, isCenter = false;

    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        waitForStart();

        detectBlock();
    }

    public void initialize()
    {
        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");

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

    }

    public void detectBlock()
    {

        moveX(-1.0, 0.25);

        while(( (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM))/2.0) >= 60)
        {
            MotorFrontY.setPower(0.1);
            MotorBackY.setPower(0.1);
        }

        if(leftColor.red()<redThreshold && leftColor.green()<greenThreshold)
        {
            telemetry.addData("CENTER", null);
            telemetry.update();
        }

        if (rightColor.red()<redThreshold && leftColor.green()<greenThreshold)
        {
            telemetry.addData("RIGHT", null);
            telemetry.update();
        }

        telemetry.addData("LEFT COLOR", leftColor.red());
        telemetry.addData("LEFT COLOR", leftColor.green());
        telemetry.addData("LEFT COLOR", leftColor.blue());
        telemetry.addData("LEFT Distance", leftDistance.getDistance(DistanceUnit.MM));

        telemetry.addData("RIGHT COLOR", rightColor.red());
        telemetry.addData("RIGHT COLOR", rightColor.green());
        telemetry.addData("RIGHT COLOR", rightColor.blue());
        telemetry.addData("RIGHT Distance", rightDistance.getDistance(DistanceUnit.MM));
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


    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;

        return (int)(COUNTS_PER_REVOLUTION*inches);
    }


}

    //    @Override
//    public void loop()
//    {
//
//        if(leftColor.red()<redThreshold && leftColor.green()<greenThreshold)
//        {
//            isLeft = true;
//            telemetry.addData("leftColor", null);
//            telemetry.update();
//        }
//
//        if (rightColor.red()<redThreshold && leftColor.green()<greenThreshold)
//        {
//            isCenter = true;
//            telemetry.addData("rightColor", null);
//            telemetry.update();
//        }
//
//    }


