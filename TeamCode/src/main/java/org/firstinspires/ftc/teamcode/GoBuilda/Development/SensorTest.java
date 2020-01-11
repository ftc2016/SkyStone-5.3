package org.firstinspires.ftc.teamcode.GoBuilda.Development;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "ColorSensor")
public class SensorTest extends OpMode
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

    //To prove that I changed this file

    public void init()
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

        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop()
    {
//            double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
//            final int DESIRED_D = 65;

                double leftNormalizedColors = (leftColor.red()*leftColor.green())/Math.pow(leftColor.blue(),2);
                double rightNormalizedColors = (rightColor.red()*rightColor.green()/ Math.pow(rightColor.blue(),2));
                telemetry.addData("Left ", leftNormalizedColors);
                telemetry.addData("Right", rightNormalizedColors);
                telemetry.update();

                // Yellow is greater than 0.5 and black is less than 0.5
                //Y[YB]
                if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
                {
                    telemetry.addData("right", null);
                    telemetry.update();
                }

                //Y[BY]
                if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
                {
                    telemetry.addData("center", null);
                    telemetry.update();
                }

                //B[YY]
                if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
                {
                    telemetry.addData("left", null);
                    telemetry.update();
                }
            }
        }