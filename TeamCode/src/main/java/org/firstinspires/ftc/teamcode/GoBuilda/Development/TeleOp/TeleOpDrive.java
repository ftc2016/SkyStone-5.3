package org.firstinspires.ftc.teamcode.GoBuilda.Development.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Scrim 4 TeleTubbies", group = "actual")
public class TeleOpDrive extends OpMode
{

    //Initializing all necessary variables
    float y;
    float x;

    private double rotnPosScale = 20, rotnPowScale = .002;
    private double extendPosScale = 5, extendPowScale = 0.002;
    double armCommand, armPosIntegrator = 0;

    private double armPosCurrent, armPosDes, armPosError;
    private double extendPosCurrent, extendPosDes, extendPosError;


    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;
    DcMotor armRotate;
    DcMotor armExtend;

    private ColorSensor colorLeft, colorRight;

    Servo grasp, angle, foundation;

    double position = 0, ANGLE = 0.731992018;

    @Override
    public void loop()
    {
        init();

        // initializing wheel variables
        float powerXWheels = 0;
        float powerYWheels = 0;

        // Handle regular movement
        powerYWheels += gamepad1.left_stick_y;

        // Handle sliding movement
        powerXWheels += gamepad1.right_stick_x;


        // Handle turning movement
        double maxX = (double) powerXWheels;
        double maxY = (double) powerYWheels;


        MotorBackX.setPower((Math.abs(maxX)*maxX));
        MotorFrontX.setPower((Math.abs(maxX)*maxX));

        MotorBackY.setPower((Math.abs(maxY)*maxY));
        MotorFrontY.setPower((Math.abs(maxY)*maxY));

        //Turning clockwise
        float rotationCW = gamepad1.right_trigger;
        float rotationACW = gamepad1.left_trigger;

        if(rotationACW !=0)
        {
            MotorFrontX.setPower(-0.5);
            MotorFrontY.setPower(0.5);
            MotorBackX.setPower(0.5);
            MotorBackY.setPower(-0.5);
        }

        //Turning anticlockwise
        if (rotationCW !=0)
        {
            MotorFrontX.setPower(0.5);
            MotorFrontY.setPower(-0.5);
            MotorBackX.setPower(-0.5);
            MotorBackY.setPower(0.5);
        }

        if (gamepad2.dpad_up&&position < 1.1)
        {
            ANGLE += 0.02;
        }
        if (gamepad2.dpad_down && position > -0.1)
        {
            ANGLE += -0.02;
        }
        if (gamepad2.left_bumper)
        {
            grasp.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            grasp.setPosition(0);
        }

        if(gamepad1.dpad_up)
        {
            foundation.setPosition(0);
        }
        else if(gamepad1.dpad_down)
        {
            foundation.setPosition(1);
        }

        angle.setPosition(-0.00027*(armRotate.getCurrentPosition())+ ANGLE);

        armPosCurrent = armRotate.getCurrentPosition();

        armPosDes += rotnPosScale * gamepad2.right_stick_y;
        armPosError = armPosDes - armPosCurrent;
        armPosIntegrator += 0.00001*armPosError;
        armCommand = Math.min(Math.max(rotnPowScale*armPosError + armPosIntegrator, -1.00), 1.00); //gain
        armRotate.setPower(armCommand);

        armExtend.setPower(gamepad2.left_stick_x * Math.abs(gamepad2.left_stick_x));

    }



    @Override
    public void init()
    {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");
        armExtend = hardwareMap.dcMotor.get("extend");
        armRotate = hardwareMap.dcMotor.get("rotate");

        MotorFrontX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);
        armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grasp = hardwareMap.servo.get("grasp");
        foundation = hardwareMap.servo.get("foundation");
        angle = hardwareMap.servo.get("angle");

        colorLeft = hardwareMap.get(ColorSensor.class, "left");
        colorRight = hardwareMap.get(ColorSensor.class, "right");

    }
}
