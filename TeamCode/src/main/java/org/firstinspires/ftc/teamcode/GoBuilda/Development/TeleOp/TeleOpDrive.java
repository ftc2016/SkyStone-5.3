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

    private double rotnPosScale = 20, rotnPowScale = .002;
    private double extendPosScale = 250, extendPowScale = 0.002;
    double armCommand, armPosIntegrator = 0;

    private double armPosCurrent, armPosDes, armPosError;
    private double extendPosCurrent, extendPosDes, extendPosError, extendPow;

    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double foundPos = 0;



    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;
    DcMotor armRotate;
    DcMotor armExtend;

    private ColorSensor colorLeft, colorRight;

    Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;

    double position = 0, ANGLE = 0.531992018, blockPos = 1  ;

    @Override
    public void loop()
    {
        if(turtle)
            multiplier = 0.45;
        else
            multiplier = 1;

        if(sloth)
            speedK = 0.45;
        else
            speedK = 1;

        // initializing wheel variables
        double powerXWheels = 0;
        double powerYWheels = 0;

        // Handle regular movement
        powerYWheels = gamepad1.left_stick_y;

        // Handle sliding movement
        powerXWheels = gamepad1.right_stick_x;

        MotorBackX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);
        MotorFrontX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);

        MotorBackY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);
        MotorFrontY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);

        //Turning clockwise
        float rotationCW = gamepad1.right_trigger;
        float rotationACW = gamepad1.left_trigger;

        if(gamepad1.a)
            turtle = !turtle;

        if(gamepad2.a)
            sloth = !sloth;

        if(rotationACW !=0)
        {
            MotorFrontX.setPower(-gamepad1.left_trigger);
            MotorFrontY.setPower(gamepad1.left_trigger);
            MotorBackX.setPower(gamepad1.left_trigger);
            MotorBackY.setPower(-gamepad1.left_trigger);
        }

        //Turning anticlockwise
        if (rotationCW !=0)
        {
            MotorFrontX.setPower(gamepad1.right_trigger);
            MotorFrontY.setPower(-gamepad1.right_trigger);
            MotorBackX.setPower(-gamepad1.right_trigger);
            MotorBackY.setPower(gamepad1.right_trigger);
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
            grasp1.setPosition(1);
            grasp2.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            grasp1.setPosition(0);
            grasp2.setPosition(1);
        }

        if(gamepad1.dpad_left)
            blockPos += 0.05;
        if(gamepad1.dpad_right)
            blockPos -= 0.05;

        leftCollection.setPosition(blockPos);
        rightCollection.setPosition(1-blockPos);

        if(gamepad1.dpad_up) { foundPos += 0.05; }

        if(gamepad1.dpad_down) {  foundPos -= 0.05; }

        foundation.setPosition(foundPos);

        telemetry.addData("Foundation Data", foundPos);
        telemetry.addData("Y power", powerYWheels*multiplier);
        telemetry.update();

        angle1.setPosition(-0.00027*(armRotate.getCurrentPosition())+ ANGLE);
        angle2.setPosition(1-(-0.00027*(armRotate.getCurrentPosition())+ ANGLE));

        telemetry.addData("Value", blockPos );

        armPosCurrent = armRotate.getCurrentPosition();

        armPosDes += rotnPosScale * speedK * gamepad2.right_stick_y;
        armPosError = armPosDes - armPosCurrent;
        armPosIntegrator += 0.00001*armPosError;
        armCommand = Math.min(Math.max(rotnPowScale*armPosError + armPosIntegrator, -1.00), 1.00); //gain
        armRotate.setPower(armCommand);


        extendPosCurrent = armExtend.getCurrentPosition();

        extendPosDes += extendPosScale * gamepad2.left_stick_x;
        extendPosError = extendPosDes - extendPosCurrent;
        extendPow = Math.min(Math.max(extendPowScale*extendPosError, -1.00), 1.00);
        armExtend.setPower(extendPow);
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

        colorLeft = hardwareMap.get(ColorSensor.class, "left");
        colorRight = hardwareMap.get(ColorSensor.class, "right");

        colorLeft.enableLed(false);
        colorRight.enableLed(false);
    }
}
