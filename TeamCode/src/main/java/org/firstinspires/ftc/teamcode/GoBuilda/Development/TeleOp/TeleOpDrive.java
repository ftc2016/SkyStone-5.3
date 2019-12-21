package org.firstinspires.ftc.teamcode.GoBuilda.Development.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;

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

   GoBuildaUtil robot = new GoBuildaUtil();

    double position = 0, ANGLE = 0.531992018, blockPos = 1  ;

    @Override
    public void init()
    {
        robot.initializeTele(hardwareMap);
        robot.initializeSensors(hardwareMap);
    }

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

        robot.MotorBackX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);
        robot.MotorFrontX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);

        robot.MotorBackY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);
        robot.MotorFrontY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);

        //Turning clockwise
        float rotationCW = gamepad1.right_trigger;
        float rotationACW = gamepad1.left_trigger;

        if(gamepad1.a)
            turtle = !turtle;

        if(gamepad2.a)
            sloth = !sloth;

        if(rotationACW !=0)
        {
            robot.MotorFrontX.setPower(-gamepad1.left_trigger);
            robot.MotorFrontY.setPower(gamepad1.left_trigger);
            robot.MotorBackX.setPower(gamepad1.left_trigger);
            robot.MotorBackY.setPower(-gamepad1.left_trigger);
        }

        //Turning anticlockwise
        if (rotationCW !=0)
        {
            robot.MotorFrontX.setPower(gamepad1.right_trigger);
            robot.MotorFrontY.setPower(-gamepad1.right_trigger);
            robot.MotorBackX.setPower(-gamepad1.right_trigger);
            robot.MotorBackY.setPower(gamepad1.right_trigger);
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
            robot.grasp1.setPosition(1);
            robot.grasp2.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            robot.grasp1.setPosition(0);
            robot.grasp2.setPosition(1);
        }

        if(gamepad1.dpad_left)
            blockPos += 0.05;
        if(gamepad1.dpad_right)
            blockPos -= 0.05;

        robot.leftCollection.setPosition(blockPos);
        robot.rightCollection.setPosition(1-blockPos);

        if(gamepad1.dpad_up) { foundPos += 0.05; }

        if(gamepad1.dpad_down) {  foundPos -= 0.05; }

        robot.foundation.setPosition(foundPos);

        telemetry.addData("Foundation Data", foundPos);
        telemetry.addData("Y power", powerYWheels*multiplier);
        telemetry.update();

        robot.angle1.setPosition(-0.00027*(robot.motorRotate.getCurrentPosition())+ ANGLE);
        robot.angle2.setPosition(1-(-0.00027*(robot.motorRotate.getCurrentPosition())+ ANGLE));

        telemetry.addData("Value", blockPos );

        armPosCurrent = robot.motorRotate.getCurrentPosition();

        armPosDes += rotnPosScale * speedK * gamepad2.right_stick_y;
        armPosError = armPosDes - armPosCurrent;
        armPosIntegrator += 0.00001*armPosError;
        armCommand = Math.min(Math.max(rotnPowScale*armPosError + armPosIntegrator, -1.00), 1.00); //gain
        robot.motorRotate.setPower(armCommand);


        extendPosCurrent = robot.motorExtend.getCurrentPosition();

        extendPosDes += extendPosScale * gamepad2.left_stick_x;
        extendPosError = extendPosDes - extendPosCurrent;
        extendPow = Math.min(Math.max(extendPowScale*extendPosError, -1.00), 1.00);
        robot.motorExtend.setPower(extendPow);
    }
}
