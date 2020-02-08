package org.firstinspires.ftc.teamcode.GoBuilda.Development.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;

@TeleOp(name = "TeleTubbies Quals", group = "actual")
public class TeleopGoBuilda extends OpMode
{
    private double liftPosScale = 48, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;

    private double extendPosScale = 4.5, extendPowScale = 0.025;
    private double extendPosCurrent=0, extendPosDes=0, extendPosError=0, extendPow=0;


    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;

    GoBuildaUtil robot = new GoBuildaUtil();

    @Override
    public void init()
    {
        robot.initializeTele(hardwareMap);
    }

    @Override
    public void loop()
    {
       robot.Brake();

        if (turtle)
            multiplier = 0.45;
        else
            multiplier = 1;

        if (sloth)
            speedK = 0.5;
        else
            speedK = 1;


        // initializing wheel variables
        double powerXWheels = 0;
        double powerYWheels = 0;

        // Handle regular movement
        powerYWheels = -gamepad1.left_stick_y;

        // Handle sliding movement
        powerXWheels = -gamepad1.right_stick_x;

        robot.MotorBackX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);
        robot.MotorFrontX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);

        robot.MotorBackY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);
        robot.MotorFrontY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);

        if(gamepad1.b)
            turtle = !turtle;

        if(gamepad2.b)
            sloth = !sloth;

        if (gamepad1.a) {
            multiplier = 1;
        }
        if (gamepad2.a) {
            speedK = 1;
        }

        //Turning clockwise
        float rotationACW = gamepad1.right_trigger;
        float rotationCW = gamepad1.left_trigger;

        if(rotationCW !=0)
        {
            robot.MotorFrontX.setPower(gamepad1.left_trigger*multiplier);
            robot.MotorFrontY.setPower(-gamepad1.left_trigger*multiplier);
            robot.MotorBackX.setPower(-gamepad1.left_trigger*multiplier);
            robot.MotorBackY.setPower(gamepad1.left_trigger*multiplier);
        }

        if (rotationACW !=0)
        {
            robot.MotorFrontX.setPower(-gamepad1.right_trigger*multiplier);
            robot.MotorFrontY.setPower(gamepad1.right_trigger*multiplier);
            robot.MotorBackX.setPower(gamepad1.right_trigger*multiplier);
            robot.MotorBackY.setPower(-gamepad1.right_trigger*multiplier);
        }

        if (gamepad2.left_bumper)
        {
            robot.graspL.setPosition(1);
            robot.graspR.setPosition(0);
        }
        if (gamepad2.right_bumper)
        {
            robot.graspL.setPosition(0);
            robot.graspR.setPosition(1);
        }

        if(gamepad1.dpad_up && foundPos<=1) { foundPos += 0.05; }

        if(gamepad1.dpad_down && foundPos >= -1) {  foundPos -= 0.05; }

        if(gamepad2.dpad_up) { rotPos = rotPos - 0.05; }

        if(gamepad2.dpad_down) { rotPos = rotPos + 0.05; }

        if(gamepad2.dpad_left) { robot.block_drag_grasp.setPosition(1); }

        if(gamepad2.dpad_right) { robot.block_drag_grasp.setPosition(0); }

        if(gamepad2.x) { while(robot.motorVertical.getCurrentPosition()>60){robot.motorVertical.setPower(-1);}}

        robot.block_drag.setPosition(rotPos);
        robot.foundation.setPosition(foundPos);
        robot.foundation2.setPosition(1-foundPos);

        liftPosCurrent = robot.motorVertical.getCurrentPosition();

        liftPosDes += speedK*liftPosScale*gamepad2.left_stick_y;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//        integrater += liftPosError;                                           //unecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        robot.motorVertical.setPower(liftPow);


        extendPosCurrent = robot.motorExtend.getCurrentPosition();
        extendPosDes += extendPosScale * gamepad2.right_stick_x;
        extendPosError = extendPosDes - extendPosCurrent;
        extendPow = Math.min(Math.max(extendPowScale*extendPosError, -1.00), 1.00);
        if(extendPow >= 1) {liftPosDes = liftPosCurrent+(1/liftPosDes);}
        if(extendPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPosDes);}
        robot.motorExtend.setPower(extendPow);

    }
}
