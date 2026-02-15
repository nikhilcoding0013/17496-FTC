package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoAnglingTELEOP", group = "TeleOp")
public class ServoAnglingTELEOP extends OpMode {

    Servo leftServo;
    Servo rightServo;

    // Base movement scaling factor
    final double SPEED_SCALE = 0.02;

    // Position limits
    final double MIN_POS = 0.0;
    final double MAX_POS = 0.7;

    // Current positions
    double leftPos;
    double rightPos;

    @Override
    public void init() {

        leftServo  = hardwareMap.get(Servo.class, "servo0");
        rightServo = hardwareMap.get(Servo.class, "servo1");

        rightServo.setDirection(Servo.Direction.REVERSE);

        // Reset to zero at start
        leftPos = 0.0;
        rightPos = 0.0;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);
    }

    @Override
    public void loop() {

        // Read trigger values (0.0 to 1.0)
        double inSpeed  = gamepad1.right_trigger;  // move IN
        double outSpeed = gamepad1.left_trigger;   // move OUT

        // Net movement (right trigger positive, left negative)
        double movement = (inSpeed - outSpeed) * SPEED_SCALE;

        // Update positions
        leftPos  = Math.min(Math.max(leftPos + movement, MIN_POS), MAX_POS);
        rightPos = Math.min(Math.max(rightPos + movement, MIN_POS), MAX_POS);

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        telemetry.addData("Left Position", "%.2f", leftPos);
        telemetry.addData("Right Position", "%.2f", rightPos);
        telemetry.addData("Angle", "%.2f",  65 - leftPos * 28.6);
        telemetry.addData("Right Trigger (IN)", inSpeed);
        telemetry.addData("Left Trigger (OUT)", outSpeed);
        telemetry.update();
    }
}
