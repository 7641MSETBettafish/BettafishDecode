package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="IntakeTest", group="Testing")
public class IntakeTest extends LinearOpMode {

    public static double motorPower = 0;

    DcMotor intakeMotor;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeShooter");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            intakeMotor.setPower(motorPower);

        }

    }
}
