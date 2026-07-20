package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MainClass", group="Test")
public class MainClass extends LinearOpMode {

    private RobotHardware hardware;
    private DriveController driveController;
    private ShooterController shooterController;
    private ClimbController climbController;
    private IntakeController intakeController;
    private MotorLogger motorLogger;
    private ExtendController extendController;
    private FeedController feedController;

    boolean prevA = false;
    boolean prevB = false;
    boolean prevUp = false;
    boolean prevDown = false;
    boolean prevUp_2 = false;
    boolean prevDown_2 = false;
    boolean prevRight = false;
    boolean prevLeft = false;
    boolean prevRight_2 = false;
    boolean prevLeft_2 = false;
    boolean prevGuide = false;
    boolean climbStopped = false;
    boolean isExtend = false;

    double shootSpeed = 800;

    @Override
    public void runOpMode() {

        // ===== 初期化 =====
        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        motorLogger = new MotorLogger(hardware, hardwareMap);
        motorLogger.init();

        driveController = new DriveController(hardware);
        shooterController = new ShooterController(hardware);
        climbController = new ClimbController(hardware);
        intakeController = new IntakeController(hardware);
        feedController = new FeedController(hardware);
        extendController = new ExtendController(hardware);

        telemetry.addData("Status", "準備完了");
        telemetry.update();

        waitForStart();

        // ===== メインループ =====
        while (opModeIsActive()) {
            double forwardRaw = driveController.applyDeadzone(-gamepad1.left_stick_y);
            double turnRaw    = driveController.applyDeadzone(gamepad1.right_stick_x);

            driveController.driveNormal(forwardRaw, turnRaw);

            extendController.extend_loop();
            shooterController.shooter_loop();

            if(gamepad1.right_trigger > 0.05){
                intakeController.Intake.setPower(-1.0);
                feedController.feedSet(1.0);
                shooterController.setSpeed(0.0);
                extendController.extendDown();
            } else if(gamepad1.right_bumper){
                shooterController.setSpeed(shootSpeed);
                feedController.feedSet(0);
                intakeController.Intake.setPower(0);
                extendController.extendUp();
                if(Math.abs(shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2 - shootSpeed <= 30){
                    feedController.feedSet(-1.0);
                    intakeController.Intake.setPower(-1.0);
                    extendController.extendUp();
                }else{
                    feedController.feedSet(0);
                    intakeController.Intake.setPower(0);
                    extendController.extendDown();
                }
            } else{
                feedController.feedSet(0);
                intakeController.Intake.setPower(0.0);
                shooterController.setSpeed(0.0);
                extendController.extendDown();
            }

            if(gamepad1.guide){
                if(isExtend){
                    extendController.extendUp();
                    isExtend = false;
                }else{
                    extendController.extendDown();
                }
            }

            if (gamepad1.left_trigger > 0.05) {
                climbController.climbUp();
                climbStopped = false;
            } else if (gamepad1.left_bumper) {
                climbController.climbDown();
                climbStopped = false;
            } else if (!climbStopped) {
                climbController.climbStop();
                climbStopped = true;
            }

            if(gamepad2.dpad_right && !prevRight_2){
                shooterController.speedUp();
            }
            if(gamepad2.dpad_left && !prevLeft_2){
                shooterController.speedDown();
            }

            int tS = (int)shooterController.targetSpeed * 60 / 28;
            int cS = (int)(shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2;
            //cS = cS / 2 * 60 / 28;

            telemetry.addData("目標角度",extendController.targetAngle);
            telemetry.addData("現在角度",extendController.current);
            telemetry.addData("現在出力",extendController.output);
            telemetry.addData("現在速度",cS);
            telemetry.update();

            motorLogger.update();

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevDown = gamepad1.dpad_down;
            prevUp = gamepad1.dpad_up;
            prevDown_2 = gamepad2.dpad_down;
            prevUp_2 = gamepad2.dpad_up;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevRight_2 = gamepad2.dpad_right;
            prevLeft_2 = gamepad2.dpad_left;
            prevGuide = gamepad1.guide;
        }
        motorLogger.close();
    }
}