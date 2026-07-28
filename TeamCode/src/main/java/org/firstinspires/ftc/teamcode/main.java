package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainClass_Blue", group="Test")
public class MainClass_Blue extends LinearOpMode {

    private RobotHardware hardware;
    private DriveController driveController;
    private ShooterController shooterController;
    private ClimbController climbController;
    private IntakeController intakeController;
    private MotorLogger motorLogger;
    private ExtendController extendController;
    private FeedController feedController;
    private ImuController imuController;

    boolean prevUp_2 = false;
    boolean prevDown_2 = false;
    boolean prevRight = false;
    boolean prevLeft = false;
    boolean prevRight_2 = false;
    boolean prevLeft_2 = false;
    boolean prevLeftBump = false;

    double prevLeftT;

    boolean climbStopped = false;
    boolean isStart = false;
    boolean isShootMode = false;
    boolean wasShoot = false;
    int climbEnum = 0;
    boolean timerRunning = false;
    boolean isReverse = false;
    double shootSpeed = 800;
    int i = 0;
    int prevLoop = 2;//ボタン判定の緩さ加減


    @Override
    public void runOpMode() {

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
        imuController = new ImuController(hardware);
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "準備完了");
        telemetry.update();

        waitForStart();

        imuController.reset();

        while (opModeIsActive()) {
            double forwardRaw = driveController.applyDeadzone(-gamepad1.left_stick_y);
            double turnRaw = driveController.applyDeadzone(gamepad1.right_stick_x);

            driveController.driveNormal(forwardRaw, turnRaw);

            extendController.extend_loop();

            if (isStart) {
                double currentVelocity = Math.abs(intakeController.Intake.getVelocity());
                if (!isReverse && !isShootMode && currentVelocity < 200) {
                    if (!timerRunning) {
                        timer.reset();
                        timerRunning = true;
                    } else if (timer.milliseconds() > 1500) {
                        isReverse = true;
                        timerRunning = false;
                        timer.reset();
                    }
                } else if (!isReverse) {
                    timerRunning = false;
                }
                if (isReverse) {
                    if (timer.milliseconds() < 750) {
                        intakeController.Intake_Reverse();
                    } else {
                        isReverse = false;
                    }
                } else {

                    if (gamepad1.b) {
                        intakeController.Intake_Reverse();
                        extendController.extendUp();
                        feedController.Feed_turn_left();
                    } else if (!isShootMode) {
                        intakeController.Intake_Forward();
                        feedController.Feed_turn_right();
                    }
                }

                if (gamepad1.left_bumper && !prevLeftBump) {
                    extendController.extendDown();
                }

                if (gamepad1.right_trigger > 0.1 && !isShootMode) {
                    isShootMode = true;
                    imuController.setTarget(90);
                    shooterController.setSpeed(shootSpeed);
                }

                if (isShootMode) {
                    imuController.update();
                    shooterController.shooter_loop();

                    if(gamepad1.left_stick_button || gamepad1.right_stick_button){
                        isShootMode = false;
                    }

                    if (gamepad1.right_bumper) {
                        feedController.Feed_turn_left();
                        extendController.extendUp();
                        wasShoot = true;
                    } else if (!gamepad1.right_bumper && wasShoot) {
                        shooterController.setSpeed(0);
                        wasShoot = false;
                        isShootMode = false;
                    }
                }

                if (gamepad1.left_trigger > 0.05 && prevLeftT < 0.05) {
                    climbEnum += 1;
                    if (climbEnum >= 3) {
                        climbEnum = 0;
                    }
                }
                if(climbController.isUpperLimit()){
                    climbEnum = 2;
                }else if(climbController.isBottomLimit()){
                    climbEnum = 1;
                }
                switch (climbEnum) {
                    case 0:
                        climbController.climbStop();
                        break;
                    case 1:
                        climbController.climbUp();
                        break;
                    case 2:
                        climbController.climbDown();
                        break;
                }

                int cS = (int) (shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2;

                telemetry.addData("目標角度", extendController.targetAngle);
                telemetry.addData("現在角度", extendController.current);
                telemetry.addData("現在出力", extendController.output);
                telemetry.addData("現在速度", cS);
                telemetry.addData("詰まり検知タイマー", timerRunning ? timer.milliseconds() : 0);
                telemetry.addData("自動逆転中", isReverse);
                telemetry.addData("シューター対目標速度誤差",Math.abs((shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2 - shootSpeed));
                telemetry.addData("シューター速度左",shooterController.leftShooter.getVelocity());
                telemetry.addData("シューター速度右",shooterController.rightShooter.getVelocity());
                telemetry.update();

                motorLogger.update();

                i ++;
                if(i <= prevLoop){
                    i = 0;
                    prevDown_2 = gamepad2.dpad_down;
                    prevUp_2 = gamepad2.dpad_up;
                    prevLeft = gamepad1.dpad_left;
                    prevRight = gamepad1.dpad_right;
                    prevRight_2 = gamepad2.dpad_right;
                    prevLeft_2 = gamepad2.dpad_left;
                    prevLeftBump = gamepad1.left_bumper;
                    prevLeftT = gamepad1.left_trigger;

                }

            } else {
                telemetry.addData("展開出力",extendController.output);
                telemetry.update();
                if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                    isStart = true;
                }
            }
        }
        motorLogger.close();
    }
}