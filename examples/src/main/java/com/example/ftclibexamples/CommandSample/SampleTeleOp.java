package com.example.ftclibexamples.CommandSample;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sample TeleOp")
public class SampleTeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 100.0; // centimeters
    static final double DRIVE_CPR = 383.6;

    private MotorEx m_left, m_right;
    private DriveSubsystem m_drive;
    private GamepadEx m_driverOp;
    private DefaultDrive m_driveCommand;
    private GripperSubsystem m_gripper;
    private GrabStone m_grabCommand;
    private ReleaseStone m_releaseCommand;
    private GamepadButton m_grabButton;
    private GamepadButton m_releaseButton;

    @Override
    public void runOpMode() throws InterruptedException {
        m_left = new MotorEx(hardwareMap, "drive_left");
        m_right = new MotorEx(hardwareMap, "drive_right");
        m_drive = new DriveSubsystem(m_left, m_right, WHEEL_DIAMETER);

        m_driverOp = new GamepadEx(gamepad1);
        m_driveCommand = new DefaultDrive(m_drive, ()->m_driverOp.getLeftY(), ()->m_driverOp.getLeftX());

        m_grabButton = new GamepadButton(m_driverOp, GamepadKeys.Button.A);
        m_releaseButton = new GamepadButton(m_driverOp, GamepadKeys.Button.B);

        m_gripper = new GripperSubsystem(hardwareMap, "gripper");
        m_grabCommand = new GrabStone(m_gripper);
        m_releaseCommand = new ReleaseStone(m_gripper);

        m_drive.setDefaultCommand(m_driveCommand);
        m_grabButton.whenPressed(m_grabCommand);
        m_releaseButton.whenPressed(m_releaseCommand);
        register(m_drive, m_gripper);

        // run the scheduler
        while (!isStopRequested()) {
            run();
        }

        reset();
    }

}