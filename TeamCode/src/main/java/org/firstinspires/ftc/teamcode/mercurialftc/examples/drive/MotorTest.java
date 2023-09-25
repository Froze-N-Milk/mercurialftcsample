package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

@TeleOp(name = "motor test")
public class MotorTest extends OpModeEX {
	private DcMotorEx fl, bl, br, fr;
	
	
	/**
	 * called before {@link #initEX()}, solely for initialising all subsystems, ensures that they are registered with the correct {@link Scheduler}, and that their init methods will be run
	 */
	@Override
	public void registerSubsystems() {
	
	}
	
	/**
	 * should contain your regular init code
	 */
	@Override
	public void initEX() {
		fl = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "fl"));
		fl.setDirection(DcMotorSimple.Direction.FORWARD); // set the required motors to reverse
		bl = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "bl"));
		bl.setDirection(DcMotorSimple.Direction.FORWARD);
		br = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "br"));
		br.setDirection(DcMotorSimple.Direction.FORWARD);
		fr = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "fr"));
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
	}
	
	/**
	 * registers triggers after the subsystem and regular init code,
	 * useful for organisation of your OpModeEX, but functionally no different to initialising them at the end of {@link #initEX()}
	 */
	@Override
	public void registerTriggers() {
	
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
	
	}
	
	@Override
	public void loopEX() {
		if (gamepadEX1().a().buttonState()) {
			fl.setPower(1.0);
		} else {
			fl.setPower(0.0);
		}
		if (gamepadEX1().b().buttonState()) {
			bl.setPower(1.0);
		} else {
			bl.setPower(0.0);
		}
		if (gamepadEX1().x().buttonState()) {
			br.setPower(1.0);
		} else {
			br.setPower(0.0);
		}
		if (gamepadEX1().y().buttonState()) {
			fr.setPower(1.0);
		} else {
			fr.setPower(0.0);
		}
	}
	
	@Override
	public void stopEX() {
	
	}
}
