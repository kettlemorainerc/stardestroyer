package org.usfirst.frc.team2077.sensors;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class AngleSensor {

	// these fields are private to the timer thread
	private final AHRS navX_;
	private boolean initializing_ = true;
	private boolean rotating_ = false;
	private double stopAngle_ = 0;
	private long n_ = 0;

	private AtomicReference<Double> angleREF_ = new AtomicReference<>(0.);

	public AngleSensor() {
		
		navX_ = new AHRS(SPI.Port.kMXP, (byte)200);
		System.out.println("NavX:" + navX_);
		System.out.println("Connected:" + navX_.isConnected());
		System.out.println("Calibrating:" + navX_.isCalibrating());

		(new Timer()).schedule(new TimerTask() {
			@Override
			public void run() {
				if (navX_.isConnected() && !navX_.isCalibrating()) {
					if (initializing_) {
						navX_.zeroYaw();
						initializing_ = false;
					}
					double angle = navX_.getAngle();
					boolean rotating = navX_.isRotating();
					//if ((n_++ % 100) == 0) System.out.println("Angle:" + angle + " R:" + rotating);
					if (!rotating) {
						if (rotating_) {
							stopAngle_ = angle;
						}
						navX_.setAngleAdjustment(navX_.getAngleAdjustment() - (angle - stopAngle_));
						angle = stopAngle_;
						rotating_ = false;
					}
					rotating_ = rotating;
					angleREF_.set(angle);
				}
			}
		}, 200, 18);
	}

	public double getAngle() {
		return angleREF_.get();
	}
}

