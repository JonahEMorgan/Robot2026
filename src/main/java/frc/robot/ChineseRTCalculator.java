package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ChineseRTCalculator {
	private final static double kTolerance = 0.001;
	private final static int[] kProposals = new int[] { 180, 90, 270, 45, 135, 225, 315 };

	public static double getAngle(double proposal, double angleA, double gearRatioA, double angleB, double gearRatioB) {
		double initial = proposal;
		for (int attempt = 0; attempt <= kProposals.length; attempt++) {
			double solution = getSolution(proposal, angleA, gearRatioA, angleB, gearRatioB);
			if (checkSolution(solution, angleA, gearRatioA, angleB, gearRatioB)) {
				return solution;
			} else if (attempt < kProposals.length) {
				proposal = kProposals[attempt];
			}
		}
		return initial;
	}

	private static double getSolution(double proposal, double angleA, double gearRatioA, double angleB,
			double gearRatioB) {
		double magic = proposal * gearRatioA % 360 - angleA - proposal * gearRatioB % 360 + angleB;
		return proposal + magic / (gearRatioB - gearRatioA);
	}

	private static boolean checkSolution(double solution, double angleA, double gearRatioA, double angleB,
			double gearRatioB) {
		if (solution > 360 || solution < 0) {
			return false;
		} else {
			return MathUtil.isNear(solution * gearRatioA % 360, angleA, kTolerance)
					&& MathUtil.isNear(solution * gearRatioB % 360, angleB, kTolerance);
		}
	}

	public static Command testCommand() {
		return Commands.runOnce(() -> {
			double gearRatioA = 5.3;
			double gearRatioB = 6.7;
			int tests = 1000;
			int correct = 0;
			for (int test = 0; test < tests; test++) {
				double angle = Math.random() * 360;
				double angleA = angle * gearRatioA % 360;
				double angleB = angle * gearRatioB % 360;
				double calculated = getAngle(0, angleA, gearRatioA, angleB, gearRatioB);
				if (MathUtil.isNear(calculated, angle, kTolerance)) {
					correct++;
				}
			}
			System.out.printf("%d/%d tests passing for chinese remainder theorem\n", correct, tests);
		});
	}
}
