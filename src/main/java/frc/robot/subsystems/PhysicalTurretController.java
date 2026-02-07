package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;

public class PhysicalTurretController {
	public PhysicalTurretController(Per<AngularVelocityUnit, VoltageUnit> kV, Per<TorqueUnit, CurrentUnit> kA,
			Resistance resistance, MomentOfInertia inertia, double gearRatio) {
		Measure<TorqueUnit> torque = Volts.one().div(resistance).timesRatio(kA).times(gearRatio);
		double acceleration = torque.in(NewtonMeters) / inertia.in(KilogramSquareMeters);
		Matrix<N2,N2> a = MatBuilder.fill(N2.instance, N2.instance, 0, 1, 0, );
		Matrix<N2,N1> b = MatBuilder.fill(N2.instance, N1.instance, 0, acceleration);
		Matrix<N1,N2> c = MatBuilder.fill(N1.instance, N2.instance, null);
		Matrix<N1,N1> d = MatBuilder.fill(N1.instance, N1.instance, null);
		LinearSystem<N2,N1,N1> system = new LinearSystem<>(a, null, null, null)
	}
}
