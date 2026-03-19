namespace FlightHub;

public sealed class AircraftState
{
    // Linear velocity (body frame)
    public double U { get; set; }
    public double V { get; set; }
    public double W { get; set; }

    // Angular rates (body frame)
    public double P { get; set; }
    public double Q { get; set; }
    public double R { get; set; }

    // Attitude (Euler angles)
    public double Roll { get; set; }   // phi (rad)
    public double Pitch { get; set; }  // theta (rad)
    public double Yaw { get; set; }    // psi (rad)

    // Position (inertial frame)
    public double X { get; set; } // m
    public double Y { get; set; } // m
    public double Z { get; set; } // m
}

public interface ISimulationEngine
{
    void Step(AircraftState state, double dt);
    string FormatState(double time, AircraftState state);
}

public sealed class SimulationEngine : ISimulationEngine
{
    // Very small deterministic Euler integrator using kinematic transforms.
    public void Step(AircraftState state, double dt)
    {
        // For the strict scope: no aerodynamics. Use body velocities as inputs.
        // Integrate linear velocities -> position (transform body->inertial using small-angle approximation)

        // Rotation matrix from body to inertial (N) using Euler angles (phi, theta, psi)
        double phi = state.Roll;
        double theta = state.Pitch;
        double psi = state.Yaw;

        double cphi = Math.Cos(phi);
        double sphi = Math.Sin(phi);
        double ctheta = Math.Cos(theta);
        double stheta = Math.Sin(theta);
        double cpsi = Math.Cos(psi);
        double spsi = Math.Sin(psi);

        // Body-to-inertial rotation R = Rz(psi) * Ry(theta) * Rx(phi)
        double r11 = ctheta * cpsi;
        double r12 = ctheta * spsi;
        double r13 = -stheta;

        double r21 = sphi * stheta * cpsi - cphi * spsi;
        double r22 = sphi * stheta * spsi + cphi * cpsi;
        double r23 = sphi * ctheta;

        double r31 = cphi * stheta * cpsi + sphi * spsi;
        double r32 = cphi * stheta * spsi - sphi * cpsi;
        double r33 = cphi * ctheta;

        // Linear velocity in inertial frame
        double Vx = r11 * state.U + r12 * state.V + r13 * state.W;
        double Vy = r21 * state.U + r22 * state.V + r23 * state.W;
        double Vz = r31 * state.U + r32 * state.V + r33 * state.W;

        // Integrate position
        state.X += Vx * dt;
        state.Y += Vy * dt;
        state.Z += Vz * dt;

        // Integrate attitudes using body rates (P,Q,R) -> Euler angle rates approximation
        // phi_dot = P + Q*sin(phi)*tan(theta) + R*cos(phi)*tan(theta)
        // theta_dot = Q*cos(phi) - R*sin(phi)
        // psi_dot = Q*sin(phi)/cos(theta) + R*cos(phi)/cos(theta)

        double tanTheta = Math.Tan(theta);
        double cosTheta = Math.Cos(theta);
        if (Math.Abs(cosTheta) < 1e-6) cosTheta = 1e-6; // avoid div by zero

        double phi_dot = state.P + state.Q * Math.Sin(phi) * tanTheta + state.R * Math.Cos(phi) * tanTheta;
        double theta_dot = state.Q * Math.Cos(phi) - state.R * Math.Sin(phi);
        double psi_dot = (state.Q * Math.Sin(phi) + state.R * Math.Cos(phi)) / cosTheta;

        state.Roll += phi_dot * dt;
        state.Pitch += theta_dot * dt;
        state.Yaw += psi_dot * dt;

        // For this scope, linear and angular velocities are constant (no dynamics model).
    }

    public string FormatState(double time, AircraftState state)
    {
        return string.Join(",", new object[] {
            time.ToString("F3"),
            state.U.ToString("F6"),
            state.V.ToString("F6"),
            state.W.ToString("F6"),
            state.P.ToString("F6"),
            state.Q.ToString("F6"),
            state.R.ToString("F6"),
            state.Roll.ToString("F6"),
            state.Pitch.ToString("F6"),
            state.Yaw.ToString("F6"),
            state.X.ToString("F6"),
            state.Y.ToString("F6"),
            state.Z.ToString("F6")
        });
    }
}
