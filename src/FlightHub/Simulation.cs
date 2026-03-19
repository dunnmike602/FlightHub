namespace FlightHub;

public interface ISimulationEngine
{
    string FormatState(double time, AircraftState state);

    void Step(AircraftState state, double dt);
}

public sealed class AircraftState
{
    // Angular rates (body frame)
    public double P { get; set; }

    public double Pitch { get; set; }

    public double Q { get; set; }

    public double R { get; set; }

    // Attitude (Euler angles)
    public double Roll { get; set; }

    // Linear velocity (body frame)
    public double U { get; set; }

    public double V { get; set; }
    public double W { get; set; }

    // Position (inertial frame)
    public double X { get; set; }

    // m
    public double Y { get; set; }

    // Mass of the aircraft (kg). Used when computing forces/accelerations.
    // Default value chosen so examples have reasonable scale; can be set by caller.
    public double Mass { get; set; } = 1000.0;

    // phi (rad)
    // theta (rad)
    public double Yaw { get; set; }    // psi (rad)

    // m
    public double Z { get; set; } // m
}

public sealed class SimulationEngine : ISimulationEngine
{
    // When true, gravity is applied to the aircraft as a body-frame
    // acceleration based on current attitude and Mass. Default is false
    // to preserve previous kinematic-only behavior.
    public bool ApplyGravity { get; set; } = false;

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

    // Very small deterministic Euler integrator using kinematic transforms.
    public void Step(AircraftState state, double dt)
    {
        // Note: This method performs a kinematic integration only. There is no
        // dynamics model (forces/moments) applied — body velocities and body
        // rates are treated as constant inputs over the timestep `dt`.

        // Extract current Euler angles from the state for readability
        // phi (roll), theta (pitch), psi (yaw)
        double phi = state.Roll;   // roll angle (rad)
        double theta = state.Pitch; // pitch angle (rad)
        double psi = state.Yaw;    // yaw angle (rad)

        // Precompute sines and cosines used in the rotation matrix and
        // Euler-rate equations to avoid repeated calls to Math.Sin/Math.Cos.
        double cphi = Math.Cos(phi); // cos(phi)
        double sphi = Math.Sin(phi); // sin(phi)
        double ctheta = Math.Cos(theta); // cos(theta)
        double stheta = Math.Sin(theta); // sin(theta)
        double cpsi = Math.Cos(psi); // cos(psi)
        double spsi = Math.Sin(psi); // sin(psi)

        // Construct the body-to-inertial rotation matrix R = Rz(psi) * Ry(theta) * Rx(phi)
        // This maps body-frame vectors into the inertial (navigation) frame.
        // The matrix elements are named r[row][col].
        double r11 = ctheta * cpsi;            // N_x = ctheta*cpsi
        double r12 = ctheta * spsi;            // N_y = ctheta*spsi
        double r13 = -stheta;                  // N_z = -sin(theta)

        double r21 = sphi * stheta * cpsi - cphi * spsi; // second row, first col
        double r22 = sphi * stheta * spsi + cphi * cpsi; // second row, second col
        double r23 = sphi * ctheta;                      // second row, third col

        double r31 = cphi * stheta * cpsi + sphi * spsi; // third row, first col
        double r32 = cphi * stheta * spsi - sphi * cpsi; // third row, second col
        double r33 = cphi * ctheta;                      // third row, third col

        // Transform body-frame linear velocity (U,V,W) into inertial-frame
        // linear velocity components (Vx,Vy,Vz) using the rotation matrix.
        double Vx = r11 * state.U + r12 * state.V + r13 * state.W;
        double Vy = r21 * state.U + r22 * state.V + r23 * state.W;
        double Vz = r31 * state.U + r32 * state.V + r33 * state.W;

        // Apply gravity force as an acceleration. Gravity acts in the
        // inertial frame along the negative Z axis (down). We convert the
        // gravity vector into the body frame (using R^T) to compute the
        // equivalent body-frame accelerations and then integrate them into
        // the body velocities (U,V,W). This is a very small and simple
        // model: no aerodynamic forces, only gravity influences linear
        // accelerations. For realistic dynamics, forces and moments from
        // aerodynamics and propulsion would be included.
        const double g = 9.80665; // m/s^2 (standard gravity)

        // If Mass is positive, compute acceleration due to gravity in body frame.
        if (ApplyGravity && state.Mass > 0)
        {
            // Gravity in inertial frame: (0, 0, -g). To get body-frame
            // components, multiply by R^T (which is the inverse of R for
            // rotation matrices). R^T element [i,j] = rji.
            double g_body_x = r11 * 0 + r21 * 0 + r31 * (-g); // = r31 * -g
            double g_body_y = r12 * 0 + r22 * 0 + r32 * (-g); // = r32 * -g
            double g_body_z = r13 * 0 + r23 * 0 + r33 * (-g); // = r33 * -g

            // Convert to accelerations (F = m*a => a = F/m). Here the
            // 'force' is simply mass*gravity in inertial frame projected to
            // body frame, so dividing by mass recovers the acceleration.
            double ax = g_body_x; // already acceleration (m/s^2) since we used g
            double ay = g_body_y;
            double az = g_body_z;

            // Integrate body-frame linear velocities using those accelerations.
            // v_{k+1} = v_k + a * dt
            state.U += ax * dt;
            state.V += ay * dt;
            state.W += az * dt;

            // Recompute inertial velocities after updating body velocities
            Vx = r11 * state.U + r12 * state.V + r13 * state.W;
            Vy = r21 * state.U + r22 * state.V + r23 * state.W;
            Vz = r31 * state.U + r32 * state.V + r33 * state.W;
        }

        // Integrate position using a simple forward Euler step:
        // x_{k+1} = x_k + Vx * dt, etc.
        state.X += Vx * dt;
        state.Y += Vy * dt;
        state.Z += Vz * dt;

        // Integrate attitudes (Euler angles) from body angular rates (P,Q,R).
        // The mapping from body rates to Euler angle rates is nonlinear; for
        // small angles and standard aerospace 3-2-1 (phi,theta,psi) ordering
        // the relations are:
        //   phi_dot   = P + Q*sin(phi)*tan(theta) + R*cos(phi)*tan(theta)
        //   theta_dot = Q*cos(phi) - R*sin(phi)
        //   psi_dot   = (Q*sin(phi) + R*cos(phi)) / cos(theta)
        // We compute these terms below. Note cos(theta) appears in denominators
        // so guard against singularities at theta ~= +/- 90 degrees.

        double tanTheta = Math.Tan(theta);      // tan(theta)
        double cosTheta = Math.Cos(theta);     // cos(theta)
        // Prevent division by zero when pitch is near +/- 90 degrees by
        // clamping cosTheta to a small non-zero value. This keeps the
        // integrator well-behaved for the limited scope of this example.
        if (Math.Abs(cosTheta) < 1e-6) cosTheta = 1e-6;

        // Compute Euler angle derivatives from body rates stored in state.
        double phi_dot = state.P + state.Q * Math.Sin(phi) * tanTheta + state.R * Math.Cos(phi) * tanTheta;
        double theta_dot = state.Q * Math.Cos(phi) - state.R * Math.Sin(phi);
        double psi_dot = (state.Q * Math.Sin(phi) + state.R * Math.Cos(phi)) / cosTheta;

        // Integrate Euler angles using forward Euler: angle_{k+1} = angle_k + angle_dot * dt
        state.Roll += phi_dot * dt;
        state.Pitch += theta_dot * dt;
        state.Yaw += psi_dot * dt;

        // Note: Linear and angular velocities (U,V,W,P,Q,R) are not updated
        // by this integrator — a full dynamics model would compute their
        // time derivatives from forces/moments. This method intentionally
        // leaves them constant over the timestep for simplicity.
    }
}