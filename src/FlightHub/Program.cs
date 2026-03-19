using System.Text;

// Top-level program: run a short deterministic 6DOF Euler integration and print CSV
using FlightHub;

var engine = new SimulationEngine();
var state = new AircraftState
{
    U = 50.0,
    V = 0.0,
    W = 0.0,
    P = 0.0,
    Q = 0.0,
    R = 0.0,
    Roll = 0.0,
    Pitch = 0.0,
    Yaw = 0.0,
    X = 0.0,
    Y = 0.0,
    Z = 0.0
};

int steps = 10;
double dt = 0.1;
var sb = new StringBuilder();
sb.AppendLine("time,U,V,W,P,Q,R,Roll,Pitch,Yaw,X,Y,Z");
for (int i = 0; i < steps; i++)
{
    sb.AppendLine(engine.FormatState(i * dt, state));
    engine.Step(state, dt);
}

Console.WriteLine(sb.ToString());
