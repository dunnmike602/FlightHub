using Xunit;
using FlightHub;

public class SimulationTests
{
    [Fact]
    public void Step_IntegratesPosition_WhenBodyVelocityForward()
    {
        var engine = new SimulationEngine();
        var s = new AircraftState{ U = 10.0, V = 0, W = 0, Roll=0, Pitch=0, Yaw=0 };
        engine.Step(s, 1.0);
        Assert.Equal(10.0, s.X, 6);
        Assert.Equal(0.0, s.Y, 6);
        Assert.Equal(0.0, s.Z, 6);
    }

    [Fact]
    public void Attitude_Changes_WithBodyRates()
    {
        var engine = new SimulationEngine();
        var s = new AircraftState{ P = 0.1, Q = 0.2, R = 0.3, Roll=0.01, Pitch=0.02, Yaw=0.03 };
        engine.Step(s, 0.5);
        // simple assert that values changed
        Assert.NotEqual(0.01, s.Roll);
        Assert.NotEqual(0.02, s.Pitch);
        Assert.NotEqual(0.03, s.Yaw);
    }
}
