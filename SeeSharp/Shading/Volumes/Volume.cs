using System.Diagnostics.CodeAnalysis;

namespace SeeSharp.Shading.Volumes;

public class HomogeneousVolume {
    public string Name { get; init; }

    public required float Sigma_a;
    public required float Sigma_s;
    public float Sigma_t => Sigma_a + Sigma_s;
    public float Albedo => Sigma_s / Sigma_t;
    //public float Mfp => 1.0f / Sigma_t;

    public float Transmittance(Vector3 x, Vector3 y) {
        return MathF.Exp(Sigma_t * (x - y).Length());
    }
}