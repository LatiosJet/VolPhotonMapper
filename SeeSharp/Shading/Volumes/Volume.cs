using System.Diagnostics.CodeAnalysis;

namespace SeeSharp.Shading.Volumes;

public record struct DistanceSample {
    public float distance;
    public float pdf;
}

public struct HomogeneousVolume {

    

    public string Name { get; init; }

    public required RgbColor SigmaA;
    public required RgbColor SigmaS;
    public readonly RgbColor SigmaT => SigmaA + SigmaS;
    public readonly RgbColor Albedo => SigmaS / SigmaT;
    //public float Mfp => 1.0f / Sigma_t;

    public readonly RgbColor Transmittance(Vector3 x, Vector3 y) {
        return new RgbColor() {
            R = MathF.Exp(SigmaT.R * (x - y).Length()),
            G = MathF.Exp(SigmaT.G * (x - y).Length()),
            B = MathF.Exp(SigmaT.B * (x - y).Length()),
        };
            
    }
    /// <summary>
    /// Computed over the 3 channels. This way we have the average of the mean free paths.
    /// </summary>
    private readonly float SigmaTHarmonicMean => 3.0f / ((1.0f / SigmaT.R) + (1.0f / SigmaT.G) + (1.0f / SigmaT.B));

    public readonly float Pdf(float distance) => SigmaTHarmonicMean * MathF.Exp(SigmaTHarmonicMean * distance);

    public DistanceSample SampleDistance(float primarySample) {
        float distance = -MathF.Log(1-primarySample)/SigmaTHarmonicMean;
        float pdf = Pdf(distance);
        return new DistanceSample() {
            t = distance,
            pdf = pdf
        };
    }
}