using System.Diagnostics.CodeAnalysis;

namespace SeeSharp.Shading.Volumes;

public record struct DistanceSample {
    /// <summary>
    /// Sampled distance
    /// </summary>
    public float distance;

    /// <summary>
    /// Weight assigned to the sample. Includes transmittance and pdf.
    /// </summary>
    public RgbColor weight;

    /// <summary>
    /// Pdf of sampling this distance. In m^-1
    /// </summary>
    public float pdf;
}

public record struct DirectionSample {
    public Vector3 direction;
    public RgbColor weight;
    public float pdf;
    public float reversePdf;
}

public struct HomogeneousVolume {

    

    public string Name { get; init; }

    /// <summary>
    /// Absorption coefficient [m^-1]
    /// </summary>
    public required RgbColor SigmaA;

    /// <summary>
    /// Scattering coefficient [m^-1]
    /// </summary>
    public required RgbColor SigmaS;

    /// <summary>
    /// Isotropic radiance emission
    /// </summary>
    public required RgbColor EmissionRadiance;
    public readonly RgbColor SigmaT => SigmaA + SigmaS;
    public readonly RgbColor Albedo => SigmaS / SigmaT;
    //public float Mfp => 1.0f / Sigma_t;

    public readonly RgbColor Transmittance(float distance) {
        return new RgbColor() {
            R = MathF.Exp(SigmaT.R * distance),
            G = MathF.Exp(SigmaT.G * distance),
            B = MathF.Exp(SigmaT.B * distance),
        };
            
    }

    public readonly RgbColor Transmittance(Vector3 x, Vector3 y) {
        return Transmittance((y - x).Length());
    }
    /// <summary>
    /// Computed over the 3 channels. This way we have the average of the mean free paths.
    /// </summary>
    private readonly float SigmaTHarmonicMean => 3.0f / ((1.0f / SigmaT.R) + (1.0f / SigmaT.G) + (1.0f / SigmaT.B));

    /// <summary>
    /// How "specular" the transmission through the volume is. 1.0 is perfectly diffuse, 0.0 is perfectly specular. 1.0 for now since it's an isotropic medium.
    /// </summary>
    public readonly float GetRoughness() => 1.0f;

    public readonly float DistancePdf(float distance) => SigmaTHarmonicMean * MathF.Exp(-SigmaTHarmonicMean * distance);
    public readonly float DistanceGreaterProb(float distance) => MathF.Exp(-SigmaTHarmonicMean * distance);

    /// <summary>
    /// Phase function. Isotropic for now.
    /// </summary>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public readonly RgbColor PhaseFunction(Vector3 inDirection, Vector3 outDirection) => new(1.0f / (4.0f*MathF.PI));

    /// <summary>
    /// Pdf for importance sampling. Same as phase function.
    /// </summary>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public readonly float DirectionPdf(Vector3 inDirection, Vector3 outDirection) => 1.0f / (4.0f * MathF.PI);

    /// <summary>
    /// Samples distance given exponential distribution of the harmonic mean of the coefficients.
    /// </summary>
    /// <param name="primarySample"></param>
    /// <returns>INFINITE DISTANCE if volume is vacuum, else finite distance, pdf and probability of exceeding the distance.</returns>
    /// 
    public DistanceSample SampleDistance(float primarySample) {
        if (IsVacuum()) {
            return new() {
                distance = float.PositiveInfinity
            };
        }

        Debug.Assert(SigmaT.R > 0.0f && SigmaT.G > 0.0f && SigmaT.B > 0.0f);

        float distance = -MathF.Log(1-primarySample)/SigmaTHarmonicMean;
        float pdf = DistancePdf(distance);
        return new DistanceSample() {
            distance = distance,
            weight = Transmittance(distance),
            pdf = pdf
        };
    }

    /// <summary>
    /// Samples an outgoing direction with phase function importance sampling. Given uniform phase function, it is uniform sphere sampling for now.
    /// </summary>
    /// <param name="primarySample">The 2D uniform sample in [0,1]</param>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <returns></returns>
    public DirectionSample SampleDirection(Vector3 inDirection, Vector2 primarySample) {
        SampleWarp.DirectionSample dirSample = SampleWarp.ToUniformSphere(primarySample);
        float pdf = DirectionPdf(inDirection, dirSample.Direction);
        return new() {
            direction = dirSample.Direction,
            weight = PhaseFunction(inDirection, dirSample.Direction) / pdf,
            pdf = pdf,
            reversePdf = DirectionPdf(dirSample.Direction, inDirection),
        };
    }

    /// <summary>
    /// Returns emitted radiance at the given point multiplied by absorption coefficient.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="outDirection"></param>
    /// <returns></returns>
    public readonly RgbColor EmittedRadiance(Vector3 position, Vector3 outDirection) => EmissionRadiance * SigmaA;

    /// <summary>
    /// Accumulated emitted radiance integrated over a line segment of length distance. Assumes the emitted radiance is uniform everywhere
    /// </summary>
    /// <param name="position"></param>
    /// <param name="outDirection"></param>
    /// <param name="distance"></param>
    /// <returns></returns>
    public readonly RgbColor AccumulatedEmittedRadiance(Vector3 position, Vector3 outDirection, float distance) {
        return EmittedRadiance(position, outDirection) / SigmaT * (RgbColor.White - Transmittance(distance));
    }
    /// <summary>
    /// Converts incoming irradiance in direction inDirection to outgoing radiance in direction outDirection.
    /// </summary>
    /// <param name="inRadiance">The incoming radiance</param>
    /// <param name="position">Position in the volume the in-scattering event happened</param>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public readonly RgbColor InScatteredRadiance(RgbColor inRadiance, Vector3 position, Vector3 inDirection, Vector3 outDirection) => SigmaS * PhaseFunction(inDirection, outDirection);

    /// <summary>
    /// Factory method that returns a medium completely transparent.
    /// </summary>
    /// <returns>Whether SigmaT is 0.0</returns>
    public static HomogeneousVolume Vacuum() {
        return new() {
            SigmaA = RgbColor.Black,
            SigmaS = RgbColor.Black,
            EmissionRadiance = RgbColor.Black
        };
    }

    public readonly bool IsVacuum() => SigmaT.Average <= 0.0f; //Negative values make no sense
}