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
    /// <summary>
    /// The sampled direction. Normalized
    /// </summary>
    public Vector3 direction;

    /// <summary>
    /// The product of scattering coefficient and phase function divided by the pdf
    /// </summary>
    public RgbColor weight;

    /// <summary>
    /// Pdf of sampling the given direction
    /// </summary>
    public float pdf;

    /// <summary>
    /// Pdf where inDirection and outDirection are swapped. For volumes the two pdfs are always the same as phase functions must be symmetric.
    /// </summary>
    public readonly float ReversePdf => pdf;
}

public class HomogeneousVolume {
    public string Name { get; init; }

    /// <summary>
    /// Absorption coefficient [m^-1]
    /// </summary>
    public required RgbColor SigmaA { get; init; }

    /// <summary>
    /// Scattering coefficient [m^-1]
    /// </summary>
    public required RgbColor SigmaS { get; init; }

    /// <summary>
    /// Isotropic radiance emission
    /// </summary>
    public RgbColor EmissionRadiance = RgbColor.Black;

    /// <summary>
    /// g-value for the Henyey-Greenstein phase function. g in [-1, 1], where 0 is isotropic, 1 is fully forward scattering and -1 is fully backward scattering.
    /// TODO: actually implement the Henyey-Greenstein
    /// </summary>
    public float G = 0.0f;
    public RgbColor SigmaT => SigmaA + SigmaS;
    public RgbColor Albedo => SigmaS / SigmaT;
    //public float Mfp => 1.0f / SigmaT;

    public RgbColor Transmittance(float distance) {
        return new RgbColor() {
            R = MathF.Exp(-SigmaT.R * distance),
            G = MathF.Exp(-SigmaT.G * distance),
            B = MathF.Exp(-SigmaT.B * distance),
        };
    }

    public RgbColor Transmittance(Vector3 x, Vector3 y) {
        return Transmittance((y - x).Length());
    }
    /// <summary>
    /// Computed over the 3 channels. This way we have the average of the mean free paths.
    /// </summary>
    private float SigmaTHarmonicMean => 3.0f / ((1.0f / SigmaT.R) + (1.0f / SigmaT.G) + (1.0f / SigmaT.B));

    /// <summary>
    /// How "specular" the transmission through the volume is. 1.0 is perfectly diffuse, 0.0 is perfectly specular. 1.0 for now since it's an isotropic medium.
    /// </summary>
    public float GetRoughness() => 1.0f;

    public float DistancePdf(float distance) => SigmaTHarmonicMean * MathF.Exp(-SigmaTHarmonicMean * distance);
    public float DistanceGreaterProb(float distance) => MathF.Exp(-SigmaTHarmonicMean * distance);

    float PdfUniform = 1 / (4 * MathF.PI);
    public RgbColor PhaseFunctionUniform => new RgbColor(PdfUniform);

    /// <summary>
    /// Phase function
    /// </summary>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public RgbColor PhaseFunction(Vector3 inDirection, Vector3 outDirection) {
        if (G < 1e-4f)
            return PhaseFunctionUniform;

        float g_sqr = G * G;
        float cosAngle = Vector3.Dot(inDirection, outDirection);
        float aux = 1 + g_sqr + 2 * G * cosAngle;
        float value = (1 - g_sqr) / (4 * MathF.PI * aux * MathF.Sqrt(aux));
        return new RgbColor(value);
    }

    /// <summary>
    /// Pdf for importance sampling. Same as phase function.
    /// </summary>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public float DirectionPdf(Vector3 inDirection, Vector3 outDirection) => PhaseFunction(inDirection, outDirection).Average;

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
            weight = Transmittance(distance) / pdf,
            pdf = pdf
        };
    }
    public DirectionSample SampleDirectionUniform(Vector2 primarySample) {
        SampleWarp.DirectionSample dirSample = SampleWarp.ToUniformSphere(primarySample);
        return new() {
            direction = dirSample.Direction,
            weight = SigmaS * PhaseFunctionUniform / PdfUniform,
            pdf = PdfUniform
        };
    }

    /// <summary>
    /// Samples an outgoing direction with phase function importance sampling. Given uniform phase function, it is uniform sphere sampling for now.
    /// </summary>
    /// <param name="primarySample">The 2D uniform sample in [0,1]</param>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <returns></returns>
    public DirectionSample SampleDirection(Vector3 inDirection, Vector2 primarySample) {
        if (G < 1e-5) {
            return SampleDirectionUniform(primarySample);
        }

        float g_sqr = G * G;
        float aux = MathF.Abs(G) > 1 - 1e-5f ? 0.0f : (1 - g_sqr) / (1 + G - 2 * G * primarySample.X);
        float cosTheta = - (1 + g_sqr - aux*aux) / (2 * G); //The minus sign is because in case of forward scattering, the sample must point in the opposite direction since both inDirection and outDirection point away from the point in the volume.
        cosTheta = float.Clamp(cosTheta, -1, 1); //Needed when primarySample.X approaches 0 or 1
        float sinTheta = MathF.Sqrt(1 - cosTheta * cosTheta);
        float phi = 2 * MathF.PI * primarySample.Y;

        Vector3 outDirectionLocal = SampleWarp.SphericalToCartesian(sinTheta, cosTheta, phi);
        SanityChecks.IsNormalized(outDirectionLocal);
        Vector3 outDirection = ShadingToWorld(inDirection, outDirectionLocal);
        float pdf = DirectionPdf(inDirection, outDirection);
        return new() {
            direction = outDirection,
            weight = SigmaS * PhaseFunction(inDirection, outDirection) / pdf,
            pdf = pdf
        };
    }

    /// <summary>
    /// Returns emitted radiance at the given point multiplied by absorption coefficient.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="outDirection"></param>
    /// <returns></returns>
    public RgbColor EmittedRadiance(Vector3 position, Vector3 outDirection) => EmissionRadiance * SigmaA;

    /// <summary>
    /// Converts incoming irradiance in direction inDirection to outgoing radiance in direction outDirection.
    /// Returns inRadiance * phaseFunction * sigmaS
    /// </summary>
    /// <param name="inRadiance">The incoming radiance</param>
    /// <param name="position">Position in the volume the in-scattering event happened</param>
    /// <param name="inDirection">The incoming direction (vector leaves the point)</param>
    /// <param name="outDirection">The outgoing direction (vector leaves the point)</param>
    /// <returns></returns>
    public RgbColor InScatteredRadiance(RgbColor inRadiance, Vector3 position, Vector3 inDirection, Vector3 outDirection) => inRadiance * PhaseFunction(inDirection, outDirection) * SigmaS;

    public bool IsVacuum() => SigmaT.Average <= 0.0f; //Negative values make no sense

    public override string ToString() {
        return "HomogeneousVolume " + Name;
    }
}
