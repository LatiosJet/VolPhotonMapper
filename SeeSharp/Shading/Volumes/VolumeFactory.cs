using SimpleImageIO;

namespace SeeSharp.Shading.Volumes;

public class VolumeFactory {
    /// <summary>
    /// Factory method that returns a medium completely transparent.
    /// </summary>
    public static HomogeneousVolume Vacuum() => new() {
        SigmaA = RgbColor.Black,
        SigmaS = RgbColor.Black,
        EmissionRadiance = RgbColor.Black,
        Name = "Vacuum"
    };
    /// <summary>
    /// Returns a medium with the given albedo and such that every "halfDistance" meters visibility is halved
    /// </summary>
    /// <returns></returns>
    public static HomogeneousVolume ByAlbedoAndHalfDistance(RgbColor albedo, RgbColor halfDistance) {
        RgbColor sigmaT = new() {
            R = -MathF.Log(0.5f) / halfDistance.R,
            G = -MathF.Log(0.5f) / halfDistance.G,
            B = -MathF.Log(0.5f) / halfDistance.B,
        };
        RgbColor sigmaS = sigmaT * albedo;
        return new() {
            SigmaA = sigmaT - sigmaS,
            SigmaS = sigmaS,
            EmissionRadiance = RgbColor.Black
        };
    }

    /// <summary>
    /// Returns a thin smoke medium w/ albedo = 0.5 for all color channels and half-distance 10 meters
    /// </summary>
    /// <returns>Whether SigmaT is 0.0</returns>
    public static HomogeneousVolume ThinSmoke() => ByAlbedoAndHalfDistance(new RgbColor(0.5f), new RgbColor(10.0f));

    /// <summary>
    /// Returns a smoke medium w/ albedo = 0.5 for all color channels and half-distance 5 meters
    /// </summary>
    /// <returns>Whether SigmaT is 0.0</returns>
    public static HomogeneousVolume Smoke() => ByAlbedoAndHalfDistance(new RgbColor(0.5f), new RgbColor(5.0f));


    /// <summary>
    /// Returns a thick smoke medium w/ albedo = 0.5 for all color channels and half-distance 2 meters
    /// </summary>
    /// <returns>Whether SigmaT is 0.0</returns>
    public static HomogeneousVolume ThickSmoke() => ByAlbedoAndHalfDistance(new RgbColor(0.5f), new RgbColor(2.0f));

}