using SeeSharp.Shading.Volumes;
using System.ComponentModel.DataAnnotations;
using System.Data;

namespace SeeSharp.Integrators.Common;

/// <summary>
/// Performs a random walk, invoking virtual callbacks for events along the path. The state of the walk is
/// tracked in this object, so it can only be used for one walk at a time.
/// </summary>
public ref struct VolRandomWalk<PayloadType> where PayloadType : new(){
    public record struct DirectionSample(
        float PdfForward,
        float PdfReverse,
        RgbColor Weight,
        Vector3 Direction,
        RgbColor ApproxReflectance
    ) {}

    public abstract class RandomWalkModifier {
        public virtual RgbColor OnInvalidHit(ref VolRandomWalk<PayloadType> walk, Ray ray, float pdfFromAncestor,
                                             RgbColor prefixWeight, int depth, int surfaceDepth)
        => RgbColor.Black;

        public virtual RgbColor OnHit(ref VolRandomWalk<PayloadType> walk, in SurfaceShader shader, float pdfFromAncestor,
                                      RgbColor prefixWeight, int depth, int surfaceDepth,  float toAncestorJacobian)
        => RgbColor.Black;

        public virtual RgbColor OnVolumeHit(ref VolRandomWalk<PayloadType> walk, Vector3 position, HomogeneousVolume volume, float pdfFromAncestor,
                                    RgbColor throughput, int depth, int surfaceDepth, float toAncestorJacobian)
        => RgbColor.Black;

        public virtual void OnContinue(ref VolRandomWalk<PayloadType> walk, float pdfToAncestor, int depth, int surfaceDepth) {}

        public virtual void OnTerminate(ref VolRandomWalk<PayloadType> walk) {}

        public virtual void OnStartCamera(ref VolRandomWalk<PayloadType> walk, CameraRaySample cameraRay, Pixel filmPosition) {}
        public virtual void OnStartEmitter(ref VolRandomWalk<PayloadType> walk, EmitterSample emitterSample, RgbColor initialWeight) {}
        public virtual void OnStartBackground(ref VolRandomWalk<PayloadType> walk, Ray ray, RgbColor initialWeight, float pdf) {}

        public virtual DirectionSample SampleNextDirection(ref VolRandomWalk<PayloadType> walk, in SurfaceShader shader,
                                                           RgbColor prefixWeight, int depth, int surfaceDepth)
        => walk.SampleBsdf(shader);

        public virtual float ComputeSurvivalProbability(ref VolRandomWalk<PayloadType> walk, in SurfacePoint hit, in Ray ray,
                                                        RgbColor prefixWeight, int depth)
        => walk.ComputeSurvivalProbability(depth);
    }

    public readonly RandomWalkModifier Modifier;
    public readonly Scene scene;
    public readonly int maxDepth;

    public Pixel FilmPosition;
    public bool isOnLightSubpath;
    public ref RNG rng;
    public PayloadType Payload;

    /// <summary>
    /// Tracks the product of (approximated) surface reflectances along the path. This is a more reliable
    /// quantity to use for Russian roulette than the prefix weight.
    /// </summary>
    public RgbColor ApproxThroughput = RgbColor.White;

    /// <summary>
    /// Initializes a new random walk
    /// </summary>
    /// <param name="scene">The scene</param>
    /// <param name="rng">Reference to the random number generator that is used to sample the path</param>
    /// <param name="maxDepth">Maximum number of edges along the path</param>
    /// <param name="modifier">Defines callbacks to be invoked at the scattering events</param>
    public VolRandomWalk(Scene scene, ref RNG rng, int maxDepth, RandomWalkModifier modifier = null) {
        this.scene = scene;
        this.maxDepth = maxDepth;
        this.rng = ref rng;
        Modifier = modifier;
    }

    public RgbColor StartFromCamera(CameraRaySample cameraRay, Pixel filmPosition, PayloadType payload) {
        isOnLightSubpath = false;
        FilmPosition = filmPosition;
        Payload = payload;
        Modifier?.OnStartCamera(ref this, cameraRay, filmPosition);

        return ContinueWalk(cameraRay.Ray, cameraRay.Point, cameraRay.PdfRay, cameraRay.Weight, 1, 1);
    }

    public RgbColor StartFromEmitter(EmitterSample emitterSample, RgbColor initialWeight, PayloadType payload) {
        isOnLightSubpath = true;
        Payload = payload;
        Modifier?.OnStartEmitter(ref this, emitterSample, initialWeight);

        Ray ray = Raytracer.SpawnRay(emitterSample.Point, emitterSample.Direction);
        return ContinueWalk(ray, emitterSample.Point, emitterSample.Pdf, initialWeight, 1, 1);
    }

    public RgbColor StartFromBackground(Ray ray, RgbColor initialWeight, float pdf, PayloadType payload) {
        isOnLightSubpath = true;
        Payload = payload;
        Modifier?.OnStartBackground(ref this, ray, initialWeight, pdf);

        // Find the first actual hitpoint on scene geometry
        bool notHitGeometry = true;
        Hit hit;
        do {
            hit = scene.Raytracer.Trace(ray);
            if (!hit) {
                var contrib = Modifier?.OnInvalidHit(ref this, ray, pdf, initialWeight, 1, 1) ?? RgbColor.Black;
                Modifier?.OnTerminate(ref this);
                return contrib;
            }
            DistanceSample distSample = scene.GlobalVolume.SampleDistance(rng.NextFloat());
            if (hit.Distance > distSample.distance) {
                //Volume hit
                //Call OnVolumeHit and shit
            }
            else {
                //Adjust
                notHitGeometry = false;
            }
        } while (notHitGeometry);



        SurfaceShader shader = new(hit, -ray.Direction, isOnLightSubpath);

        // Sample the next direction (required to know the reverse pdf)
        //  = SampleNextDirection(shader, initialWeight, 1);
        var dirSample = Modifier?.SampleNextDirection(ref this, shader, initialWeight, 1, 1)
            ?? SampleBsdf(shader);
        ApproxThroughput *= dirSample.ApproxReflectance;

        // Both pdfs have unit sr-1
        float pdfFromAncestor = pdf;
        float pdfToAncestor = dirSample.PdfReverse;

        RgbColor estimate = Modifier?.OnHit(ref this, shader, pdfFromAncestor, initialWeight, 1, 1, 1.0f) ?? RgbColor.Black;
        Modifier?.OnContinue(ref this, pdfToAncestor, 1, 1);

        // Terminate if the maximum depth has been reached
        if (maxDepth <= 1) {
            Modifier?.OnTerminate(ref this);
            return estimate;
        }

        // Terminate absorbed paths and invalid samples
        if (dirSample.PdfForward == 0 || dirSample.Weight == RgbColor.Black) {
            Modifier?.OnTerminate(ref this);
            return estimate;
        }

        // Continue the path with the next ray
        ray = Raytracer.SpawnRay(hit, dirSample.Direction);
        return estimate + ContinueWalk(ray, hit, dirSample.PdfForward, initialWeight * dirSample.Weight, 2);
    }

    public DirectionSample SampleBsdf(in SurfaceShader shader) {
        var bsdfSample = shader.Sample(rng.NextFloat2D());
        return new(
            bsdfSample.Pdf,
            bsdfSample.PdfReverse,
            bsdfSample.Weight,
            bsdfSample.Direction,
            bsdfSample.Weight
        );
    }

    public float ComputeSurvivalProbability(int depth) {
        if (depth > 4)
            return Math.Clamp(ApproxThroughput.Average, 0.05f, 0.95f);
        else
            return 1.0f;
    }

    RgbColor ContinueWalk(Ray ray, SurfacePoint previousPoint, float pdfDirection, RgbColor prefixWeight, int depth, int surfaceDepth, HomogeneousVolume volume) {
        RgbColor estimate = RgbColor.Black;
        while (depth < maxDepth) {
            var hit = scene.Raytracer.Trace(ray);
            if (!hit) {
                estimate += Modifier?.OnInvalidHit(ref this, ray, pdfDirection, prefixWeight, depth, surfaceDepth) ?? RgbColor.Black;
                break;
            }

            DistanceSample distSample = volume.SampleDistance(rng.NextFloat());
            if (hit.Distance > distSample.distance) {
                float pdfFromAncestor = pdfDirection * distSample.pdf;
                Vector3 newPosition = ray.Direction * distSample.distance;
                estimate += Modifier?.OnVolumeHit(ref this, newPosition, volume, pdfFromAncestor, prefixWeight, depth, surfaceDepth, 1.0f) ?? RgbColor.Black; //Jacobian is 1.0 because no conversion should happen

                // Don't sample continuations if we are going to terminate anyway
                if (depth + 1 >= maxDepth)
                    break;

                // Terminate with Russian roulette
                float survivalProb = Modifier?.ComputeSurvivalProbability(ref this, hit, ray, prefixWeight, depth)
                    ?? ComputeSurvivalProbability(depth);
                if (rng.NextFloat() > survivalProb)
                    break;

                var dirSample = volume.SampleDirection(Vector3.Normalize(previousPoint.Position - newPosition), rng.NextFloat2D());
                ApproxThroughput *= dirSample.weight / survivalProb;
                float pdfToAncestor = dirSample.reversePdf;

                Modifier?.OnContinue(ref this, pdfToAncestor, depth, surfaceDepth);

                if (dirSample.pdf == 0 || dirSample.weight == RgbColor.Black)
                    break;

                // Continue the path with the next ray
                prefixWeight *= dirSample.weight * volume.Transmittance(newPosition, previousPoint.Position) / survivalProb;
                depth++;
                pdfDirection = dirSample.pdf;
                previousPoint = new() { Position = newPosition}; //There may be some issues with this
                ray = Raytracer.SpawnRay(hit, dirSample.direction);

            } else {
                SurfaceShader shader = new(hit, -ray.Direction, isOnLightSubpath);

                // Convert the PDF of the previous hemispherical sample to surface area
                float pdfFromAncestor = pdfDirection * SampleWarp.SurfaceAreaToSolidAngle(previousPoint, hit);

                // Geometry term might be zero due to, e.g., shading normal issues
                // Avoid NaNs in that case by terminating early
                if (pdfFromAncestor == 0) break;

                float jacobian = SampleWarp.SurfaceAreaToSolidAngle(hit, previousPoint);
                estimate += Modifier?.OnHit(ref this, shader, pdfFromAncestor, prefixWeight, depth, surfaceDepth, jacobian) ?? RgbColor.Black;

                // Don't sample continuations if we are going to terminate anyway
                if (depth + 1 >= maxDepth)
                    break;

                // Terminate with Russian roulette
                float survivalProb = Modifier?.ComputeSurvivalProbability(ref this, hit, ray, prefixWeight, depth)
                    ?? ComputeSurvivalProbability(depth);
                if (rng.NextFloat() > survivalProb)
                    break;

                // Sample the next direction and convert the reverse pdf
                // var (pdfNext, pdfReverse, weight, direction) = SampleNextDirection(shader, prefixWeight, depth);
                var dirSample = Modifier?.SampleNextDirection(ref this, shader, prefixWeight, depth, surfaceDepth) ?? SampleBsdf(shader);
                ApproxThroughput *= dirSample.ApproxReflectance / survivalProb;
                float pdfToAncestor = dirSample.PdfReverse * SampleWarp.SurfaceAreaToSolidAngle(hit, previousPoint);

                Modifier?.OnContinue(ref this, pdfToAncestor, depth, surfaceDepth);

                if (dirSample.PdfForward == 0 || dirSample.Weight == RgbColor.Black)
                    break;

                // Continue the path with the next ray
                prefixWeight *= dirSample.Weight * volume.Transmittance(previousPoint.Position, hit.Position) / survivalProb / volume.DistanceGreaterProb(hit.Distance);
                depth++;
                surfaceDepth++;
                pdfDirection = dirSample.PdfForward;
                previousPoint = hit;
                ray = Raytracer.SpawnRay(hit, dirSample.Direction);
            }
            
            
        }

        Modifier?.OnTerminate(ref this);
        return estimate;
    }
}