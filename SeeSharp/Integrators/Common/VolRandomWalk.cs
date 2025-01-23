using SeeSharp.Shading.Emitters;
using SeeSharp.Shading.Volumes;
using System.ComponentModel.DataAnnotations;
using System.Data;

namespace SeeSharp.Integrators.Common;

/// <summary>
/// Performs a random walk, invoking virtual callbacks for events along the path. The state of the walk is
/// tracked in this object, so it can only be used for one walk at a time.
/// TODO: being able to sample emissive volume (not trivial!)
/// </summary>
public ref struct VolRandomWalk<PayloadType> where PayloadType : new(){
    public record struct DirectionSample(
        float PdfForward,
        float PdfReverse,
        RgbColor Weight,
        Vector3 Direction,
        RgbColor ApproxReflectance,
        bool Transmission
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

        public virtual RgbColor OnContinue(ref VolRandomWalk<PayloadType> walk, Vector3 position, Vector3 outDirection, float pdfToAncestor, RgbColor throughput, HomogeneousVolume volume, int depth, int surfaceDepth, bool hitVolume)
            => RgbColor.Black;

        public virtual void OnTerminate(ref VolRandomWalk<PayloadType> walk) {}

        public virtual void OnStartCamera(ref VolRandomWalk<PayloadType> walk, CameraRaySample cameraRay, Pixel filmPosition) {}
        public virtual void OnStartEmitter(ref VolRandomWalk<PayloadType> walk, EmitterSample emitterSample, RgbColor initialWeight) {}
        public virtual void OnStartBackground(ref VolRandomWalk<PayloadType> walk, Ray ray, RgbColor initialWeight, float pdf, HomogeneousVolume volume) {}

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

        return ContinueWalk(cameraRay.Ray, cameraRay.Point, cameraRay.PdfRay, cameraRay.Weight, 1, 1, scene.GlobalVolume, false);
    }

    public RgbColor StartFromEmitter(EmitterSample emitterSample, RgbColor initialWeight, PayloadType payload) {
        isOnLightSubpath = true;
        Payload = payload;
        Modifier?.OnStartEmitter(ref this, emitterSample, initialWeight);

        Ray ray = Raytracer.SpawnRay(emitterSample.Point, emitterSample.Direction);
        return ContinueWalk(ray, emitterSample.Point, emitterSample.Pdf, initialWeight, 1, 1, scene.GlobalVolume, false);
    }
    public RgbColor StartFromBackground(Ray ray, RgbColor initialWeight, float pdf, PayloadType payload) {
        isOnLightSubpath = true;
        Payload = payload;
        Modifier?.OnStartBackground(ref this, ray, initialWeight, pdf, scene.GlobalVolume);
        //Starting from background inside a non-vacuum global volume might always result in black and wasted computation
        return ContinueWalk(ray, new SurfacePoint() {Position =  ray.Origin}, pdf, initialWeight, 1, 0, scene.GlobalVolume, false);
    }

    public DirectionSample SampleBsdf(in SurfaceShader shader) {
        var bsdfSample = shader.Sample(rng.NextFloat2D());
        return new(
            bsdfSample.Pdf,
            bsdfSample.PdfReverse,
            bsdfSample.Weight,
            bsdfSample.Direction,
            bsdfSample.Weight,
            bsdfSample.Transmission
        );
    }

    public float ComputeSurvivalProbability(int depth) {
        if (depth > 4)
            return Math.Clamp(ApproxThroughput.Average, 0.05f, 0.95f);
        else
            return 1.0f;
    }

    RgbColor ContinueWalk(Ray ray, SurfacePoint previousPoint, float pdfDirection, RgbColor prefixWeight, int depth, int surfaceDepth, HomogeneousVolume volume, bool previouslyHitVolume) {
        RgbColor estimate = RgbColor.Black;
        Stack<HomogeneousVolume> volumeStack = new(); volumeStack.Push(volume);
        while (depth < maxDepth) {
            volume = volumeStack.Peek();
            var hit = scene.Raytracer.Trace(ray);
            if (!hit) {
                estimate += Modifier?.OnInvalidHit(ref this, ray, pdfDirection, prefixWeight, depth, surfaceDepth) ?? RgbColor.Black;
                break;
            }

            DistanceSample distSample = volume.SampleDistance(rng.NextFloat());
            if (hit.Distance > distSample.distance) {
                Vector3 newPosition = ray.ComputePoint(distSample.distance);
                float pdfFromAncestor = pdfDirection * distSample.pdf;

                ApproxThroughput *= distSample.weight;
                prefixWeight *= distSample.weight;

                if (!float.IsFinite(prefixWeight.Average) || prefixWeight == RgbColor.Black)
                    break;

                float jacobian = 1.0f;
                //float jacobian = previouslyHitVolume ? 1.0f : SampleWarp.SurfaceAreaToSolidAngle(hit, previousPoint);
                estimate += Modifier?.OnVolumeHit(ref this, newPosition, volume, pdfFromAncestor, prefixWeight, depth, surfaceDepth, jacobian) ?? RgbColor.Black;

                // Don't sample continuations if we are going to terminate anyway
                if (depth > maxDepth)
                    break;

                // Terminate with Russian roulette
                float survivalProb = Modifier?.ComputeSurvivalProbability(ref this, hit, ray, prefixWeight, depth)
                    ?? ComputeSurvivalProbability(depth);
                if (rng.NextFloat() > survivalProb)
                    break;

                previouslyHitVolume = true;
                var dirSample = volume.SampleDirection(-ray.Direction, rng.NextFloat2D());
                float pdfToAncestor = dirSample.ReversePdf * jacobian;

                estimate += Modifier?.OnContinue(ref this, newPosition, dirSample.direction, pdfToAncestor, prefixWeight, volume, depth, surfaceDepth, previouslyHitVolume) ?? RgbColor.Black;

                if (dirSample.pdf == 0.0f || dirSample.weight == RgbColor.Black)
                    break;

                ApproxThroughput *= dirSample.weight / survivalProb;
                prefixWeight *= dirSample.weight / survivalProb;

                // Continue the path with the next ray
                depth++;
                pdfDirection = dirSample.pdf;
                previousPoint = new() { Position = newPosition};
                ray = new Ray() {
                    Direction = dirSample.direction,
                    Origin = newPosition,
                    MinDistance = 0
                };

            } else {
                SurfaceShader shader = new(hit, -ray.Direction, isOnLightSubpath);

                // Convert the PDF of the previous hemispherical sample to surface area
                float pdfFromAncestor = pdfDirection * volume.DistanceGreaterProb(hit.Distance);
                //float pdfFromAncestor = pdfDirection * volume.DistanceGreaterProb(hit.Distance) * SampleWarp.SurfaceAreaToSolidAngle(previousPoint, hit); //Maybe incorrect for the first iteration? I don't know...

                // Geometry term might be zero due to, e.g., shading normal issues
                // Avoid NaNs in that case by terminating early
                if (pdfFromAncestor == 0) break;

                RgbColor distanceWeight = volume.Transmittance(hit.Distance) / volume.DistanceGreaterProb(hit.Distance);

                ApproxThroughput *= distanceWeight;
                prefixWeight *= distanceWeight;

                float jacobian = 1.0f;
                //float jacobian = previouslyHitVolume ? 1.0f : SampleWarp.SurfaceAreaToSolidAngle(hit, previousPoint);
                estimate += Modifier?.OnHit(ref this, shader, pdfFromAncestor, prefixWeight, depth, surfaceDepth, jacobian) ?? RgbColor.Black;

                // Don't sample continuations if we are going to terminate anyway
                if (depth > maxDepth)
                    break;

                // Terminate with Russian roulette
                float survivalProb = Modifier?.ComputeSurvivalProbability(ref this, hit, ray, prefixWeight, depth)
                    ?? ComputeSurvivalProbability(depth);
                if (rng.NextFloat() > survivalProb)
                    break;

                // Sample the next direction and convert the reverse pdf
                // var (pdfNext, pdfReverse, weight, direction) = SampleNextDirection(shader, prefixWeight, depth);
                previouslyHitVolume = false;
                var dirSample = Modifier?.SampleNextDirection(ref this, shader, prefixWeight, depth, surfaceDepth) ?? SampleBsdf(shader);
                
                float pdfToAncestor = dirSample.PdfReverse * jacobian;

                estimate += Modifier?.OnContinue(ref this, hit.Position, dirSample.Direction, pdfToAncestor, prefixWeight, volume, depth, surfaceDepth, previouslyHitVolume) ?? RgbColor.Black;

                if (dirSample.PdfForward == 0 || dirSample.Weight == RgbColor.Black)
                    break;

                prefixWeight *= dirSample.Weight / survivalProb;
                ApproxThroughput *= dirSample.ApproxReflectance / survivalProb;

                // Continue the path with the next ray
                var crossedVolume = shader.material.InterfaceTo;
                if (crossedVolume != null && dirSample.Transmission) {
                    bool entering = Vector3.Dot(dirSample.Direction, shader.Context.Normal) <= 0.0f;
                    if (entering) {
                        volumeStack.Push(crossedVolume);
                    } else if (volumeStack.Count > 1) {
                        volumeStack.Pop();
                    }
                }
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