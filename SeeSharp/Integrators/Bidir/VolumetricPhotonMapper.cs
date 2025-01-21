using SeeSharp.Shading.Volumes;
using System.Reflection.Metadata.Ecma335;

namespace SeeSharp.Integrators.Bidir;

/// <summary>
/// A pure photon mapper in its most naive form: merging at the first camera vertex with a fixed radius
/// computed from a fraction of the scene size.
/// TODO: being able to sample emissive volume (not trivial!)
/// </summary>
public class VolumetricPhotonMapper : Integrator {
    /// <summary>
    /// Number of iterations to render.
    /// </summary>
    public int NumIterations = 2;

    /// <summary>
    /// Maximum number of nearest neighbor photons to search for.
    /// </summary>
    public int MaxNumPhotons = 10;

    /// <summary>
    /// Number of light paths in each iteration.
    /// </summary>
    public int NumLightPaths = 0;

    /// <summary>
    /// Seed for the random samples used to generate the photons
    /// </summary>
    public uint BaseSeedLight = 0xC030114u;

    /// <summary>
    /// Seed for the random samples used to generate the camera rays
    /// </summary>
    public uint BaseSeedCamera = 0x13C0FEFEu;

    /// <summary>
    /// The scene that is currently rendered
    /// </summary>
    protected Scene scene;

    /// <summary>
    /// Generates and stores the light paths / photons
    /// </summary>
    protected VolLightPathCache lightPaths;

    NearestNeighborSearch<int> surfacePhotonMap;
    NearestNeighborSearch<int> volumePhotonMap;

    /// <inheritdoc />
    public override void Render(Scene scene) {
        this.scene = scene;

        if (NumLightPaths <= 0) {
            NumLightPaths = scene.FrameBuffer.Width * scene.FrameBuffer.Height;
        }

        lightPaths = new VolLightPathCache {
            MaxDepth = MaxDepth,
            NumPaths = NumLightPaths,
            Scene = scene,
        };

        surfacePhotonMap ??= new();
        volumePhotonMap ??= new();

        for (uint iter = 0; iter < NumIterations; ++iter) {
            scene.FrameBuffer.StartIteration();
            lightPaths.TraceAllPaths(BaseSeedLight, iter, null);
            ProcessPathCache();
            TraceAllCameraPaths(iter);
            scene.FrameBuffer.EndIteration();
            surfacePhotonMap.Clear();
            volumePhotonMap.Clear();
        }

        surfacePhotonMap.Dispose();
        surfacePhotonMap = null;
        volumePhotonMap.Dispose();
        volumePhotonMap = null;
    }

    List<(int PathIndex, int VertexIndex)> photons = new();

    /// <summary>
    /// Builds the photon maps from the cached light paths
    /// </summary>
    protected virtual void ProcessPathCache() {
        int index = 0;
        int volVertices = 0, surVertices = 0;
        photons = [];
        for (int i = 0; i < lightPaths.NumPaths; ++i) {
            for (int k = 1; k < lightPaths.Length(i); ++k) {
                var vertex = lightPaths[i, k];
                if (vertex.Depth >= 1 && vertex.Weight != RgbColor.Black) {
                    if (vertex.IsVolVertex())
                        volVertices++;
                    else
                        surVertices++;
                    var photonMap = vertex.IsVolVertex() ? volumePhotonMap : surfacePhotonMap;
                    photonMap.AddPoint(vertex.SurPoint.Position, index++);
                    photons.Add((i, k));
                }
            }
        }
        Logger.Log($"This iteration surface | volume photons are {surVertices} | {volVertices}");
        surfacePhotonMap.Build();
        volumePhotonMap.Build();
    }

    RgbColor Merge(float radius, SurfacePoint hit, Vector3 outDir, int pathIdx, int vertIdx, float distSqr,
                   float radiusSquared) {
        // Compute the contribution of the photon
        var photon = lightPaths[pathIdx, vertIdx];
        var ancestor = lightPaths[pathIdx, vertIdx - 1];
        var dirToAncestor = Vector3.Normalize(ancestor.SurPoint.Position - photon.SurPoint.Position);
        var bsdfValue = photon.SurPoint.Material.Evaluate(hit, outDir, dirToAncestor, false);
        var photonContrib = photon.Weight * bsdfValue / NumLightPaths;

        // Epanechnikov kernel
        photonContrib *= (radiusSquared - distSqr) * 2.0f / (radiusSquared * radiusSquared * MathF.PI);

        return photonContrib;
    }
    /// <summary>
    /// Calculates in-scattered photon contribution at "hit" from photon corresponding to path/vertex indices.
    /// The given indices must correspond to a photon corresponding to a volume hit. 
    /// </summary>
    /// <param name="radius"></param>
    /// <param name="hitPoint"></param>
    /// <param name="outDir"></param>
    /// <param name="pathIdx"></param>
    /// <param name="vertIdx"></param>
    /// <param name="distSqr"></param>
    /// <param name="radiusSquared"></param>
    /// <returns></returns>
    RgbColor Merge3D(float radius, Vector3 hitPoint, Vector3 outDir, int pathIdx, int vertIdx, float distSqr,
                   float radiusSquared) {
        // Compute the contribution of the photon
        var photon = lightPaths[pathIdx, vertIdx];
        var ancestor = lightPaths[pathIdx, vertIdx - 1];
        var dirToAncestor = Vector3.Normalize(ancestor.SurPoint.Position - photon.SurPoint.Position);
        var volume = photon.Volume;
        var photonContrib = volume.InScatteredRadiance(photon.Weight, hitPoint, dirToAncestor, outDir) / NumLightPaths;

        // 3d Epanechnikov kernel
        photonContrib *= (radiusSquared - distSqr) * 15.0f / (8.0f * radiusSquared * radiusSquared * radius * MathF.PI);

        return photonContrib;
    }
    float ComputeSurvivalProbability(RgbColor throughput, int depth) {
        if (depth > 4)
            return Math.Clamp(throughput.Average, 0.50f, 0.95f);
        else
            return 1.0f;
    }

    /// <summary>
    /// Computes the estimated radiance travelling along a sampled camera ray
    /// </summary>
    /// <param name="pixel">Position on the image plane</param>
    /// <param name="ray">Ray sampled from the camera</param>
    /// <param name="weight">Contribution of the ray to the image, multiplied with the radiance</param>
    /// <param name="rng">Random number generator</param>
    /// <returns>Pixel value estimate</returns>
    protected virtual RgbColor EstimatePixelValue(Vector2 pixel, Ray ray, RgbColor weight, ref RNG rng) {
        // Trace the primary ray into the scene
        RgbColor estimate = RgbColor.Black; //Radiance accumulator
        int depth = 1;
        Stack<HomogeneousVolume> volumeStack = new(); volumeStack.Push(scene.GlobalVolume);
        bool notHitDiffuse = true;
        Hit hit;
        Emitter light;
        do {
            HomogeneousVolume volume = volumeStack.Peek();
            hit = scene.Raytracer.Trace(ray);
            if (!hit) {
                if (volume.IsVacuum())
                    return weight * scene.Background?.EmittedRadiance(ray.Direction) ?? RgbColor.Black;
                else {
                    return RgbColor.Black;
                }
            }
            var distSample = volume.SampleDistance(rng.NextFloat());
            if (distSample.distance < hit.Distance) {
                Vector3 volPosition = ray.ComputePoint(distSample.distance);
                weight *= distSample.weight;
                estimate += weight * volume.EmittedRadiance(volPosition, -ray.Direction);

                //for each photon in radius, compute radiance contribution, estimate in scattered radiance
                float maxRadius = scene.Radius / 10.0f;

                volumePhotonMap.ForAllNearest(volPosition, MaxNumPhotons, maxRadius, (position, idx, distance, numFound, maxDist) => {
                    float kernelRadius = numFound == MaxNumPhotons ? maxDist : maxRadius;
                    estimate += weight * Merge3D(kernelRadius, volPosition, -ray.Direction, photons[idx].PathIndex, photons[idx].VertexIndex,
                        distance * distance, kernelRadius * kernelRadius);
                });

                if (depth > MaxDepth)
                    return estimate;

                float survivalProb = ComputeSurvivalProbability(weight, depth);
                if (rng.NextFloat() > survivalProb) {
                    return estimate;
                }

                var dirSample = volume.SampleDirection(-ray.Direction, rng.NextFloat2D());
                weight *= dirSample.weight / survivalProb;
                ray = new Ray() {
                    Direction = dirSample.direction,
                    Origin = volPosition,
                    MinDistance = 0
                };
            } else {
                weight *= volume.Transmittance(hit.Distance) / volume.DistanceGreaterProb(hit.Distance);
                SurfaceShader shader = new(hit, -ray.Direction, false);
                if (shader.GetRoughness() < 0.1f) {
                    light = scene.QueryEmitter(hit);

                    estimate += weight * light?.EmittedRadiance(hit, -ray.Direction) ?? RgbColor.Black; //Maybe MIS?

                    if (depth > MaxDepth)
                        return estimate;

                    float survivalProb = ComputeSurvivalProbability(weight, depth);
                    if (rng.NextFloat() > survivalProb) {
                        return estimate;
                    }

                    var dirSample = shader.Sample(rng.NextFloat2D());

                    //Avoid NaN
                    if (dirSample.Weight == RgbColor.Black || dirSample.Pdf == 0.0f)
                        return estimate;

                    weight *= dirSample.Weight / survivalProb;

                    var crossedVolume = shader.material.InterfaceTo;
                    if (crossedVolume != null && dirSample.Transmission) {
                        //bool entering = Vector3.Dot(dirSample.Direction, shader.Context.Normal) <= 0.0f;
                        if (volume == crossedVolume) {
                            volumeStack.Pop();
                        } else {
                            volumeStack.Push(crossedVolume);
                        }
                    }

                    depth++;
                    ray = Raytracer.SpawnRay(hit, dirSample.Direction);
                }
                else 
                    notHitDiffuse = false; //exit the loop
            }
        } while (notHitDiffuse);


        //Hit a surface!
        // Gather nearby photons
        float radius = scene.Radius / 1000.0f;
        float footprint = hit.Distance * MathF.Tan(0.1f * MathF.PI / 180);
        radius = MathF.Min(footprint, radius);

        surfacePhotonMap.ForAllNearest(hit.Position, MaxNumPhotons, radius, (position, idx, distance, numFound, maxDist) => {
            float radiusSquared = numFound == MaxNumPhotons ? maxDist * maxDist : radius * radius;
            estimate += weight * Merge(radius, hit, -ray.Direction, photons[idx].PathIndex, photons[idx].VertexIndex,
                distance * distance, radius * radius);
        });

        // Add contribution from directly visible light sources
        light = scene.QueryEmitter(hit);
        if (light != null) {
            estimate += weight * light.EmittedRadiance(hit, -ray.Direction);
        }

        return estimate;
    }

    private void RenderPixel(uint row, uint col, ref RNG rng) {
        // Sample a ray from the camera
        var offset = rng.NextFloat2D();
        var filmSample = new Vector2(col, row) + offset;
        var cameraRay = scene.Camera.GenerateRay(filmSample, ref rng);
        var value = EstimatePixelValue(filmSample, cameraRay.Ray, cameraRay.Weight, ref rng);
        scene.FrameBuffer.Splat((int)col, (int)row, value);
    }

    private void TraceAllCameraPaths(uint iter) {
        Parallel.For(0, scene.FrameBuffer.Height,
            row => {
                var rng = new RNG(BaseSeedCamera, (uint)row, iter);
                for (uint col = 0; col < scene.FrameBuffer.Width; ++col) {
                    RenderPixel((uint)row, col, ref rng);
                }
            }
        );
    }
}
