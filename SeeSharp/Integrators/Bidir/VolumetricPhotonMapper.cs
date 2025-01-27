using SeeSharp.Shading.Volumes;
using System.Reflection.Metadata.Ecma335;
using TinyEmbree;

namespace SeeSharp.Integrators.Bidir;

/// <summary>
/// A pure photon mapper in its most naive form: merging at the first camera vertex with a fixed radius
/// computed from a fraction of the scene size.
/// TODO: being able to sample emissive volume (not trivial!)
/// </summary>
/// 
public class VolumetricPhotonMapper : Integrator {
    /// <summary>
    /// Number of iterations to render.
    /// </summary>
    public int NumIterations = 2;

    /// <summary>
    /// Maximum number of nearest neighbor surface photons to search for.
    /// </summary>
    public int MaxNumSurPhotons = 10;

    /// <summary>
    /// Maximum number of nearest neighbor volume photons to search for.
    /// </summary>
    public int MaxNumVolPhotons = 40;

    /// <summary>
    /// Maximum number of ray marching steps done during integration of in-scattered and emission radiance along a ray.
    /// </summary>
    public int MaxRayMarchingDepth = 100;

    /// <summary>
    /// Number of light paths in each iteration.
    /// </summary>
    public int NumLightPaths = 0;

    /// <summary>
    /// Coefficient of radius reduction for surface progressive photon mapping.
    /// </summary>
    public float AlphaSur = 0.75f;

    /// <summary>
    /// Coefficient of radius reduction for volume progressive photon mapping.
    /// </summary>
    public float AlphaVol = 0.75f;


    /// <summary>
    /// Seed for the random samples used to generate the photons
    /// </summary>
    public uint BaseSeedLight = 0xC030114u;

    /// <summary>
    /// Seed for the random samples used to generate the camera rays
    /// </summary>
    public uint BaseSeedCamera = 0x13C0FEFEu;

    private float MaxSurRadius;
    private float MaxVolRadius;

    /// <summary>
    /// The scene that is currently rendered
    /// </summary>
    protected Scene scene {
        get;
        set {
            field = value;
            MaxSurRadius = value.Radius / 1000f;
            MaxVolRadius = value.Radius / 500f;
        }
    }


    /// <summary>
    /// Generates and stores the light paths / photons
    /// </summary>
    protected VolLightPathCache lightPaths;

    NearestNeighborSearch<int> surfacePhotonMap;
    NearestNeighborSearch<int> volumePhotonMap;

    public virtual float BackgroundProbability 
    => scene.Background != null && scene.GlobalVolume.IsVacuum() ? 1.0f / (1.0f + scene.Emitters.Count) : 0.0f;

    /// <summary>
    /// Randomly samples either the background or an emitter from the scene
    /// </summary>
    /// <returns>The emitter and its selection probability</returns>
    public virtual (Emitter, float) SelectLight(ref RNG rng) {
        Emitter light;
        if (BackgroundProbability > 0 && rng.NextFloat() <= BackgroundProbability) {
            light = null;
        } else {
            light = scene.Emitters[rng.NextInt(scene.Emitters.Count)];
        }
        return (light, SelectLightPmf(light));
    }

    /// <summary>
    /// Computes the sampling probability used by <see cref="SelectLight"/>
    /// </summary>
    /// <param name="em">An emitter in the scene</param>
    /// <returns>The selection probability</returns>
    public virtual float SelectLightPmf(Emitter em) {
        if (em == null) { // background
            return BackgroundProbability;
        } else {
            return (1 - BackgroundProbability) / scene.Emitters.Count;
        }
    }

    public void UpdateRadiuses(uint iter) {
        float reductionCoeffSur = MathF.Sqrt((iter + AlphaSur) / (iter + 1));
        float reductionCoeffVol = MathF.Pow((iter + AlphaVol) / (iter + 1), 1.0f/3);
        MaxSurRadius *= reductionCoeffSur;
        MaxVolRadius *= reductionCoeffVol;
    }

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
            UpdateRadiuses(iter);
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
        Logger.Log($"{surVertices} surface | {volVertices} volume photons have been laid this iteration.");
        surfacePhotonMap.Build();
        volumePhotonMap.Build();
    }

    RgbColor Merge(float radius, float dist, SurfacePoint hit, Vector3 outDir, int pathIdx, int vertIdx) {
        float distSqr = dist * dist;
        float radiusSqr = radius * radius;

        // Compute the contribution of the photon
        var photon = lightPaths[pathIdx, vertIdx];
        var ancestor = lightPaths[pathIdx, vertIdx - 1];
        var dirToAncestor = Vector3.Normalize(ancestor.SurPoint.Position - photon.SurPoint.Position);
        var bsdfValue = photon.SurPoint.Material.Evaluate(hit, outDir, dirToAncestor, false);
        var photonContrib = photon.Weight * bsdfValue / NumLightPaths;

        // Epanechnikov kernel
        photonContrib *= (radiusSqr - distSqr) * 2.0f / (radiusSqr * radiusSqr * MathF.PI);

        return photonContrib;
    }
    /// <summary>
    /// Calculates in-scattered photon contribution at "hit" from photon corresponding to path/vertex indices.
    /// The given indices must correspond to a photon corresponding to a volume hit. 
    /// </summary>
    /// <returns></returns>
    RgbColor Merge3D(float radius, float dist, Vector3 hitPoint, Vector3 outDir, int pathIdx, int vertIdx) {
        float distSqr = dist * dist;
        float radiusSqr = radius * radius;

        // Compute the contribution of the photon
        var photon = lightPaths[pathIdx, vertIdx];
        var ancestor = lightPaths[pathIdx, vertIdx - 1];
        var dirToAncestor = Vector3.Normalize(ancestor.SurPoint.Position - photon.SurPoint.Position);
        var volume = photon.Volume;
        var photonContrib = photon.Weight * volume.PhaseFunction(dirToAncestor, outDir) / NumLightPaths;
        if (!float.IsFinite(photonContrib.Average)) {
            //Logger.Log("Non-finite in-scattered photon contribution detected");
            return RgbColor.Black;
        }
        // 3d Epanechnikov kernel
        photonContrib *= (radiusSqr - distSqr) * 15.0f / (8.0f * radiusSqr * radiusSqr * radius * MathF.PI);

        //Uniform sphere kernel
        //photonContrib *= 3 / (4 * MathF.PI * radiusSquared * radius);

        return photonContrib;
    }
    float ComputeSurvivalProbability(RgbColor throughput, int depth) {
        if (depth > 4)
            return Math.Clamp(throughput.Average, 0.05f, 0.95f);
        else
            return 1.0f;
    }

    float ComputeVolumeSurvivalProbability(RgbColor throughput, int depth) {
        if (depth > 2)
            return Math.Clamp(throughput.Average, 0.05f, 0.95f);
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
    protected virtual RgbColor EstimatePixelValue(Vector2 pixel, Ray ray, RgbColor weight, uint iter, ref RNG rng) {
        RgbColor estimate = RgbColor.Black; //Radiance accumulator
        int depth = 1;
        bool performedNEE = false;
        float bsdf_pdf = 0.0f; //For MIS. Its default value will never be used.
        Hit prevHit = new();   //For MIS. Its default value will never be used.
        Hit hit;
        Emitter light;

        Stack<HomogeneousVolume> volumeStack = new();
        volumeStack.Push(scene.GlobalVolume);

        do {
            HomogeneousVolume volume = volumeStack.Peek();
            hit = scene.Raytracer.Trace(ray);

            //Guard: we missed the scene and return the estimate
            if (!hit) {
                if (scene.Background != null && volume.IsVacuum()) {
                    float bsdf_mis_weight = 1.0f;
                    if (performedNEE) {
                        float bsdf_nee_pdf = scene.Background.DirectionPdf(ray.Direction) * BackgroundProbability;
                        bsdf_mis_weight = 1 / (1 + (bsdf_nee_pdf/ bsdf_pdf));
                    }
                    estimate += weight * bsdf_mis_weight * scene.Background.EmittedRadiance(ray.Direction);
                }
                break;
            }

            //Integrate in-scattered radiance and emitted radiance along the ray crossing the volume

            float t = 0.0f; //Distance travelled through the volume
            float rr_accumulator = 1.0f; //I don't know how to call this
            for (int rmDepth=1; t < hit.Distance; rmDepth++) {
                if (rmDepth >= MaxRayMarchingDepth) {
                    Logger.Warning("MaxRayMarchingDepth reached. You might wanna raise this parameter.");
                    return estimate;
                }
                var distSample = volume.SampleDistance(rng.NextFloat());
                if (t + distSample.distance < hit.Distance) {
                    rr_accumulator *= volume.DistanceLeqProb(hit.Distance - t);
                    float mc_normalization = volume.DistanceLeqProb(distSample.distance) / (rr_accumulator * distSample.pdf); //I have the formula derivation in my notes somewhere. Believe we need this factor.
                    Vector3 volPosition = ray.ComputePoint(t + distSample.distance);
                    RgbColor emittedRadiance = volume.EmittedRadiance(volPosition, -ray.Direction);
                    RgbColor inScatteredRadiance = RgbColor.Black;

                    //for each photon in radius, compute radiance contribution, estimate in scattered

                    volumePhotonMap.ForAllNearest(volPosition, MaxNumVolPhotons, MaxVolRadius, (position, idx, distance, numFound, maxDist) => {
                        float kernelRadius = numFound == MaxNumVolPhotons ? maxDist : MaxVolRadius;
                        inScatteredRadiance += Merge3D(kernelRadius, distance, volPosition, -ray.Direction, photons[idx].PathIndex, photons[idx].VertexIndex);
                    });
                    return estimate + weight * distSample.weight * (volume.SigmaA * emittedRadiance + volume.SigmaS * inScatteredRadiance);
                    
                    //Combine everything as per heat transfer equation
                    estimate += weight * volume.Transmittance(t + distSample.distance) * mc_normalization * (volume.SigmaA * emittedRadiance + volume.SigmaS * inScatteredRadiance);

                    float survivalProb = ComputeVolumeSurvivalProbability(weight, rmDepth);
                    if (rng.NextFloat() > survivalProb) {
                        return estimate;
                    }

                    weight /= survivalProb;
                }
                t += distSample.distance;
            }

            //Now we reached a surface.
            weight *= volume.Transmittance(hit.Distance) / volume.DistanceGreaterProb(hit.Distance);
            //weight *= volume.Transmittance(hit.Distance);
            SurfaceShader shader = new(hit, -ray.Direction, false);
            if (shader.GetRoughness() < 0.5f || depth <= 4) { //If the surface is kinda-specular we perform path tracing
                light = scene.QueryEmitter(hit);
                if (light != null) {
                    float bsdf_mis_weight = 1.0f;
                    if (performedNEE) {
                        float bsdf_nee_pdf = light.PdfUniformArea(hit) / SampleWarp.SurfaceAreaToSolidAngle(prevHit, hit) * SelectLightPmf(light);
                        bsdf_mis_weight = 1 / (1 + (bsdf_nee_pdf / bsdf_pdf));
                    }
                    estimate += weight * bsdf_mis_weight * light.EmittedRadiance(hit, -ray.Direction);
                }

                if (depth + 1 > MaxDepth)
                    break;

                //RR
                float survivalProb = ComputeSurvivalProbability(weight, depth);
                if (rng.NextFloat() > survivalProb) {
                    break;
                }

                weight /= survivalProb;

                //NEE
                if (volume.IsVacuum() && shader.GetRoughness() > 0.0f) {
                    performedNEE = true;
                    RgbColor neeContrib = RgbColor.Black;
                    (light, float lightPmf) = SelectLight(ref rng);
                    Vector3 toLight;
                    float neePdf = lightPmf;
                    if (light == null) {
                        var bgSample = scene.Background.SampleDirection(rng.NextFloat2D()); // sr^-1 pdf
                        toLight = bgSample.Direction;
                        neePdf *= bgSample.Pdf;
                        if (scene.Raytracer.LeavesScene(hit, toLight))
                            neeContrib = bgSample.Weight / lightPmf * shader.EvaluateWithCosine(toLight);
                    } else {
                        var lightSample = light.SampleUniformArea(rng.NextFloat2D());
                        toLight = Vector3.Normalize(lightSample.Point.Position - hit.Position);
                        neePdf *= lightSample.Pdf / SampleWarp.SurfaceAreaToSolidAngle(hit, lightSample.Point);
                        if (!scene.Raytracer.IsOccluded(hit, lightSample.Point))
                            neeContrib = light.EmittedRadiance(lightSample.Point, -toLight) * shader.EvaluateWithCosine(toLight) / neePdf;
                    }
                    (float nee_bsdf_pdf, _) = shader.Pdf(toLight);
                    float nee_mis_weight = 1 / (1 + (nee_bsdf_pdf / neePdf));

                    //Avoid NaN
                    if (float.IsFinite(neeContrib.Average)) {
                        estimate += weight * neeContrib * nee_mis_weight;
                    }
                } else {
                    performedNEE = false;
                }

                //Sample new direction
                var dirSample = shader.Sample(rng.NextFloat2D());

                //Avoid NaN
                if (dirSample.Weight == RgbColor.Black || dirSample.Pdf == 0.0f)
                    return estimate;
                weight *= dirSample.Weight;
                bsdf_pdf = dirSample.Pdf;

                //Update volume if we crossed some volume-bounding mesh
                var crossedVolume = shader.material.InterfaceTo;
                if (crossedVolume != null && dirSample.Transmission) {
                    bool entering = Vector3.Dot(dirSample.Direction, shader.Context.Normal) <= 0.0f;
                    if (entering) {
                        volumeStack.Push(crossedVolume);
                    } else if (volumeStack.Count > 1) {
                        volumeStack.Pop();
                    }
                }
                ray = Raytracer.SpawnRay(hit, dirSample.Direction);
            } else { //else the surface is rough enough, we then perform surface photon mapping and stop path tracing.

                // Gather nearby photons
                surfacePhotonMap.ForAllNearest(hit.Position, MaxNumSurPhotons, MaxSurRadius, (position, idx, distance, numFound, maxDist) => {
                    float kernelRadius = numFound == MaxNumSurPhotons ? maxDist : MaxSurRadius;
                    estimate += weight * Merge(kernelRadius, distance, hit, -ray.Direction, photons[idx].PathIndex, photons[idx].VertexIndex);
                });

                // Add contribution from directly visible light sources
                light = scene.QueryEmitter(hit);
                if (light != null) {
                    estimate += weight * light.EmittedRadiance(hit, -ray.Direction);
                }

                break; //stop path tracing
            }
            prevHit = hit;
            depth++;
        } while (true);

        return estimate;
    }

    private void RenderPixel(uint row, uint col, uint iter, ref RNG rng) {
        // Sample a ray from the camera
        var offset = rng.NextFloat2D();
        var filmSample = new Vector2(col, row) + offset;
        var cameraRay = scene.Camera.GenerateRay(filmSample, ref rng);
        var value = EstimatePixelValue(filmSample, cameraRay.Ray, cameraRay.Weight, iter, ref rng);
        scene.FrameBuffer.Splat((int)col, (int)row, value);
    }

    private void TraceAllCameraPaths(uint iter) {
        Parallel.For(0, scene.FrameBuffer.Height,
            row => {
                var rng = new RNG(BaseSeedCamera, (uint)row, iter);
                for (uint col = 0; col < scene.FrameBuffer.Width; ++col) {
                    RenderPixel((uint)row, col, iter, ref rng);
                }
            }
        );
    }
}
