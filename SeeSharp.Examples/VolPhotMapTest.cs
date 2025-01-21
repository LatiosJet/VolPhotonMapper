using SeeSharp.Experiments;
using SeeSharp.Integrators;
using SeeSharp.Integrators.Bidir;
using System.Collections.Generic;

namespace SeeSharp.Examples;

/// <summary>
/// Renders a scene with Volumetric Photon Mapping
/// </summary>
class VolPhotMapTest : Experiment {
    public override List<Method> MakeMethods() => [
        new("Volumetric Photon Mapping", new VolumetricPhotonMapper() {
            NumIterations = 20000,
            MaxNumPhotons = 10,
            NumLightPaths = 0
        }),
    ];
}