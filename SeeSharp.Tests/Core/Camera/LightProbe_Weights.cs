﻿using SeeSharp.Cameras;
using System.Numerics;
using TinyEmbree;
using Xunit;

namespace SeeSharp.Tests.Core.Camera {
    public class LightProbe_Weights {
        LightProbeCamera MakeCamera() {
            // A simple plane with the z axis as its normal
            Raytracer raytracer = new();
            raytracer.AddMesh(new(new Vector3[] {
                new(-1, -1, 0),
                new( 1, -1, 0),
                new( 1,  1, 0),
                new(-1,  1, 0)
            }, new[] { 0, 1, 2, 0, 2, 3 }));
            raytracer.CommitScene();

            // Find the center point via ray tracing so we get a good error offset
            Hit hit = raytracer.Trace(new Ray {
                Origin = Vector3.UnitZ,
                Direction = -Vector3.UnitZ,
                MinDistance = 0
            });

            var camera = new LightProbeCamera(hit.Position, hit.Normal, hit.ErrorOffset, Vector3.UnitY);
            camera.UpdateResolution(512, 256);

            return camera;
        }

        [Fact]
        public void Position_ShouldBeZero() {
            var camera = MakeCamera();

            Assert.Equal(0.0f, camera.Position.X, 2);
            Assert.Equal(0.0f, camera.Position.Y, 2);
            Assert.Equal(0.0f, camera.Position.Z, 2);
        }

        [Fact]
        public void Direction_ShouldBeDown() {
            var camera = MakeCamera();

            RNG rng = new();
            var sample = camera.GenerateRay(new(200, 255), ref rng);

            Assert.Equal(0.0f, sample.Ray.Direction.X, 1);
            Assert.Equal(-1.0f, sample.Ray.Direction.Y, 2);
            Assert.Equal(0.0f, sample.Ray.Direction.Z, 1);
        }

        [Fact]
        public void Direction_ShouldBeUp() {
            var camera = MakeCamera();

            RNG rng = new();
            var sample = camera.GenerateRay(new(200, 1), ref rng);

            Assert.Equal(0.0f, sample.Ray.Direction.X, 1);
            Assert.Equal(1.0f, sample.Ray.Direction.Y, 2);
            Assert.Equal(0.0f, sample.Ray.Direction.Z, 1);
        }
    }
}
