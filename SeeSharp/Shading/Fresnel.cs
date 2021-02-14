﻿using System;

namespace SeeSharp.Shading {
    public interface Fresnel {
        ColorRGB Evaluate(float cosine);
    }

    public struct FresnelConductor : Fresnel {
        public ColorRGB EtaI;
        public ColorRGB EtaT;
        public ColorRGB K;

        ColorRGB Fresnel.Evaluate(float cosine) => Evaluate(MathF.Abs(cosine), EtaI, EtaT, K);

        // https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
        public static ColorRGB Evaluate(float cosThetaI, ColorRGB etai, ColorRGB etat, ColorRGB k) {
            cosThetaI = Math.Clamp(cosThetaI, -1, 1);
            ColorRGB eta = etat / etai;
            ColorRGB etak = k / etai;

            float cosThetaI2 = cosThetaI * cosThetaI;
            float sinThetaI2 = 1 - cosThetaI2;
            ColorRGB eta2 = eta * eta;
            ColorRGB etak2 = etak * etak;

            ColorRGB t0 = eta2 - etak2 - sinThetaI2;
            ColorRGB a2plusb2 = ColorRGB.Sqrt(t0 * t0 + 4 * eta2 * etak2);
            ColorRGB t1 = a2plusb2 + cosThetaI2;
            ColorRGB a = ColorRGB.Sqrt(0.5f * (a2plusb2 + t0));
            ColorRGB t2 = 2 * cosThetaI * a;
            ColorRGB Rs = (t1 - t2) / (t1 + t2);

            ColorRGB t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
            ColorRGB t4 = t2 * sinThetaI2;
            ColorRGB Rp = Rs * (t3 - t4) / (t3 + t4);

            return 0.5f * (Rp + Rs);
        }
    }

    public struct FresnelDielectric : Fresnel {
        public float EtaI;
        public float EtaT;

        ColorRGB Fresnel.Evaluate(float cosine) => new ColorRGB(Evaluate(cosine, EtaI, EtaT));

        public static float Evaluate(float cosThetaI, float etaI, float etaT) {
            cosThetaI = Math.Clamp(cosThetaI, -1, 1);
            // Potentially swap indices of refraction
            bool entering = cosThetaI > 0;
            if (!entering) {
                (etaT, etaI) = (etaI, etaT);
                cosThetaI = Math.Abs(cosThetaI);
            }

            // Compute _cosThetaT_ using Snell's law
            float sinThetaI = MathF.Sqrt(Math.Max(0, 1 - cosThetaI * cosThetaI));
            float sinThetaT = etaI / etaT * sinThetaI;

            // Handle total internal reflection
            if (sinThetaT >= 1) return 1;
            float cosThetaT = MathF.Sqrt(Math.Max(0, 1 - sinThetaT * sinThetaT));
            float Rparl = (etaT * cosThetaI - etaI * cosThetaT) /
                  (etaT * cosThetaI + etaI * cosThetaT);
            float Rperp = (etaI * cosThetaI - etaT * cosThetaT) /
                  (etaI * cosThetaI + etaT * cosThetaT);
            return (Rparl * Rparl + Rperp * Rperp) / 2;
        }
    }

    public struct FresnelSchlick {
        // https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
        //
        // The Schlick Fresnel approximation is:
        //
        // R = R(0) + (1 - R(0)) (1 - cos theta)^5,
        //
        // where R(0) is the reflectance at normal indicence.
        public static float SchlickWeight(float cosTheta) {
            float m = Math.Clamp(1 - cosTheta, 0, 1);
            return (m * m) * (m * m) * m;
        }

        public static float Evaluate(float R0, float cosTheta) {
            var w = SchlickWeight(cosTheta);
            return (1 - w) * R0 + w;
        }

        public static ColorRGB Evaluate(ColorRGB R0, float cosTheta) {
            return ColorRGB.Lerp(SchlickWeight(cosTheta), R0, ColorRGB.White);
        }

        // For a dielectric, R(0) = (eta - 1)^2 / (eta + 1)^2, assuming we're
        // coming from air..
        public static float SchlickR0FromEta(float eta) {
            var ratio = (eta - 1) / (eta + 1);
            return ratio * ratio;
        }
    }

    // Specialized Fresnel function used for the specular component, based on
    // a mixture between dielectric and the Schlick Fresnel approximation.
    public struct DisneyFresnel : Fresnel {
        public ColorRGB ReflectanceAtNormal;
        public float Metallic;
        public float IndexOfRefraction;

        ColorRGB Fresnel.Evaluate(float cosI) {
            var diel = new ColorRGB(FresnelDielectric.Evaluate(cosI, 1, IndexOfRefraction));
            var schlick = FresnelSchlick.Evaluate(ReflectanceAtNormal, cosI);
            return ColorRGB.Lerp(Metallic, diel, schlick);
        }
    }
}
