/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"

TR_NAMESPACE_BEGIN

/**
 * Perfectly diffuse, Lambertian reflectance model
 */
struct DiffuseBSDF : BSDF {
    std::unique_ptr<Texture < v3f>> albedo;

    DiffuseBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
        const tinyobj::material_t& mat = scene.materials[matID];

        if (mat.diffuse_texname.empty())
            albedo = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.diffuse)));
        else
            albedo = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.diffuse_texname));

        components.push_back(EDiffuseReflection);

        combinedType = 0;
        for (size_t i = 0; i < components.size(); ++i)
            combinedType |= components[i];
    }

    v3f eval(const SurfaceInteraction& i) const override {

        //Base case, returns black if the ray is hitting the backface
        v3f val(0.f);

        //Checks that the incoming ray is not hitting a backface
        if(Frame::cosTheta(i.wo) > 0 && Frame::cosTheta(i.wi) > 0) {

            //Computes the BRDF using the given albedo value
            val = (albedo -> eval(worldData, i)) * Frame::cosTheta(i.wi) * M_1_PI;

        }
        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {
        float pdf = 0.f;

        pdf = Warp::squareToCosineHemispherePdf(i.wi);

        return pdf;
    }


    v3f sample(SurfaceInteraction& i, const v2f& sample, float* pdf) const override {
        v3f val(0.f);

        //sample direction in BRDF lobe using cosine-weighted hemispherical distribution
        i.wi = Warp::squareToCosineHemisphere(sample);

        //evaluate corresponding PDF in direction
        *pdf = Warp::squareToCosineHemispherePdf(i.wi);

        if(*pdf != 0.f)
            val = eval(i)/(*pdf);

        return val;
    }

    std::string toString() const override { return "Diffuse"; }
};

TR_NAMESPACE_END