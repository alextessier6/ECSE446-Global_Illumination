/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

#include "core/core.h"

TR_NAMESPACE_BEGIN

/**
 * Modified Phong reflectance model + Diffuse
 */
struct MixtureBSDF : BSDF {
    std::unique_ptr<Texture < v3f>> specularReflectance;
    std::unique_ptr<Texture < v3f>> diffuseReflectance;
    std::unique_ptr<Texture < float>> exponent;
    float specularSamplingWeight;
    float scale;

    MixtureBSDF(const WorldData& scene, const Config& config, const size_t& matID) : BSDF(scene, config, matID) {
        const tinyobj::material_t& mat = scene.materials[matID];

        if (mat.specular_texname.empty())
            specularReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.specular)));
        else
            specularReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.specular_texname));

        if (mat.diffuse_texname.empty())
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new ConstantTexture3f(glm::make_vec3(mat.diffuse)));
        else
            diffuseReflectance = std::unique_ptr<Texture<v3f>>(new BitmapTexture3f(config, mat.diffuse_texname));

        exponent = std::unique_ptr<Texture<float>>(new ConstantTexture1f(mat.shininess));

        //get scale value to ensure energy conservation
        v3f maxValue = specularReflectance->getMax() + diffuseReflectance->getMax();
        float actualMax = max(max(maxValue.x, maxValue.y), maxValue.z);
        scale = actualMax > 1.0f ? 0.99f * (1.0f / actualMax) : 1.0f;

        float dAvg = getLuminance(diffuseReflectance->getAverage() * scale);
        float sAvg = getLuminance(specularReflectance->getAverage() * scale);
        specularSamplingWeight = sAvg / (dAvg + sAvg);

        components.push_back(EGlossyReflection);
        components.push_back(EDiffuseReflection);

        combinedType = 0;
        for (unsigned int component : components)
            combinedType |= component;
    }

    inline v3f reflect(const v3f& d) const {
        return v3f(-d.x, -d.y, d.z);
    }

    v3f eval(const SurfaceInteraction& i) const override {

        //Base case, if lighting direction is negative
        v3f val(0.f);

        //Checks that the incoming ray is not hitting a backface
        if(Frame::cosTheta(i.wo) > 0 && Frame::cosTheta(i.wi) > 0) {

            //Gathers the required data to compute the BRDF
            v3f diffReflectivity = diffuseReflectance -> eval(worldData, i);
            v3f specReflectivity = specularReflectance -> eval(worldData, i);
            float exp = exponent -> eval(worldData, i);

            //The specular angle is the angle between the perfect specular reflect direction and the lighting direction
            //Reflecting i.wo and computing the angle with i.wi is the same as reflecting i.wi and computing the angle with i.wo
            v3f specularDir = (reflect(i.wo));
            specularDir = normalize(specularDir);
            float specularAngle = fmax(0.f, glm::angle(specularDir, i.wi));

            //Computes the scaling factor
            float cosFactor = Frame::cosTheta(i.wi);

            //Computes the BRDF
            //POW(cos(angle), exp) returns NAN if cos(angle) is negative, the absolute value of cos(angle) is taken instead
            val = scale * cosFactor * ((diffReflectivity * INV_PI) + ((specReflectivity) * (exp + 2.f) * INV_TWOPI * pow(abs(cos(specularAngle)), exp)));
        }

        return val;
    }

    float pdf(const SurfaceInteraction& i) const override {
        float pdf = 0.f;

        float pdfSpec;
        float pdfDiff;

        //Gets the pdf for the diffuse component
        pdfDiff = fmax(Warp::squareToCosineHemispherePdf(i.wi), 0);

        //------------------SPECULAR--------------------------
        //Reflection of the view vector
        v3f woRefl = reflect(i.wo);

        //Rotates the lobe to align it with the camera ray
        glm::quat rot(woRefl, v3f(0.0f, 0.0f, 1.0f));
        v3f wi = glm::toMat4(rot) * v4f(i.wi, 1.f);

        float exp = exponent -> eval(worldData, i);
        pdfSpec = fmax(Warp::squareToPhongLobePdf(wi, exp), 0);
        //----------------------------------------------------

        //Weighted average
        pdf = pdfDiff * (1-specularSamplingWeight) + pdfSpec * specularSamplingWeight;

        return pdf;
    }

    v3f sample(SurfaceInteraction& i, const v2f& _sample, float* pdf1) const override {
        v3f val(0.f);

        //tests x-component for type
        bool specular = _sample.x < specularSamplingWeight;

        if(specular){ //do glossy

            //rescale x-component based on weight
            v2f _sampleScaled(_sample.x / specularSamplingWeight, _sample.y);

            float exp = exponent -> eval(worldData, i);

            //sample direction in BRDF lobe using cosine-power hemispherical distribution
            i.wi = Warp::squareToPhongLobe(_sampleScaled, exp);

            //Reflection of the view vector
            v3f woRefl = reflect(i.wo);

            //Rotates the lobe to align it with the camera ray
            glm::quat rot(v3f(0.0f, 0.0f, 1.0f), woRefl);
            i.wi = glm::toMat4(rot) * v4f(i.wi, 1.f);

            i.sampledType = 1; //setting it as specular (for mirror BRDF)

        } else { //do diffuse if not specular

            //rescale x-component based on weight
            v2f _sampleScaled((_sample.x - specularSamplingWeight) / (1 - specularSamplingWeight), _sample.y);

            //use cosine-weighted IS for diffuse
            i.wi = Warp::squareToCosineHemisphere(_sampleScaled);

            i.sampledType = 0; //setting it as diffuse (for mirror BRDF)
        }

        *pdf1 = pdf(i);

        if(*pdf1 != 0.f) {
            val = eval(i)/(*pdf1);
        }

        return val;
    }

    std::string toString() const override { return "Mixture"; }
};

TR_NAMESPACE_END