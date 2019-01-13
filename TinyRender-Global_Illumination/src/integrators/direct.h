/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Direct illumination integrator with MIS
 */
struct DirectIntegrator : Integrator {
    explicit DirectIntegrator(const Scene& scene) : Integrator(scene) {
        m_emitterSamples = scene.config.integratorSettings.di.emitterSamples;
        m_bsdfSamples = scene.config.integratorSettings.di.bsdfSamples;
        m_samplingStrategy = scene.config.integratorSettings.di.samplingStrategy;
    }

    static inline float balanceHeuristic(float nf, float fPdf, float ng, float gPdf) {
        float f = nf * fPdf, g = ng * gPdf;
        return f / (f + g);
    }

    void sampleSphereByCosineHemisphere(const p2f& sample,
                                        const v3f& n,
                                        const p3f& pShading,
                                        const v3f& emitterCenter,
                                        float emitterRadius,
                                        v3f& wiW,
                                        float& pdf) const {
        // TODO: Implement this
    }

    void sampleSphereByArea(const p2f& sample,
                            const p3f& pShading,
                            const v3f& emitterCenter,
                            float emitterRadius,
                            v3f& pos,   //position on light
                            v3f& ne,    //normal emitter
                            v3f& wiW,   //wi in world
                            float& pdf) const {

        v3f sampleDir = Warp::squareToUniformSphere(sample);
        pdf = Warp::squareToUniformSpherePdf();
        pdf = pdf / pow(emitterRadius, 2);

        //Position in world
        pos = emitterCenter + emitterRadius * sampleDir;

        wiW = pos - pShading;
        wiW = normalize(wiW);

        //Normal of the emitter must be in local
        ne = normalize(emitterRadius * sampleDir);

    }

    void sampleSphereBySolidAngle(const p2f& sample,
                                  const p3f& pShading,
                                  const v3f& emitterCenter,
                                  float emitterRadius,
                                  v3f& wiW,
                                  float& pdf) const {

        //Calculates the subtended angle
        float sinThetaMax = emitterRadius * emitterRadius / pow(glm::distance(pShading, emitterCenter), 2);
        float cosThetaMax = std::sqrt(std::max(0.f, 1 - sinThetaMax));

        v3f sampleDir = Warp::squareToUniformCone(sample, cosThetaMax);
        pdf = Warp::squareToUniformConePdf(cosThetaMax);

        //Rotates the cone to align it with the emitter
        glm::quat rot(v3f(0.0f, 0.0f, 1.0f), emitterCenter - pShading);
        wiW = glm::toMat4(rot) * v4f(sampleDir, 1.f);
        wiW = normalize(wiW);

    }

    v3f renderArea(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);

        SurfaceInteraction i;
        v3f emission;

        //Checks if ray intersects the scene (to display emitter in scene)
        if (scene.bvh->intersect(ray, i)) {

            emission = getEmission(i);

            //Checks if the intersection lands directly on an emitter
            if (emission != v3f(0)) {
                Lr = emission;
                return Lr;
            }
        }

        for (int x = 0; x < m_emitterSamples; x++ ) {

            //PDF to be returned
            float emPdf;

            //Gets light and relevant data
            size_t id = selectEmitter(sampler.next(), emPdf);
            const Emitter& em = getEmitterByID(id);
            v3f emCenter = scene.getShapeCenter(em.shapeID);
            float emRadius = scene.getShapeRadius(em.shapeID);

            //Samples the light
            v3f pShading = i.p;
            v3f pos;
            v3f ne;
            v3f wiW;
            float pdf;
            sampleSphereByArea(sampler.next2D(), pShading, emCenter, emRadius, pos, ne, wiW, pdf);

            //wi must be in local coordinates
            i.wi = i.frameNs.toLocal(wiW);

            //Creates a ray with the intersection point and the sampled direction
            Ray shadowRay(i.p, wiW);

            SurfaceInteraction iShadow;

            //Checks if there is no object intersection between the shading point and the light source
            if (scene.bvh->intersect(shadowRay, iShadow)) {
                emission = getEmission(iShadow);

                //Checks if the intersection lies on a surface emitter
                if(emission != v3f(0)){

                    //Computes the geometry term
                    float geom = dot(ne, wiW) / pow(glm::distance(pos, pShading), 2);

                    //checks to see if sample was done in the right hemisphere
                    float vis = 1;
                    if(glm::dot(pos-pShading, ne) <= 0)
                        vis = 0;

                    v3f brdf = getBSDF(i) -> eval(i);

                    //Computes the value for the monte carlo estimator
                    Lr += vis * emission * brdf * geom / (pdf * emPdf);

                }
            }
        }
        Lr = Lr / m_emitterSamples;

        return Lr;
    }

    v3f renderCosineHemisphere(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);


        for (int x = 0; x < m_emitterSamples; x++ ) {

            SurfaceInteraction i;

            //Checks if ray intersects the scene
            if (scene.bvh->intersect(ray, i)) {

                v3f emission = getEmission(i);

                //Checks if the intersection lands directly on an emitter
                if(emission != v3f(0)){
                    Lr = emission;
                    return Lr;
                }

                //For Cosine-weighted Hemispherical Direction Sampling
                v3f mcSample = Warp::squareToCosineHemisphere(sampler.next2D());
                float pdf = Warp::squareToCosineHemispherePdf(mcSample);

                //Converts to world view
                i.wi = mcSample;
                mcSample = i.frameNs.toWorld(i.wi);
                mcSample = normalize(mcSample);

                //Creates a ray with the intersection point and the sampled direction
                Ray shadowRay(i.p, mcSample);

                SurfaceInteraction iShadow;

                //Checks if there is no object intersection
                if (scene.bvh->intersect(shadowRay, iShadow)) {
                    emission = getEmission(iShadow);

                    //Checks if the intersection lies on a surface emitter
                    if(emission != v3f(0)){

                        //float cosTheta = dot(i.wi, i.frameNs.n);

                        v3f brdf = getBSDF(i) -> eval(i);
                        //Computes the value for the monte carlo estimator
                        Lr += emission * brdf / pdf;

                    }
                }
            }
        }
        Lr = Lr / m_emitterSamples;
        return Lr;
    }

    v3f renderBSDF(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);

        SurfaceInteraction i;

        //Checks if ray intersects the scene
        if (scene.bvh->intersect(ray, i)) {

            v3f emission = getEmission(i);

            //Checks if the intersection lands directly on an emitter
            if (emission != v3f(0)) {
                Lr = emission;
                return Lr;
            }
        }

        for (int x = 0; x < m_emitterSamples; x++ ) {

            float pdf;
            v3f sample = getBSDF(i)->sample(i, sampler.next2D(), &pdf);

            //Converts to world view
            v3f dir = i.wi;
            dir = i.frameNs.toWorld(dir);
            dir = normalize(dir);

            //Creates a ray with the intersection point and the sampled direction
            Ray shadowRay(i.p, dir);

            SurfaceInteraction iShadow;

            //Checks if there is no object intersection
            if (scene.bvh->intersect(shadowRay, iShadow)) {
                v3f emission = getEmission(iShadow);

                //Checks if the intersection lies on a surface emitter
                if(emission != v3f(0)){

                    //Computes the value
                    Lr += emission * sample;

                }
            }
        }

        Lr = Lr / m_emitterSamples;

        return Lr;
    }

    v3f renderSolidAngle(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);
        SurfaceInteraction i;
        v3f emission;

        //Checks if ray intersects the scene (to display emitter in scene)
        if (scene.bvh->intersect(ray, i)) {

            emission = getEmission(i);

            //Checks if the intersection lands directly on an emitter
            if (emission != v3f(0)) {
                Lr = emission;
                return Lr;
            }
        }

        for (int x = 0; x < m_emitterSamples; x++ ) {

            //PDF to be returned
            float emPdf;

            //Gets light and relevant data
            size_t id = selectEmitter(sampler.next(), emPdf);
            const Emitter& em = getEmitterByID(id);
            v3f emCenter = scene.getShapeCenter(em.shapeID);
            float emRadius = scene.getShapeRadius(em.shapeID);

            //Samples the light
            v3f pShading = i.p;
            v3f wiW;
            float pdf;
            sampleSphereBySolidAngle(sampler.next2D(), pShading, emCenter, emRadius, wiW, pdf);

            //wi must be in local coordinates
            i.wi = i.frameNs.toLocal(wiW);

            //Creates a ray with the intersection point and the sampled direction
            Ray shadowRay(i.p, wiW);

            SurfaceInteraction iShadow;

            //Checks if there is no object intersection between the shading point and the light source
            if (scene.bvh->intersect(shadowRay, iShadow)) {
                emission = getEmission(iShadow);

                //Checks if the intersection lies on a surface emitter
                if(emission != v3f(0)){

                    v3f brdf = getBSDF(i) -> eval(i);

                    //Computes the value for the monte carlo estimator
                    Lr += emission * brdf / (pdf * emPdf);

                }
            }
        }
        Lr = Lr / m_emitterSamples;
        return Lr;
    }

    v3f renderMIS(const Ray& ray, Sampler& sampler) const {
        v3f Lr(0.f);
        v3f LrEm(0.f);
        v3f LrBsdf(0.f);

        SurfaceInteraction i;
        v3f emission;

        //Checks if ray intersects the scene (to display emitter in scene)
        if (scene.bvh->intersect(ray, i)) {

            emission = getEmission(i);

            //Checks if the intersection lands directly on an emitter
            if (emission != v3f(0)) {
                Lr = emission;
                return Lr;
            }
        }

        //PDF for the emitter
        float emPdf;

        for (int x = 0; x < m_emitterSamples; x++) {

            //Gets light and relevant data
            size_t id = selectEmitter(sampler.next(), emPdf);
            const Emitter &em = getEmitterByID(id);
            v3f emCenter = scene.getShapeCenter(em.shapeID);
            float emRadius = scene.getShapeRadius(em.shapeID);

            //Samples the light
            v3f pShading = i.p;
            v3f wiW;
            float emitPdf;
            sampleSphereBySolidAngle(sampler.next2D(), pShading, emCenter, emRadius, wiW, emitPdf);

            //wi must be in local coordinates
            i.wi = i.frameNs.toLocal(wiW);

            //Creates a ray with the intersection point and the sampled direction
            Ray shadowRay(i.p, wiW);

            SurfaceInteraction iShadow;

            //Checks if there is no object intersection between the shading point and the light source
            if (scene.bvh->intersect(shadowRay, iShadow)) {
                emission = getEmission(iShadow);

                //Checks if the intersection lies on a surface emitter
                if (emission != v3f(0)) {

                    v3f bsdf = getBSDF(i)->eval(i);

                    //Gets the pdf for the bsdf
                    float bsdfPdf = getBSDF(i)->pdf(i);

                    //Gets the weight for the sample
                    float weight = balanceHeuristic(m_emitterSamples, emPdf * emitPdf, m_bsdfSamples, bsdfPdf);

                    //Computes the value for the monte carlo estimator
                    LrEm += emission * bsdf * weight / (emitPdf * emPdf);

                }
            }
        }

        for (int x = 0; x < m_bsdfSamples; x++ ) {

            SurfaceInteraction i;

            //Checks if ray intersects the scene
            if (scene.bvh->intersect(ray, i)) {

                float bsdfPdf;
                v3f sample = getBSDF(i)->sample(i, sampler.next2D(), &bsdfPdf);

                //Converts to world view
                v3f dir = i.wi;
                dir = i.frameNs.toWorld(dir);
                dir = normalize(dir);

                //Creates a ray with the intersection point and the sampled direction
                Ray shadowRay(i.p, dir);

                SurfaceInteraction iShadow;

                //Checks if there is no object intersection
                if (scene.bvh->intersect(shadowRay, iShadow)) {
                    v3f emission = getEmission(iShadow);

                    //Checks if the intersection lies on a surface emitter
                    if(emission != v3f(0)){

                        //Gets light and relevant data
                        size_t id = getEmitterIDByShapeID(iShadow.shapeID);
                        const Emitter &em = getEmitterByID(id);
                        emPdf = getEmitterPdf(em);
                        v3f emCenter = scene.getShapeCenter(em.shapeID);
                        float emRadius = scene.getShapeRadius(em.shapeID);

                        //Calculates the subtended angle to find the pdf for the emitter sampling
                        float sinThetaMax = emRadius * emRadius / pow(glm::distance(i.p, emCenter), 2);
                        float cosThetaMax = std::sqrt(std::max(0.f, 1 - sinThetaMax));
                        float emitPdf = Warp::squareToUniformConePdf(cosThetaMax);

                        //Gets the weight for the sample
                        float weight = balanceHeuristic(m_bsdfSamples, bsdfPdf, m_emitterSamples, emitPdf * emPdf);

                        //Computes the value
                        LrBsdf += weight * emission * sample;

                    }
                }
            }
        }

        if (m_emitterSamples == 0) {
            Lr = LrBsdf / m_bsdfSamples;
        } else if(m_bsdfSamples == 0) {
            Lr = LrEm / m_emitterSamples;
        } else {
            Lr = LrBsdf / m_bsdfSamples + LrEm / m_emitterSamples;
        }

        return Lr;
    }

    v3f render(const Ray& ray, Sampler& sampler) const override {
        if (m_samplingStrategy == "mis")
            return this->renderMIS(ray, sampler);
        else if (m_samplingStrategy == "area")
            return this->renderArea(ray, sampler);
        else if (m_samplingStrategy == "solidAngle")
            return this->renderSolidAngle(ray, sampler);
        else if (m_samplingStrategy == "cosineHemisphere")
            return this->renderCosineHemisphere(ray, sampler);
        else if (m_samplingStrategy == "bsdf")
            return this->renderBSDF(ray, sampler);
        std::cout << "Error: wrong strategy" << std::endl;
        exit(EXIT_FAILURE);
    }

    size_t m_emitterSamples;     // Number of emitter samples
    size_t m_bsdfSamples;        // Number of BSDF samples
    string m_samplingStrategy;   // Sampling strategy to use
};

TR_NAMESPACE_END