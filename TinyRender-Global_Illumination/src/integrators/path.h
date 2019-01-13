/*
    This file is part of TinyRender, an educative rendering system.

    Designed for ECSE 446/546 Realistic/Advanced Image Synthesis.
    Derek Nowrouzezahrai, McGill University.
*/

#pragma once

TR_NAMESPACE_BEGIN

/**
 * Path tracer integrator
 */
struct PathTracerIntegrator : Integrator {
    explicit PathTracerIntegrator(const Scene& scene) : Integrator(scene) {
        m_isExplicit = scene.config.integratorSettings.pt.isExplicit;
        m_maxDepth = scene.config.integratorSettings.pt.maxDepth;
        m_rrDepth = scene.config.integratorSettings.pt.rrDepth;
        m_rrProb = scene.config.integratorSettings.pt.rrProb;
    }


    //with brdf sampling
    //each path should bounce m_maxDepth times away from the eye
    v3f renderImplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit) const {
        v3f Li(0.f);
        v3f throughput(1.f);
        int depth = 0;

        //checks if maxDepth was reached
        //if maxDepth = -1, the loop will be executed infinitely
        while(depth <= m_maxDepth || m_maxDepth < 0 ) {

            depth++;

            v3f emission = getEmission(hit);

            //Checks if the intersection lands directly on an emitter
            if (emission != v3f(0) && glm::dot(v3f(0.f, 0.f, 1.f), hit.wo) > 0) {
                Li += throughput * emission;
                return Li;
            }

            float pdf;
            v3f sample = getBSDF(hit)->sample(hit, sampler.next2D(), &pdf);
            throughput *= sample;
            //Converts to world view
            v3f dir = hit.wi;
            dir = hit.frameNs.toWorld(dir);
            dir = normalize(dir);

            //Creates a ray with the intersection point and the sampled direction
            Ray ray(hit.p, dir);

            //Checks if there is no object intersection
            if (!scene.bvh->intersect(ray, hit)) {
                    Li = v3f(0);
                    return Li;
            }
        }

        //No emitters hit before reaching maxDepth
        Li = v3f(0);
        return Li;
    }

    v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit2) const {
        v3f Li(0.f);
        v3f throughput(1.f);
        int depth = 0;
        SurfaceInteraction hit = hit2;
        /* -------------------------- EMITTER INTERSECTION --------------------- */
        v3f emission = getEmission(hit);

        //Checks if the intersection lands directly on an emitter
        if (emission != v3f(0)) {
            Li += throughput * emission;
            return Li;
        }
        /* --------------------------------------------------------------------- */

        //checks if maxDepth was reached
        //if maxDepth = -1, the loop will be executed infinitely
        while(depth < m_maxDepth || m_maxDepth < 0 ) {

            depth++;

            /* --------------------- SURFACE AREA SAMPLING ------------------------ */

            float emPdf;

            //Gets light and relevant data
            size_t id = selectEmitter(sampler.next(), emPdf);
            const Emitter &em = getEmitterByID(id);

            //Samples the light
            v3f n;
            v3f pos;
            v3f wiW;
            float pdf;
            sampleEmitterPosition(sampler, em, n, pos, pdf);
            wiW = pos - hit.p;
            wiW = normalize(wiW);

            //wi must be in local coordinates
            hit.wi = hit.frameNs.toLocal(wiW);

            //Creates a ray with the intersection point and the sampled direction
            Ray shadowRay(hit.p, wiW);

            SurfaceInteraction iShadow;

            //Checks if there is no object intersection between the shading point and the light source
            if (scene.bvh->intersect(shadowRay, iShadow)) {
                emission = getEmission(iShadow);

                //Checks if the intersection lies on a surface emitter
                if (emission != v3f(0)) {

                    //Computes the geometry term
                    float geom = glm::dot(-wiW, n) / glm::distance2(pos, hit.p);

                    //checks to see if sample was done in the right hemisphere
                    float vis = 1;
                    if (glm::dot(-wiW, n) < 0)
                        vis = 0;

                    v3f brdf = getBSDF(hit)->eval(hit);

                    //Computes the value for the monte carlo estimator
                    Li += throughput * vis * emission * brdf * geom / (pdf * emPdf);

                }
            }

            /* ---------------------------------------------------- ---------------- */

            /* -------------------------- BSDF SAMPLING ---------------------------- */

            //float pdf;
            v3f sample;
            SurfaceInteraction hitReSamp;
            hitReSamp = hit;
            emission = v3f(0.1);
            while(emission != v3f(0.f)) {

                sample = getBSDF(hit)->sample(hit, sampler.next2D(), &pdf);

                //Converts to world view
                v3f dir = hit.wi;
                dir = hit.frameNs.toWorld(dir);
                dir = glm::normalize(dir);

                //Creates a ray with the intersection point and the sampled direction
                Ray ray2(hit.p, dir);

                //Checks if there is no object intersection
                if (!scene.bvh->intersect(ray2, hitReSamp)) {
                    //Li = v3f(0);
                    return Li;
                }

                emission = getEmission(hitReSamp);
            }

            throughput *= sample;
            hit = hitReSamp;
            //Sets new hit point to be used for surface area sampling
            /* -------------------------------------------------------------------- */

            //Perform russian roulette
            if(depth >= m_rrDepth) {
                if(sampler.next() > m_rrProb)
                    return Li;
                throughput /= m_rrProb;
            }

        }

        //No emitters hit before reaching maxDepth
        return Li;
    }







//    static inline float misWeight(float fPdf, float gPdf) {
//        fPdf *= fPdf;
//        gPdf *= gPdf;
//        return fPdf / (fPdf + gPdf);
//    }
//
//    //WITH MIS
//    v3f renderExplicit(const Ray& ray, Sampler& sampler, SurfaceInteraction& hit2) const {
//        v3f Li(0.f);
//        v3f throughput(1.f);
//        int depth = 0;
//        SurfaceInteraction hit = hit2;
//        /* -------------------------- EMITTER INTERSECTION --------------------- */
//        v3f emission = getEmission(hit);
//
//        //Checks if the intersection lands directly on an emitter
//        if (emission != v3f(0)) {
//            Li += throughput * emission;
//            return Li;
//        }
//        /* --------------------------------------------------------------------- */
//        bool MIS = false;
//        //checks if maxDepth was reached
//        //if maxDepth = -1, the loop will be executed infinitely
//        while(depth < m_maxDepth || m_maxDepth < 0 ) {
//
//            depth++;
//
//            /* --------------------- SURFACE AREA SAMPLING ------------------------ */
//
//            float emPdf;
//
//            //Gets light and relevant data
//            size_t id = selectEmitter(sampler.next(), emPdf);
//            const Emitter &em = getEmitterByID(id);
//
//            //Samples the light
//            v3f n;
//            v3f pos;
//            v3f wiW;
//            float pdf;
//            sampleEmitterPosition(sampler, em, n, pos, pdf);
//            wiW = pos - hit.p;
//            wiW = normalize(wiW);
//
//            //wi must be in local coordinates
//            hit.wi = hit.frameNs.toLocal(wiW);
//
//            //Creates a ray with the intersection point and the sampled direction
//            Ray shadowRay(hit.p, wiW);
//
//            SurfaceInteraction iShadow;
//
//            //Checks if there is no object intersection between the shading point and the light source
//            if (scene.bvh->intersect(shadowRay, iShadow)) {
//                emission = getEmission(iShadow);
//
//                //Checks if the intersection lies on a surface emitter
//                if (emission != v3f(0)) {
//
//                    //Computes the geometry term
//                    float geom = glm::dot(-wiW, n) / glm::distance2(pos, hit.p);
//
//                    //checks to see if sample was done in the right hemisphere
//                    float vis = 1;
//                    if (glm::dot(-wiW, n) < 0)
//                        vis = 0;
//
//                    //Gets the pdf for the bsdf
//                    float bsdfPdf = getBSDF(hit)->pdf(hit);
//
//                    v3f brdf = getBSDF(hit)->eval(hit);
//
//                    float miWeight = misWeight(pdf * emPdf * 1/geom, bsdfPdf);
//
//                    //Computes the value for the monte carlo estimator
//                    Li +=  miWeight * throughput * vis * emission * brdf * geom / (pdf * emPdf);
//                }
//            }
//
//            /* ---------------------------------------------------- ---------------- */
////            if(depth == m_maxDepth)
////                return Li;
//            /* -------------------------- BSDF SAMPLING ---------------------------- */
//
//            float BSDFpdf;
//            v3f sample;
//            SurfaceInteraction hitReSamp = hit;
////              hitReSamp = hit;
////            emission = v3f(0.1);
////            while(emission != v3f(0.f)) {
//
//                sample = getBSDF(hit)->sample(hit, sampler.next2D(), &BSDFpdf);
//                throughput *= sample;
//
//                //Converts to world view
//                v3f dir = hit.wi;
//                dir = hit.frameNs.toWorld(dir);
//                dir = glm::normalize(dir);
//
//                //Creates a ray with the intersection point and the sampled direction
//                Ray ray2(hit.p, dir);
//
//                //Checks if there is no object intersection
////                if (!scene.bvh->intersect(ray2, hitReSamp)) {
//                if (!scene.bvh->intersect(ray2, hitReSamp)) {
//                    //Li = v3f(0);
//                    return Li;
//                }
//
//                emission = getEmission(hitReSamp);
//                if(emission != v3f(0.f)){
//                    //Gets light and relevant data
//                    size_t id = getEmitterIDByShapeID(hitReSamp.shapeID);
//                    const Emitter &em = getEmitterByID(id);
//                    emPdf = getEmitterPdf(em);
//
////                  //Samples the light
//                    sampleEmitterPosition(sampler, em, n, pos, pdf);
//                    wiW =  pos - hit.p;
//                    wiW = normalize(wiW);
//                    float emitPdf = pdf;
//
//                    float geom = glm::dot(-wiW, n) / glm::distance2(pos, hit.p);
//                    geom = 1/geom;
//                    float miWeight = misWeight(BSDFpdf, emitPdf * emPdf * geom);
////                    if(BSDFpdf + (emitPdf * emPdf) > 0)
//                    Li += throughput * emission * miWeight;
////                    Li += throughput * emission;
//
//                hit = hitReSamp;
////            }
//
//
//            //Sets new hit point to be used for surface area sampling
//            /* -------------------------------------------------------------------- */
//
//            //Perform russian roulette
//            if(depth >= m_rrDepth) {
//                if(sampler.next() > m_rrProb)
//                    return Li;
//                throughput /= m_rrProb;
//            }
//
//        }
//
//        //No emitters hit before reaching maxDepth
//        return Li;
//    }

    v3f render(const Ray& ray, Sampler& sampler) const override {
        Ray r = ray;
        SurfaceInteraction hit;

        if (scene.bvh->intersect(r, hit)) {
            if (m_isExplicit)
                return this->renderExplicit(ray, sampler, hit);
            else
                return this->renderImplicit(ray, sampler, hit);
        }
        return v3f(0.0);
    }

    int m_maxDepth;     // Maximum number of bounces
    int m_rrDepth;      // When to start Russian roulette
    float m_rrProb;     // Russian roulette probability
    bool m_isExplicit;  // Implicit or explicit
};

TR_NAMESPACE_END