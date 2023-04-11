// Copyright (C) 2018 - 2023 Fernando García Liñán
// SPDX-License-Identifier: LGPL-2.0-or-later

#include <simgear/props/condition.hxx>
#include <simgear/props/props.hxx>
#include <simgear/scene/tgdb/userdata.hxx>

#include "CompositorUtil.hxx"

namespace simgear {
namespace compositor {

bool
checkConditional(const SGPropertyNode *node)
{
    const SGPropertyNode *p_condition = node->getChild("condition");
    if (!p_condition)
        return true;
    SGSharedPtr<SGCondition> condition =
        sgReadCondition(getPropertyRoot(), p_condition);
    return !condition || condition->test();
}

const SGPropertyNode *
getPropertyNode(const SGPropertyNode *prop)
{
    if (!prop)
        return 0;
    if (prop->nChildren() > 0) {
        const SGPropertyNode *propertyProp = prop->getChild("property");
        if (!propertyProp)
            return prop;
        return getPropertyRoot()->getNode(propertyProp->getStringValue());
    }
    return prop;
}

const SGPropertyNode *
getPropertyChild(const SGPropertyNode *prop,
                 const char *name)
{
    const SGPropertyNode *child = prop->getChild(name);
    if (!child)
        return 0;
    return getPropertyNode(child);
}

void makeNewProjMat(osg::Matrixd& oldProj, double znear,
                    double zfar, osg::Matrixd& projection) {
    projection = oldProj;
    // Slightly inflate the near & far planes to avoid objects at the
    // extremes being clipped out.
    znear *= 0.999;
    zfar *= 1.001;

    // Clamp the projection matrix z values to the range (near, far)
    double epsilon = 1.0e-6;
    if (fabs(projection(0,3)) < epsilon &&
        fabs(projection(1,3)) < epsilon &&
        fabs(projection(2,3)) < epsilon) {
        // Projection is Orthographic
        epsilon = -1.0/(zfar - znear); // Used as a temp variable
        projection(2,2) = 2.0*epsilon;
        projection(3,2) = (zfar + znear)*epsilon;
    } else {
        // Projection is Perspective
        double trans_near = (-znear*projection(2,2) + projection(3,2)) /
            (-znear*projection(2,3) + projection(3,3));
        double trans_far = (-zfar*projection(2,2) + projection(3,2)) /
            (-zfar*projection(2,3) + projection(3,3));
        double ratio = fabs(2.0/(trans_near - trans_far));
        double center = -0.5*(trans_near + trans_far);

        projection.postMult(osg::Matrixd(1.0, 0.0, 0.0, 0.0,
                                         0.0, 1.0, 0.0, 0.0,
                                         0.0, 0.0, ratio, 0.0,
                                         0.0, 0.0, center*ratio, 1.0));
    }
}

} // namespace compositor
} // namespace simgear
