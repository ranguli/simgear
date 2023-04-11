// Copyright (C) 2018 - 2023 Fernando García Liñán
// SPDX-License-Identifier: LGPL-2.0-or-later

#pragma once

#include <osgDB/ReaderWriter>
#include <osg/Group>

#include <simgear/props/props.hxx>
#include <simgear/structure/SGExpression.hxx>

class SGLight : public osg::Node {
public:
    enum Type {
        POINT,
        SPOT
    };

    enum Priority {
        HIGH,
        MEDIUM,
        LOW
    };

    class UpdateCallback : public osg::NodeCallback {
    public:
        UpdateCallback(const SGExpressiond *expression) : _expression(expression) {}
        UpdateCallback() {}

        void configure(const osg::Vec4& ambient, const osg::Vec4& diffuse, const osg::Vec4& specular);

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override;

    private:
        SGSharedPtr<SGExpressiond const> _expression {nullptr};
        osg::Vec4 _ambient, _diffuse, _specular;
        double _prev_value = -1.0;
    };

    SGLight();
    SGLight(const SGLight& l, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);
    SGLight(osg::MatrixTransform *transform) : SGLight() { _transform = transform; };

    META_Node(simgear, SGLight);

    static osg::ref_ptr<osg::Node> appendLight(const SGPropertyNode* configNode,
                                               SGPropertyNode* modelRoot,
                                               const osgDB::Options* options);

    void configure(const SGPropertyNode *config_root);

    void setType(Type type) { _type = type; }
    Type getType() const { return _type; }

    void setRange(float range) { _range = range; }
    float getRange() const { return _range; }

    void setAmbient(const osg::Vec4 &ambient) { _ambient = ambient; }
    const osg::Vec4 &getAmbient() const { return _ambient; }

    void setDiffuse(const osg::Vec4 &diffuse) { _diffuse = diffuse; }
    const osg::Vec4 &getDiffuse() const { return _diffuse; }

    void setSpecular(const osg::Vec4 &specular) { _specular = specular; }
    const osg::Vec4 &getSpecular() const { return _specular; }

    void setConstantAttenuation(float constant_attenuation) { _constant_attenuation = constant_attenuation; }
    float getConstantAttenuation() const { return _constant_attenuation; }

    void setLinearAttenuation(float linear_attenuation) { _linear_attenuation = linear_attenuation; }
    float getLinearAttenuation() const { return _linear_attenuation; }

    void setQuadraticAttenuation(float quadratic_attenuation) { _quadratic_attenuation = quadratic_attenuation; }
    float getQuadraticAttenuation() const { return _quadratic_attenuation; }

    void setSpotExponent(float spot_exponent) { _spot_exponent = spot_exponent; }
    float getSpotExponent() const { return _spot_exponent; }

    void setSpotCutoff(float spot_cutoff) { _spot_cutoff = spot_cutoff; }
    float getSpotCutoff() const { return _spot_cutoff; }

    void setPriority(Priority priority) { _priority = priority; }
    Priority getPriority() const { return _priority; }

    void setColor(const osg::Vec3 &color) { _color = color; }
    const osg::Vec3 &getColor() const { return _color; }

    void setIntensity(float intensity) { _intensity = intensity; }
    float getIntensity() const { return _intensity; }

protected:
    virtual ~SGLight();

    Type _type {Type::POINT};

    float _range {0.0f};

    osg::Vec4 _ambient;
    osg::Vec4 _diffuse;
    osg::Vec4 _specular;

    float _constant_attenuation {1.0f};
    float _linear_attenuation {0.0f};
    float _quadratic_attenuation {0.0f};
    float _spot_exponent {0.0f};
    float _spot_cutoff {180.0f};
    Priority _priority {Priority::LOW};
    osg::MatrixTransform *_transform {nullptr};

    // Physically-based parameters. These are an alternative to the classic
    // ambient/diffuse/specular scheme.

    // Color of emitted light, as a linear sRGB color
    osg::Vec3 _color;
    // The light's brightness. The unit depends on the renderer
    float _intensity {1.0f};
};

typedef std::vector<osg::ref_ptr<SGLight>> SGLightList;
