// ReaderWriterSTG.cxx -- routines to handle a scenery tile
//
// Written by Curtis Olson, started May 1998.
//
// Copyright (C) 1998 - 2001  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#ifdef HAVE_CONFIG_H
#  include <simgear_config.h>
#endif
#include <algorithm>
#include "ReaderWriterSTG.hxx"

#include <osg/LOD>
#include <osg/MatrixTransform>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>

#include <simgear/bucket/newbucket.hxx>
#include <simgear/debug/ErrorReportingCallback.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/math/SGGeometry.hxx>
#include <simgear/math/sg_random.hxx>

#include <simgear/io/iostreams/sgstream.hxx>
#include <simgear/scene/material/matlib.hxx>
#include <simgear/scene/tgdb/LightBin.hxx>
#include <simgear/scene/tgdb/ObjectInstanceBin.hxx>
#include <simgear/scene/tgdb/SGBuildingBin.hxx>
#include <simgear/scene/tgdb/TreeBin.hxx>
#include <simgear/scene/tgdb/VPBRasterRenderer.hxx>
#include <simgear/scene/tgdb/VPBTechnique.hxx>
#include <simgear/scene/tgdb/apt_signs.hxx>
#include <simgear/scene/tgdb/obj.hxx>
#include <simgear/scene/util/OptionsReadFileCallback.hxx>
#include <simgear/scene/util/OsgMath.hxx>
#include <simgear/scene/util/QuadTreeBuilder.hxx>
#include <simgear/scene/util/RenderConstants.hxx>
#include <simgear/scene/util/SGReaderWriterOptions.hxx>

#include <simgear/scene/util/SGSceneFeatures.hxx>

#include "SGOceanTile.hxx"

#define BUILDING_ROUGH "OBJECT_BUILDING_MESH_ROUGH"
#define BUILDING_DETAILED "OBJECT_BUILDING_MESH_DETAILED"
#define ROAD_ROUGH "OBJECT_ROAD_ROUGH"
#define ROAD_DETAILED "OBJECT_ROAD_DETAILED"
#define RAILWAY_ROUGH "OBJECT_RAILWAY_ROUGH"
#define RAILWAY_DETAILED "OBJECT_RAILWAY_DETAILED"
#define BUILDING_LIST "BUILDING_LIST"
#define TREE_LIST "TREE_LIST"
#define LINE_FEATURE_LIST "LINE_FEATURE_LIST"
#define AREA_FEATURE_LIST "AREA_FEATURE_LIST"
#define COASTLINE_LIST "COASTLINE_LIST"
#define OBJECT_LIGHT "OBJECT_LIGHT"
#define LIGHT_LIST "LIGHT_LIST"
#define OBJECT_INSTANCED "OBJECT_INSTANCED"

namespace simgear {

/// Ok, this is a hack - we do not exactly know if it's an airport or not.
/// This feature might also vanish again later. This is currently to
/// support testing an external ai component that just loads the the airports
/// and supports ground queries on only these areas.
static bool isAirportBtg(const std::string& name)
{
    if (name.size() < 8)
        return false;
    if (name.substr(4, 4) != ".btg")
        return false;
    for (unsigned i = 0; i < 4; ++i) {
        if ('A' <= name[i] && name[i] <= 'Z')
            continue;
        return false;
    }
    return true;
}

static SGBucket bucketIndexFromFileName(const std::string& fileName)
{
  // Extract the bucket from the filename
  std::istringstream ss(osgDB::getNameLessExtension(fileName));
  long index;
  ss >> index;
  if (ss.fail())
    return SGBucket();

  return SGBucket(index);
}

/**
 * callback per STG token, with access synced by a lock.
 */
using TokenCallbackMap = std::map<std::string,ReaderWriterSTG::STGObjectCallback>;
static TokenCallbackMap globalStgObjectCallbacks = {};
static OpenThreads::Mutex globalStgObjectCallbackLock;

struct ReaderWriterSTG::_ModelBin {
    struct _Object {
        SGPath _errorLocation;
        std::string _token;
        std::string _name;
        osg::ref_ptr<SGReaderWriterOptions> _options;
    };
    struct _ObjectStatic {
        _ObjectStatic() : _agl(false), _proxy(false), _lon(0), _lat(0), _elev(0), _hdg(0), _pitch(0), _roll(0), _range(SG_OBJECT_RANGE_ROUGH), _radius(10) { }
        SGPath _errorLocation;
        std::string _token;
        std::string _name;
        bool _agl;
        bool _proxy;
        double _lon, _lat, _elev;
        double _hdg, _pitch, _roll;
        double _range, _radius;
        osg::ref_ptr<SGReaderWriterOptions> _options;
    };
    struct _Sign {
        _Sign() : _agl(false), _lon(0), _lat(0), _elev(0), _hdg(0), _size(-1) { }
        SGPath _errorLocation;
        std::string _token;
        std::string _name;
        bool _agl;
        double _lon, _lat, _elev;
        double _hdg;
        int _size;
    };
    struct _BuildingList {
      _BuildingList() : _lon(0), _lat(0), _elev(0) { }
      std::string _filename;
      std::string _material_name;
      double _lon, _lat, _elev;
    };
    struct _TreeList {
      _TreeList() : _lon(0), _lat(0), _elev(0) { }
      std::string _filename;
      std::string _material_name;
      double _lon, _lat, _elev;
    };
    struct _Light {
        _Light() : _lon(0), _lat(0), _elev(0), _size(0), _intensity(0), _on_period(0), _horizontal_angle(0), _vertical_angle(0) { }
        double _lon, _lat, _elev;
        double _size, _intensity;
        int _on_period;
        SGVec4f _color;
        SGVec3f _direction;
        double _horizontal_angle, _vertical_angle;
        SGVec4f _animation_params;
    };
    struct _LightList {
      _LightList() : _lon(0), _lat(0), _elev(0) { }
      std::string _filename;
      double _lon, _lat, _elev;
    };

    struct _InstancedObject {
        _InstancedObject() : _lon(0), _lat(0), _elev(0) {}
        SGPath _errorLocation;
        std::string _modelname;
        std::string _filename;
        std::string _effect;
        double _lon, _lat, _elev;
        osg::ref_ptr<SGReaderWriterOptions> _options;
    };

    struct _LineFeatureList {
      _LineFeatureList() { }
      std::string _filename;
      std::string _material;
      SGBucket _bucket;
    };
    struct _AreaFeatureList {
      _AreaFeatureList() { }
      std::string _filename;
      std::string _material;
      SGBucket _bucket;
    };

    struct _CoastlineList {
      _CoastlineList() { }
      std::string _filename;
      SGBucket _bucket;
    };

    class DelayLoadReadFileCallback : public OptionsReadFileCallback {

    private:
      // QuadTreeBuilder for structuring static objects
      struct MakeQuadLeaf {
          osg::LOD* operator() () const { return new osg::LOD; }
      };
      struct AddModelLOD {
          void operator() (osg::LOD* leaf, _ObjectStatic o) const
          {
            osg::ref_ptr<osg::Node> node;
            if (o._proxy)  {
                osg::ref_ptr<osg::ProxyNode> proxy = new osg::ProxyNode;
                proxy->setName("proxyNode");
                proxy->setLoadingExternalReferenceMode(osg::ProxyNode::DEFER_LOADING_TO_DATABASE_PAGER);
                proxy->setFileName(0, o._name);
                proxy->setDatabaseOptions(o._options);

                // Give the node some values so the Quadtree builder has
                // a BoundingBox to work with prior to the model being loaded.
                proxy->setCenter(osg::Vec3f(0.0f,0.0f,0.0f));
                proxy->setRadius(o._radius);
                proxy->setCenterMode(osg::ProxyNode::UNION_OF_BOUNDING_SPHERE_AND_USER_DEFINED);
                node = proxy;
            } else {
                ErrorReportContext ec("terrain-stg", o._errorLocation.utf8Str());
                node = osgDB::readRefNodeFile(o._name, o._options.get());
                if (!node.valid()) {
                    SG_LOG(SG_TERRAIN, SG_ALERT, o._errorLocation << ": Failed to load "
                           << o._token << " '" << o._name << "'");
                    return;
                }
            }
            if (SGPath(o._name).lower_extension() == "ac")
                node->setNodeMask(~simgear::MODELLIGHT_BIT);

            osg::Matrix matrix;
            matrix = makeZUpFrame(SGGeod::fromDegM(o._lon, o._lat, o._elev));
            matrix.preMultRotate(osg::Quat(SGMiscd::deg2rad(o._hdg), osg::Vec3(0, 0, 1)));
            matrix.preMultRotate(osg::Quat(SGMiscd::deg2rad(o._pitch), osg::Vec3(0, 1, 0)));
            matrix.preMultRotate(osg::Quat(SGMiscd::deg2rad(o._roll), osg::Vec3(1, 0, 0)));

            osg::MatrixTransform* matrixTransform;
            matrixTransform = new osg::MatrixTransform(matrix);
            matrixTransform->setName("rotateStaticObject");
            matrixTransform->setDataVariance(osg::Object::STATIC);
            matrixTransform->addChild(node.get());

            leaf->addChild(matrixTransform, 0, o._range);
          }
      };
      struct GetModelLODCoord {
          GetModelLODCoord() {}
          GetModelLODCoord(const GetModelLODCoord& rhs)
          {}
          osg::Vec3 operator() (const _ObjectStatic& o) const
          {
              SGVec3d coord;
              SGGeodesy::SGGeodToCart(SGGeod::fromDegM(o._lon, o._lat, o._elev), coord);
              return toOsg(coord);
          }
      };
      typedef QuadTreeBuilder<osg::LOD*, _ObjectStatic, MakeQuadLeaf, AddModelLOD,
                              GetModelLODCoord>  STGObjectsQuadtree;
    public:
        virtual osgDB::ReaderWriter::ReadResult
        readNode(const std::string&, const osgDB::Options*)
        {
            ErrorReportContext ec("terrain-bucket", _bucket.gen_index_str());

            STGObjectsQuadtree quadtree((GetModelLODCoord()), (AddModelLOD()));
            quadtree.buildQuadTree(_objectStaticList.begin(), _objectStaticList.end());
            osg::ref_ptr<osg::Group> group = quadtree.getRoot();
            string group_name = string("STG-group-A ").append(_bucket.gen_index_str());
            group->setName(group_name);
            group->setDataVariance(osg::Object::STATIC);

            simgear::AirportSignBuilder signBuilder(_options->getMaterialLib(), _bucket.get_center());
            for (std::list<_Sign>::iterator i = _signList.begin(); i != _signList.end(); ++i)
                signBuilder.addSign(SGGeod::fromDegM(i->_lon, i->_lat, i->_elev), i->_hdg, i->_name, i->_size);
            if (signBuilder.getSignsGroup())
                group->addChild(signBuilder.getSignsGroup());

            if (!_buildingList.empty()) {
                SGMaterialLibPtr matlib = _options->getMaterialLib();
                bool useVBOs = (_options->getPluginStringData("SimGear::USE_VBOS") == "ON");

                if (!matlib) {
                    SG_LOG( SG_TERRAIN, SG_ALERT, "Unable to get materials definition for buildings");
                } else {
                    for (const auto& b : _buildingList) {
                        // Build buildings for each list of buildings
                        SGGeod geodPos = SGGeod::fromDegM(b._lon, b._lat, 0.0);
                        SGSharedPtr<SGMaterial> mat = matlib->find(b._material_name, geodPos);

                        // trying to avoid crash on null material, see:
                        // https://sentry.io/organizations/flightgear/issues/1867075869
                        if (!mat) {
                            SG_LOG(SG_TERRAIN, SG_ALERT, "Building specifies unknown material: " << b._material_name);
                            continue;
                        }

                        const auto path = SGPath(b._filename);
                        SGBuildingBin* buildingBin = new SGBuildingBin(path, mat, useVBOs);

                        SGBuildingBinList bbList;
                        bbList.push_back(buildingBin);

                        osg::MatrixTransform* matrixTransform;
                        matrixTransform = new osg::MatrixTransform(makeZUpFrame(SGGeod::fromDegM(b._lon, b._lat, b._elev)));
                        matrixTransform->setName("rotateBuildings");
                        matrixTransform->setDataVariance(osg::Object::STATIC);
                        matrixTransform->addChild(createRandomBuildings(bbList, osg::Matrix::identity(), _options));
                        group->addChild(matrixTransform);

                        std::for_each(bbList.begin(), bbList.end(), [](SGBuildingBin* bb) {
                            delete bb;
                        });
                    }
                }
            }

            if (!_treeList.empty()) {
                SGMaterialLibPtr matlib = _options->getMaterialLib();

                if (!matlib) {
                    SG_LOG( SG_TERRAIN, SG_ALERT, "Unable to get materials definition for buildings");
                } else {
                    for (const auto& b : _treeList) {
                        // Build trees for each list of trees
                        SGGeod geodPos = SGGeod::fromDegM(b._lon, b._lat, 0.0);
                        SGSharedPtr<SGMaterial> mat = matlib->find(b._material_name, geodPos);

                        // trying to avoid crash on null material, see:
                        // https://sentry.io/organizations/flightgear/issues/1867075869
                        if (!mat) {
                            SG_LOG(SG_TERRAIN, SG_ALERT, "Tree list specifies unknown material: " << b._filename << " " << b._material_name);
                            continue;
                        }

                        const auto path = SGPath(b._filename);
                        TreeBin* treeBin = new TreeBin(path, mat);

                        SGTreeBinList treeList;
                        treeList.push_back(treeBin);

                        osg::MatrixTransform* matrixTransform;
                        matrixTransform = new osg::MatrixTransform(makeZUpFrame(SGGeod::fromDegM(b._lon, b._lat, b._elev)));
                        matrixTransform->setName("rotateTrees");
                        matrixTransform->setDataVariance(osg::Object::STATIC);
                        matrixTransform->addChild(createForest(treeList, osg::Matrix::identity(), _options));
                        group->addChild(matrixTransform);

                        std::for_each(treeList.begin(), treeList.end(), [](TreeBin* bb) {
                            delete bb;
                        });
                    }
                }
            }

            // Add lights
            if (!_lightList.empty()) {
                // Transform lights frame of ref for better precision
                osg::MatrixTransform* matrixTransform;
                matrixTransform = new osg::MatrixTransform(makeZUpFrame(_bucket.get_center()));
                matrixTransform->setName("rotateLights");
                matrixTransform->setDataVariance(osg::Object::STATIC);

                LightBin lightList;
                for (const auto& light : _lightList) {
                    osg::Matrix _position_frame(makeZUpFrame(SGGeod::fromDegM(light._lon, light._lat, light._elev)));
                    SGVec3f _position(_position_frame(3,0), _position_frame(3,1), _position_frame(3,2));

                    lightList.insert(
                        _position,
                        light._size, light._intensity,
                        light._on_period,
                        light._color,
                        light._direction,
                        light._horizontal_angle, light._vertical_angle,
                        light._animation_params
                    );
                }

                matrixTransform->addChild(createLights(lightList, matrixTransform->getInverseMatrix(), _options));
                group->addChild(matrixTransform);
            }

            if (!_lightListList.empty()) {
                for (const auto& ll : _lightListList) {
                    osg::MatrixTransform* matrixTransform;
                    matrixTransform = new osg::MatrixTransform(makeZUpFrame(SGGeod::fromDegM(ll._lon, ll._lat, ll._elev)));
                    matrixTransform->setName("rotateLights");
                    matrixTransform->setDataVariance(osg::Object::STATIC);

                    const auto path = SGPath(ll._filename);
                    LightBin lightList(path);

                    matrixTransform->addChild(createLights(lightList, osg::Matrix::identity(), _options));
                    group->addChild(matrixTransform);
                }
            }

            if (!_instancedObjectList.empty()) {
                for (const auto& io : _instancedObjectList) {
                    osg::MatrixTransform* matrixTransform;
                    matrixTransform = new osg::MatrixTransform(makeZUpFrame(SGGeod::fromDegM(io._lon, io._lat, io._elev)));
                    matrixTransform->setName("rotateInstancedObject");
                    matrixTransform->setDataVariance(osg::Object::STATIC);

                    const auto path = SGPath(io._filename);
                    ObjectInstanceBin objectInstances(io._modelname, path, io._effect);

                    const auto loadedModelRename = createObjectInstances(objectInstances, osg::Matrix::identity(), io._options);
                    if (loadedModelRename) {
                        matrixTransform->addChild(loadedModelRename);
                        group->addChild(matrixTransform);
                    }
                }
            }

            return group.release();
        }

        mt _seed;
        std::list<_ObjectStatic> _objectStaticList;
        std::list<_Sign> _signList;
        std::list<_BuildingList> _buildingList;
        std::list<_TreeList> _treeList;
        std::list<_Light> _lightList;
        std::list<_LightList> _lightListList;
        std::list<_InstancedObject> _instancedObjectList;

        /// The original options to use for this bunch of models
        osg::ref_ptr<SGReaderWriterOptions> _options;
        SGBucket _bucket;
    };

    _ModelBin() :
        _object_range_bare(SG_OBJECT_RANGE_BARE),
        _object_range_rough(SG_OBJECT_RANGE_ROUGH),
        _object_range_detailed(SG_OBJECT_RANGE_DETAILED),
        _foundBase(false)
    { }

    SGReaderWriterOptions* sharedOptions(const std::string& filePath, const osgDB::Options* options)
    {
        osg::ref_ptr<SGReaderWriterOptions> sharedOptions;
        sharedOptions = SGReaderWriterOptions::copyOrCreate(options);
        sharedOptions->getDatabasePathList().clear();

        SGPath path = filePath;
        path.append(".."); path.append(".."); path.append("..");
        sharedOptions->getDatabasePathList().push_back(path.utf8Str());

        // ensure Models directory synced via TerraSync is searched before the copy in
        // FG_ROOT, so that updated models can be used.
        std::string terrasync_root = options->getPluginStringData("SimGear::TERRASYNC_ROOT");
        if (!terrasync_root.empty()) {
            sharedOptions->getDatabasePathList().push_back(terrasync_root);
        }

        std::string fg_root = options->getPluginStringData("SimGear::FG_ROOT");
        sharedOptions->getDatabasePathList().push_back(fg_root);

        // TODO how should we handle this for OBJECT_SHARED?
        sharedOptions->setModelData
        (
            sharedOptions->getModelData()
          ? sharedOptions->getModelData()->clone()
          : 0
        );

        return sharedOptions.release();
    }
    SGReaderWriterOptions* staticOptions(const std::string& filePath, const osgDB::Options* options)
    {
        osg::ref_ptr<SGReaderWriterOptions> staticOptions;
        staticOptions = SGReaderWriterOptions::copyOrCreate(options);
        staticOptions->getDatabasePathList().clear();

        staticOptions->getDatabasePathList().push_back(filePath);
        staticOptions->setObjectCacheHint(osgDB::Options::CACHE_NONE);

        // Every model needs its own SGModelData to ensure load/unload is
        // working properly
        staticOptions->setModelData
        (
            staticOptions->getModelData()
          ? staticOptions->getModelData()->clone()
          : 0
        );

        return staticOptions.release();
    }

    double elevation(osg::Group& group, const SGGeod& geod)
    {
        SGVec3d start = SGVec3d::fromGeod(SGGeod::fromGeodM(geod, 10000));
        SGVec3d end = SGVec3d::fromGeod(SGGeod::fromGeodM(geod, -1000));

        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
        intersector = new osgUtil::LineSegmentIntersector(toOsg(start), toOsg(end));
        osgUtil::IntersectionVisitor visitor(intersector.get());
        group.accept(visitor);

        if (!intersector->containsIntersections())
            return 0;

        SGVec3d cart = toSG(intersector->getFirstIntersection().getWorldIntersectPoint());
        return SGGeod::fromCart(cart).getElevationM();
    }

    void checkInsideBucket(const SGPath& absoluteFileName, float lon, float lat) {
        SGBucket bucket = bucketIndexFromFileName(absoluteFileName.file_base().c_str());
        SGBucket correctBucket = SGBucket( SGGeod::fromDeg(lon, lat));

        if (bucket != correctBucket) {
          SG_LOG( SG_TERRAIN, SG_DEV_WARN, absoluteFileName
                  << ": Object at " << lon << ", " << lat <<
                  " in incorrect bucket (" << bucket << ") - should be in " <<
                  correctBucket.gen_index_str() << " (" << correctBucket << ")");
        }
    }

    bool read(const SGPath& absoluteFileName, const osgDB::Options* options)
    {
        if (!absoluteFileName.exists()) {
            return false;
        }

        sg_gzifstream stream(absoluteFileName);
        if (!stream.is_open()) {
            return false;
        }

        // starting with 2018.3 we will use deltas rather than absolutes as it is more intuitive for the user
        // and somewhat easier to visualise
        double detailedRange = atof(options->getPluginStringData("SimGear::LOD_RANGE_DETAILED").c_str());
        double bareRangeDelta = atof(options->getPluginStringData("SimGear::LOD_RANGE_BARE").c_str());
        double roughRangeDelta = atof(options->getPluginStringData("SimGear::LOD_RANGE_ROUGH").c_str());

        _object_range_detailed = detailedRange;
        _object_range_bare = _object_range_detailed + bareRangeDelta;
        _object_range_rough = _object_range_detailed + roughRangeDelta;

        SG_LOG(SG_TERRAIN, SG_INFO, "Loading stg file " << absoluteFileName);

        std::string filePath = osgDB::getFilePath(absoluteFileName.utf8Str());

        // Bucket provides a consistent seed
        // so we have consistent set of pseudo-random numbers for each STG file
        pc_init(std::stoi(absoluteFileName.file_base()));

        bool vpb_active = SGSceneFeatures::instance()->getVPBActive();

        // do only load airport btg files.
        bool onlyAirports = options->getPluginStringData("SimGear::FG_ONLY_AIRPORTS") == "ON";
        // do only load terrain btg files
        bool onlyTerrain = options->getPluginStringData("SimGear::FG_ONLY_TERRAIN") == "ON";

        while (!stream.eof()) {
            // read a line
            std::string line;
            std::getline(stream, line);

            // strip comments
            std::string::size_type hash_pos = line.find('#');
            if (hash_pos != std::string::npos)
                line.resize(hash_pos);

            // and process further
            std::stringstream in(line);

            std::string token;
            in >> token;

            // No comment
            if (token.empty())
                continue;

            // Then there is always a name
            std::string name;
            in >> name;

            SGPath path = filePath;
            path.append(name);

            if (token == "OBJECT_BASE") {
                if (!vpb_active) {
                    // Load only once (first found)
                    SG_LOG( SG_TERRAIN, SG_BULK, "    " << token << " " << name );
                    _foundBase = true;
                    if (!onlyAirports || isAirportBtg(name)) {
                        _Object obj;
                        obj._errorLocation = absoluteFileName;
                        obj._token = token;
                        obj._name = path.utf8Str();
                        obj._options = staticOptions(filePath, options);
                        _objectList.push_back(obj);
                    }
                }
            } else if (token == "OBJECT") {
                if (!onlyAirports || isAirportBtg(name)) {
                    _Object obj;
                    obj._errorLocation = absoluteFileName;
                    obj._token = token;
                    obj._name = path.utf8Str();
                    obj._options = staticOptions(filePath, options);
                    _objectList.push_back(obj);
                }
            } else if (!onlyTerrain) {
                // Load non-terrain objects

                // Determine an appropriate range for the object, which has some randomness
                double range = _object_range_rough;
                double lrand = pc_rand();
                if      (lrand < 0.1) range = range * 2.0;
                else if (lrand < 0.4) range = range * 1.5;

                if (token == "OBJECT_STATIC" || token == "OBJECT_STATIC_AGL") {
                    osg::ref_ptr<SGReaderWriterOptions> opt;
                    opt = staticOptions(filePath, options);
                    if (SGPath(name).lower_extension() == "ac")
                            opt->setInstantiateEffects(true);
                    else
                            opt->setInstantiateEffects(false);
                    _ObjectStatic obj;

                    opt->addErrorContext("terrain-stg", absoluteFileName.utf8Str());
                    obj._errorLocation = absoluteFileName;
                    obj._token = token;
                    obj._name = name;
                    obj._agl = (token == "OBJECT_STATIC_AGL");
                    obj._proxy = true;
                    in >> obj._lon >> obj._lat >> obj._elev >> obj._hdg >> obj._pitch >> obj._roll >> obj._radius;
                    obj._range = range;
                    obj._options = opt;
                    checkInsideBucket(absoluteFileName, obj._lon, obj._lat);
                    _objectStaticList.push_back(obj);
                } else if (token == "OBJECT_SHARED" || token == "OBJECT_SHARED_AGL") {
                    osg::ref_ptr<SGReaderWriterOptions> opt;
                    opt = sharedOptions(filePath, options);
                    if (SGPath(name).lower_extension() == "ac")
                            opt->setInstantiateEffects(true);
                    else
                            opt->setInstantiateEffects(false);
                    _ObjectStatic obj;
                    obj._errorLocation = absoluteFileName;
                    obj._token = token;
                    obj._name = name;
                    obj._agl = (token == "OBJECT_SHARED_AGL");
                    obj._proxy = false;
                    in >> obj._lon >> obj._lat >> obj._elev >> obj._hdg >> obj._pitch >> obj._roll >> obj._radius;
                    obj._range = range;
                    obj._options = opt;
                    checkInsideBucket(absoluteFileName, obj._lon, obj._lat);
                    _objectStaticList.push_back(obj);
                } else if (token == "OBJECT_SIGN" || token == "OBJECT_SIGN_AGL") {
                    _Sign sign;
                    sign._token = token;
                    sign._name = name;
                    sign._agl = (token == "OBJECT_SIGN_AGL");
                    in >> sign._lon >> sign._lat >> sign._elev >> sign._hdg >> sign._size;
                    _signList.push_back(sign);
                } else if (token == BUILDING_ROUGH || token == BUILDING_DETAILED ||
                           token == ROAD_ROUGH     || token == ROAD_DETAILED     ||
                           token == RAILWAY_ROUGH  || token == RAILWAY_DETAILED)   {
                    osg::ref_ptr<SGReaderWriterOptions> opt;
                    opt = staticOptions(filePath, options);
                    _ObjectStatic obj;

                    opt->setInstantiateEffects(false);
                    opt->addErrorContext("terrain-stg", absoluteFileName.utf8Str());

                    if (SGPath(name).lower_extension() == "ac") {
                      // Generate material/Effects lookups for raw models, i.e.
                      // those not wrapped in an XML while will include Effects
                      opt->setInstantiateMaterialEffects(true);

                      if (token == BUILDING_ROUGH || token == BUILDING_DETAILED) {
                        opt->setMaterialName("OSM_Building");
                      } else if (token == ROAD_ROUGH || token == ROAD_DETAILED) {
                        opt->setMaterialName("OSM_Road");
                      } else if (token == RAILWAY_ROUGH || token == RAILWAY_DETAILED) {
                        opt->setMaterialName("OSM_Railway");
                      } else {
                        // Programming error.  If we get here then someone has added a verb to the list of
                        // tokens above but not in this set of if-else statements.
                        SG_LOG(SG_TERRAIN, SG_ALERT, "Programming Error - STG token without material branch");
                      }
                    }

                    obj._errorLocation = absoluteFileName;
                    obj._token = token;
                    obj._name = name;
                    obj._agl = false;
                    obj._proxy = true;
                    in >> obj._lon >> obj._lat >> obj._elev >> obj._hdg >> obj._pitch >> obj._roll >> obj._radius;

                    opt->setLocation(obj._lon, obj._lat);
                    if (token == BUILDING_DETAILED || token == ROAD_DETAILED || token == RAILWAY_DETAILED ) {
                        // Apply a lower LOD range if this is a detailed mesh.
                        range = _object_range_detailed;
                        double lrand = pc_rand();
                        if      (lrand < 0.1) range = range * 2.0;
                        else if (lrand < 0.4) range = range * 1.5;                                        
                    }
                    
                    obj._range = range;

                    obj._options = opt;
                    checkInsideBucket(absoluteFileName, obj._lon, obj._lat);
                    _objectStaticList.push_back(obj);
                } else if (token == BUILDING_LIST) {
                  _BuildingList buildinglist;
                  buildinglist._filename = path.utf8Str();
                  in >> buildinglist._material_name >> buildinglist._lon >> buildinglist._lat >> buildinglist._elev;
                  checkInsideBucket(absoluteFileName, buildinglist._lon, buildinglist._lat);
                  _buildingListList.push_back(buildinglist);
                } else if (token == TREE_LIST) {
                  _TreeList treelist;
                  treelist._filename = path.utf8Str();
                  in >> treelist._material_name >> treelist._lon >> treelist._lat >> treelist._elev;
                  checkInsideBucket(absoluteFileName, treelist._lon, treelist._lat);
                  _treeListList.push_back(treelist);
                } else if (token == LINE_FEATURE_LIST) {
                  _LineFeatureList lineFeaturelist;
                  lineFeaturelist._filename = path.utf8Str();
                  in >> lineFeaturelist._material;
                  lineFeaturelist._bucket = bucketIndexFromFileName(absoluteFileName.file_base().c_str());
                  _lineFeatureListList.push_back(lineFeaturelist);
                } else if (token == AREA_FEATURE_LIST) {
                  _AreaFeatureList areaFeaturelist;
                  areaFeaturelist._filename = path.utf8Str();
                  in >> areaFeaturelist._material;
                  areaFeaturelist._bucket = bucketIndexFromFileName(absoluteFileName.file_base().c_str());
                  _areaFeatureListList.push_back(areaFeaturelist);
                } else if (token == COASTLINE_LIST) {
                  _CoastlineList coastFeaturelist;
                  coastFeaturelist._filename = path.utf8Str();
                  coastFeaturelist._bucket = bucketIndexFromFileName(absoluteFileName.file_base().c_str());
                  _coastFeatureListList.push_back(coastFeaturelist);
                } else if (token == OBJECT_LIGHT) {
                    _Light light;
                    in >> light._lon >> light._lat >> light._elev
                        >> light._size >> light._intensity
                        >> light._on_period
                        >> light._color[0] >> light._color[1] >> light._color[2] >> light._color[3]
                        >> light._direction[0] >> light._direction[1] >> light._direction[2]
                        >> light._horizontal_angle >> light._vertical_angle
                        >> light._animation_params[0] >> light._animation_params[1] >> light._animation_params[2] >> light._animation_params[3];
                    checkInsideBucket(absoluteFileName, light._lon, light._lat);
                    _lightList.push_back(light);
                } else if (token == LIGHT_LIST) {
                    _LightList lightList;
                    lightList._filename = path.utf8Str();
                    in >> lightList._lon >> lightList._lat >> lightList._elev;
                    checkInsideBucket(absoluteFileName, lightList._lon, lightList._lat);
                    _lightListList.push_back(lightList);
                } else if (token == OBJECT_INSTANCED) {
                    osg::ref_ptr<SGReaderWriterOptions> opt;

                    opt = sharedOptions(filePath, options);
                    opt->addErrorContext("terrain-stg", absoluteFileName.utf8Str());

                    if (SGPath(name).lower_extension() == "ac")
                        opt->setInstantiateEffects(true);
                    else
                        opt->setInstantiateEffects(false);

                    _InstancedObject instancedObject;
                    instancedObject._modelname = name;

                    std::string filename;
                    in >> filename;
                    SGPath _filepath = filePath;
                    _filepath.append(filename);

                    instancedObject._errorLocation = absoluteFileName;
                    instancedObject._filename = _filepath.utf8Str();
                    instancedObject._options = opt;

                    in >> instancedObject._effect;

                    if (instancedObject._effect == "default") {
                        instancedObject._effect = "Effects/object-instancing";
                    }

                    if (!(instancedObject._effect == "Effects/object-instancing" || instancedObject._effect == "Effects/object-instancing-colored")) {
                        SG_LOG(SG_TERRAIN, SG_ALERT, "Unknown effect for instancing");
                    } else {
                        opt->setDefaultEffect(instancedObject._effect);

                        in >> instancedObject._lon >> instancedObject._lat >> instancedObject._elev;
                        checkInsideBucket(absoluteFileName, instancedObject._lon, instancedObject._lat);

                        _instancedObjectList.push_back(instancedObject);
                    }
                } else {
                    // Check registered callback for token. Keep lock until callback completed to make sure it will not be
                    // executed after a thread successfully executed removeSTGObjectHandler()
                    {
                        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(globalStgObjectCallbackLock);
                        STGObjectCallback callback = globalStgObjectCallbacks[token];

                        if (callback != nullptr) {
                            _ObjectStatic obj;
                            // pitch and roll are not common, so passed in "restofline" only
                            in >> obj._lon >> obj._lat >> obj._elev >> obj._hdg;
                            string_list restofline;
                            std::string buf;
                            while (in >> buf) {
                                restofline.push_back(buf);
                            }
                            callback(token,name, SGGeod::fromDegM(obj._lon, obj._lat, obj._elev), obj._hdg,restofline);
                        } else {
                            // SG_LOG( SG_TERRAIN, SG_ALERT, absoluteFileName << ": Unknown token '" << token << "'" );
                            simgear::reportFailure(simgear::LoadFailure::Misconfigured, simgear::ErrorCode::BTGLoad,
                                                   "Unknown STG token:" + token, absoluteFileName);
                        }
                    }
                }
            }
        }

        return true;
    }

    osg::Node* load(const SGBucket& bucket, const osgDB::Options* opt)
    {
        osg::ref_ptr<SGReaderWriterOptions> options;
        options = SGReaderWriterOptions::copyOrCreate(opt);
        float pagedLODExpiry = atoi(options->getPluginStringData("SimGear::PAGED_LOD_EXPIRY").c_str());

        osg::ref_ptr<osg::Group> terrainGroup = new osg::Group;
        terrainGroup->setDataVariance(osg::Object::STATIC);
        std::string terrain_name = string("terrain ").append(bucket.gen_index_str());
        terrainGroup->setName(terrain_name);

        simgear::ErrorReportContext ec{"terrain-bucket", bucket.gen_index_str()};

        bool vpb_active = SGSceneFeatures::instance()->getVPBActive();
        if (vpb_active) {

            // Load any line area or coastline features, which we will need before we generate the tile.
            if (!_lineFeatureListList.empty()) {

                LineFeatureBinList lineFeatures;

                for (const auto& b : _lineFeatureListList) {
                    // add the lineFeatures to the list
                    const auto path = SGPath(b._filename);
                    lineFeatures.push_back(new LineFeatureBin(path, b._material));
                }

                VPBTechnique::addLineFeatureList(bucket, lineFeatures);
            }

            if (!_areaFeatureListList.empty()) {

                AreaFeatureBinList areaFeatures;

                for (const auto& b : _areaFeatureListList) {
                    // add the lineFeatures to the list
                    const auto path = SGPath(b._filename);
                    areaFeatures.push_back(new AreaFeatureBin(path, b._material));
                }

                VPBTechnique::addAreaFeatureList(bucket, areaFeatures);
            }

            if (!_coastFeatureListList.empty()) {

                CoastlineBinList coastFeatures;

                for (const auto& b : _coastFeatureListList) {
                    // add the lineFeatures to the list
                    const auto path = SGPath(b._filename);
                    coastFeatures.push_back(new CoastlineBin(path));
                }

                VPBRasterRenderer::addCoastlineList(bucket, coastFeatures);
            }

            // OBJECTs include airports
            for (auto stgObject : _objectList) {
                osg::ref_ptr<osg::Node> node;
                node = osgDB::readRefNodeFile(stgObject._name, stgObject._options.get());

                if (!node.valid()) {
                    SG_LOG(SG_TERRAIN, SG_ALERT, stgObject._errorLocation << ": Failed to load "
                        << stgObject._token << " '" << stgObject._name << "'");
                    continue;
                }

                // Add the OBJECT to the elevation constraints of the terrain so the terrain
                // doesn't poke through the airport
                simgear::VPBTechnique::addElevationConstraint(node);

                terrainGroup->addChild(node.get());
            }
        } else if (_foundBase) {
            for (auto stgObject : _objectList) {
                osg::ref_ptr<osg::Node> node;
                simgear::ErrorReportContext ec("terrain-stg", stgObject._errorLocation.utf8Str());
                node = osgDB::readRefNodeFile(stgObject._name, stgObject._options.get());

                if (!node.valid()) {
                    SG_LOG(SG_TERRAIN, SG_ALERT, stgObject._errorLocation << ": Failed to load "
                        << stgObject._token << " '" << stgObject._name << "'");
                    continue;
                }
                terrainGroup->addChild(node.get());
            }
        } else {
            SG_LOG(SG_TERRAIN, SG_INFO, "  Generating ocean tile: " << bucket.gen_base_path() << "/" << bucket.gen_index_str());

            osg::Node* node = SGOceanTile(bucket, options->getMaterialLib());
            if (node) {
                node->setName("SGOceanTile");
                terrainGroup->addChild(node);
            } else {
                SG_LOG( SG_TERRAIN, SG_ALERT,
                        "Warning: failed to generate ocean tile!" );
            }
        }

        for (std::list<_ObjectStatic>::iterator i = _objectStaticList.begin(); i != _objectStaticList.end(); ++i) {
            if (!i->_agl)
                continue;
            i->_elev += elevation(*terrainGroup, SGGeod::fromDeg(i->_lon, i->_lat));
        }

        for (std::list<_Sign>::iterator i = _signList.begin(); i != _signList.end(); ++i) {
            if (!i->_agl)
                continue;
            i->_elev += elevation(*terrainGroup, SGGeod::fromDeg(i->_lon, i->_lat));
        }

        if (_objectStaticList.empty() &&
            _signList.empty() &&
            _buildingListList.empty() &&
            _treeListList.empty() &&
            _lineFeatureListList.empty() &&
            _areaFeatureListList.empty() &&
            _coastFeatureListList.empty() &&
            _lightList.empty() &&
            _lightListList.empty() &&
            _instancedObjectList.empty()) {
            // The simple case, just return the terrain group
            return terrainGroup.release();
        }

        osg::PagedLOD* pagedLOD = new osg::PagedLOD;
        std::string name = string("pagedObjectLOD ").append(bucket.gen_index_str());
        pagedLOD->setName(name);

        // This should be visible in any case.
        // If this is replaced by some lower level of detail, the parent LOD node handles this.
        pagedLOD->addChild(terrainGroup, 0, std::numeric_limits<float>::max());
        pagedLOD->setMinimumExpiryTime(0, pagedLODExpiry);

        // we just need to know about the read file callback that itself holds the data
        osg::ref_ptr<DelayLoadReadFileCallback> readFileCallback = new DelayLoadReadFileCallback;
        readFileCallback->_objectStaticList = _objectStaticList;
        readFileCallback->_buildingList = _buildingListList;
        readFileCallback->_treeList = _treeListList;
        readFileCallback->_lightList = _lightList;
        readFileCallback->_lightListList = _lightListList;
        readFileCallback->_instancedObjectList = _instancedObjectList;
        readFileCallback->_signList = _signList;
        readFileCallback->_options = options;
        readFileCallback->_bucket = bucket;

        osg::ref_ptr<osgDB::Options> callbackOptions = new osgDB::Options;
        callbackOptions->setReadFileCallback(readFileCallback.get());
        pagedLOD->setDatabaseOptions(callbackOptions.get());

        pagedLOD->setFileName(pagedLOD->getNumChildren(), "Dummy name - use the stored data in the read file callback");

        // Objects may end up displayed up to 2x the object range.
        pagedLOD->setRange(pagedLOD->getNumChildren(), 0, 2.0 * _object_range_rough);
        pagedLOD->setMinimumExpiryTime(pagedLOD->getNumChildren(), pagedLODExpiry);
        pagedLOD->setRadius(SG_TILE_RADIUS);
        pagedLOD->setCenterMode(osg::PagedLOD::USER_DEFINED_CENTER);

        SGVec3d coord;
        SGGeodesy::SGGeodToCart(bucket.get_center(), coord);
        pagedLOD->setCenter(toOsg(coord));
        SG_LOG( SG_TERRAIN, SG_DEBUG, "Tile " << bucket.gen_index_str() << " PagedLOD Center: " << pagedLOD->getCenter().x() << "," << pagedLOD->getCenter().y() << "," << pagedLOD->getCenter().z() );
        SG_LOG( SG_TERRAIN, SG_DEBUG, "Tile " << bucket.gen_index_str() << " PagedLOD Range: " << (2.0 * _object_range_rough));
        SG_LOG( SG_TERRAIN, SG_DEBUG, "Tile " << bucket.gen_index_str() << " PagedLOD Radius: " << SG_TILE_RADIUS);
        return pagedLOD;
    }

    double _object_range_bare;
    double _object_range_rough;
    double _object_range_detailed;
    bool _foundBase;
    std::list<_Object> _objectList;
    std::list<_ObjectStatic> _objectStaticList;
    std::list<_Sign> _signList;
    std::list<_BuildingList> _buildingListList;
    std::list<_TreeList> _treeListList;
    std::list<_LineFeatureList> _lineFeatureListList;
    std::list<_AreaFeatureList> _areaFeatureListList;
    std::list<_CoastlineList> _coastFeatureListList;
    std::list<_Light> _lightList;
    std::list<_LightList> _lightListList;
    std::list<_InstancedObject> _instancedObjectList;
};

ReaderWriterSTG::ReaderWriterSTG()
{
    supportsExtension("stg", "SimGear stg database format");
}

ReaderWriterSTG::~ReaderWriterSTG()
{
}

const char* ReaderWriterSTG::className() const
{
    return "STG Database reader";
}

osgDB::ReaderWriter::ReadResult
ReaderWriterSTG::readNode(const std::string& fileName, const osgDB::Options* options) const
{
    _ModelBin modelBin;
    SGBucket bucket(bucketIndexFromFileName(fileName));
    simgear::ErrorReportContext ec("terrain-bucket", bucket.gen_index_str());

    // We treat 123.stg different than ./123.stg.
    // The difference is that ./123.stg as well as any absolute path
    // really loads the given stg file and only this.
    // In contrast 123.stg uses the search paths to load a set of stg
    // files spread across the scenery directories.
    if (osgDB::getSimpleFileName(fileName) != fileName) {
        simgear::ErrorReportContext ec("terrain-stg", fileName);
        if (!modelBin.read(fileName, options))
            return ReadResult::FILE_NOT_FOUND;
    }

    // For stg meta files, we need options for the search path.
    if (!options) {
        return ReadResult::FILE_NOT_FOUND;
    }

    const auto sgOpts = dynamic_cast<const SGReaderWriterOptions*>(options);
    if (!sgOpts || sgOpts->getSceneryPathSuffixes().empty()) {
        SG_LOG(SG_TERRAIN, SG_ALERT, "Loading tile " << fileName << ", no scenery path suffixes were configured so giving up");
        return ReadResult::FILE_NOT_FOUND;
    }

    SG_LOG(SG_TERRAIN, SG_INFO, "Loading tile " << fileName);

    std::string basePath = bucket.gen_base_path();

    // Stop scanning paths once an object base is found
    // But do load all STGs at the same level (i.e from the same scenery path)
    const osgDB::FilePathList& filePathList = options->getDatabasePathList();
    for (auto path : filePathList) {
        SG_LOG(SG_TERRAIN, SG_DEBUG, "OSG DatabasePath: " << path);

        if (modelBin._foundBase) {
            break;
        }

        SGPath base(path);
        // check for non-suffixed file, and warn.
        SGPath pathWithoutSuffix = base / basePath / fileName;
        if (pathWithoutSuffix.exists()) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Found scenery file " << pathWithoutSuffix << " in scenery path " << path
                   << ".\nScenery paths without type subdirectories are no longer supported, please move thse files\n"
                   << "into a an appropriate subdirectory, for example:" << base / "Objects" / basePath / fileName);
        }

        for (auto suffix : sgOpts->getSceneryPathSuffixes()) {
            const auto p = base / suffix / basePath / fileName;
            simgear::ErrorReportContext ec("terrain-stg", p.utf8Str());
            modelBin.read(p, options);
        }
    }

    return modelBin.load(bucket, options);
}


void ReaderWriterSTG::setSTGObjectHandler(const std::string &token, STGObjectCallback callback)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(globalStgObjectCallbackLock);
    globalStgObjectCallbacks[token] = callback;
}

void ReaderWriterSTG::removeSTGObjectHandler(const std::string &token, STGObjectCallback callback)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(globalStgObjectCallbackLock);
    globalStgObjectCallbacks.erase(token);
}
}
