// matlib.cxx -- class to handle material properties
//
// Written by Curtis Olson, started May 1998.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id$


#ifdef HAVE_CONFIG_H
#  include <simgear_config.h>
#endif

#include <simgear/compiler.h>
#include <simgear/constants.h>
#include <simgear/structure/exception.hxx>

#include <string.h>
#include <string>
#include <mutex>
#include <utility>

#include <osgDB/ReadFile>

#include <simgear/debug/logstream.hxx>
#include <simgear/misc/sg_path.hxx>
#include <simgear/io/iostreams/sgstream.hxx>
#include <simgear/props/props.hxx>
#include <simgear/props/props_io.hxx>
#include <simgear/props/condition.hxx>
#include <simgear/scene/model/modellib.hxx>
#include <simgear/scene/tgdb/userdata.hxx>
#include <simgear/scene/util/SGReaderWriterOptions.hxx>
#include <simgear/scene/util/SGSceneFeatures.hxx>

#include "mat.hxx"

#include "Effect.hxx"
#include "Technique.hxx"
#include "matlib.hxx"

using std::string;
using namespace simgear;

class SGMaterialLib::MatLibPrivate
{
public:
    std::mutex mutex;
};

// Constructor
SGMaterialLib::SGMaterialLib ( void ) :
    d(new MatLibPrivate)
{
}

// Load a library of material properties
bool SGMaterialLib::load( const SGPath &fg_root, const SGPath& mpath,
        SGPropertyNode *prop_root )
{
    SGPropertyNode materialblocks;

    SG_LOG( SG_INPUT, SG_INFO, "Reading materials from " << mpath );
    try {
        readProperties( mpath, &materialblocks );
    } catch (const sg_exception &ex) {
        SG_LOG( SG_INPUT, SG_ALERT, "Error reading materials: "
                << ex.getMessage() );
        throw;
    }
    osg::ref_ptr<osgDB::Options> options
        = new osgDB::Options;
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    options->setDatabasePath(fg_root.utf8Str());

    std::lock_guard<std::mutex> g(d->mutex);

    simgear::PropertyList blocks = materialblocks.getChildren("region");
    simgear::PropertyList::const_iterator block_iter = blocks.begin();

    for (; block_iter != blocks.end(); block_iter++) {
    	SGPropertyNode_ptr node = block_iter->get();

		// Read name node purely for logging purposes
		const SGPropertyNode *nameNode = node->getChild("name");
		if (nameNode) {
			SG_LOG( SG_TERRAIN, SG_DEBUG, "Loading region "
					<< nameNode->getStringValue());
		}

		// Read list of areas
        auto arealist = std::make_shared<AreaList>();

		const simgear::PropertyList areas = node->getChildren("area");
		simgear::PropertyList::const_iterator area_iter = areas.begin();
		for (; area_iter != areas.end(); area_iter++) {
			float x1 = area_iter->get()->getFloatValue("lon1", -180.0f);
			float x2 = area_iter->get()->getFloatValue("lon2", 180.0);
			float y1 = area_iter->get()->getFloatValue("lat1", -90.0f);
			float y2 = area_iter->get()->getFloatValue("lat2", 90.0f);
			SGRect<float> rect = SGRect<float>(
					std::min<float>(x1, x2),
					std::min<float>(y1, y2),
					fabs(x2 - x1),
					fabs(y2 - y1));
			arealist->push_back(rect);
			SG_LOG( SG_TERRAIN, SG_DEBUG, " Area ("
					<< rect.x() << ","
					<< rect.y() << ") width:"
					<< rect.width() << " height:"
					<< rect.height());
		}

		// Read conditions node
		const SGPropertyNode *conditionNode = node->getChild("condition");
		SGSharedPtr<const SGCondition> condition;
		if (conditionNode) {
			condition = sgReadCondition(prop_root, conditionNode);
		}

		// Now build all the materials for this set of areas and conditions
        const std::string region = node->getStringValue("name");
		const simgear::PropertyList materials = node->getChildren("material");
		simgear::PropertyList::const_iterator materials_iter = materials.begin();
		for (; materials_iter != materials.end(); materials_iter++) {
			const SGPropertyNode *node = materials_iter->get();
			SGSharedPtr<SGMaterial> m =
					new SGMaterial(options.get(), node, prop_root, arealist, condition, region);

			std::vector<SGPropertyNode_ptr>names = node->getChildren("name");
			for ( unsigned int j = 0; j < names.size(); j++ ) {
				string name = names[j]->getStringValue();
				// cerr << "Material " << name << endl;
				matlib[name].push_back(m);
				m->add_name(name);
				SG_LOG( SG_TERRAIN, SG_DEBUG, "  Loading material "
						<< names[j]->getStringValue() );
			}
		}
    }

    simgear::PropertyList landclasses = materialblocks.getNode("landclass-mapping", true)->getChildren("map");
    simgear::PropertyList::const_iterator lc_iter = landclasses.begin();

    for (; lc_iter != landclasses.end(); lc_iter++) {
        SGPropertyNode_ptr node = lc_iter->get();
        int lc = node->getIntValue("landclass");
        const std::string mat = node->getStringValue("material-name");
        const bool water = node->getBoolValue("water");
        const bool sea = node->getBoolValue("sea");
        
        // Verify that the landclass mapping exists before creating the mapping
        const_material_map_iterator it = matlib.find( mat );
        if ( it == end() ) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Unable to find material " << mat << " for landclass " << lc);
        } else {
            landclasslib[lc] = {mat, water, sea};
            SG_LOG(SG_TERRAIN, SG_DEBUG, "Landclass mapping: " << lc << " : " << mat);
        }
    }

    return true;
}

// find a material record by material name and tile center
SGMaterial *SGMaterialLib::find( const string& material, const SGVec2f center ) const
{
    std::lock_guard<std::mutex> g(d->mutex);
    return internalFind(material, center);
}

SGMaterial* SGMaterialLib::internalFind(const string& material, const SGVec2f center) const
{
    SGMaterial *result = NULL;
    const_material_map_iterator it = matlib.find( material );
    if (it != end()) {
        // We now have a list of materials that match this
        // name. Find the first one that matches.
        // We start at the end of the list, as the materials
        // list is ordered with the smallest regions at the end.
        material_list::const_reverse_iterator iter = it->second.rbegin();
        while (iter != it->second.rend()) {
            result = *iter;
            if (result->valid(center)) {
                return result;
            }
            iter++;
        }
    }

    return NULL;
}

SGMaterial *SGMaterialLib::find( int lc, const SGVec2f center ) const
{
    std::string materialName;
    {
        std::lock_guard<std::mutex> g(d->mutex);
        const_landclass_map_iterator it = landclasslib.find(lc);
        if (it == landclasslib.end()) {
            return nullptr;
        }

        materialName = it->second._mat;
    }

    return find(materialName, center);
}

// find a material record by material name and tile center
SGMaterial *SGMaterialLib::find( const string& material, const SGGeod& center ) const
{
	SGVec2f c = SGVec2f(center.getLongitudeDeg(), center.getLatitudeDeg());
	return find(material, c);
}

// find a material record by material name and tile center
SGMaterial *SGMaterialLib::find( int lc, const SGGeod& center ) const
{
    std::string materialName;
    {
        std::lock_guard<std::mutex> g(d->mutex);
        const_landclass_map_iterator it = landclasslib.find(lc);
        if (it == landclasslib.end()) {
            return nullptr;
        }

        materialName = it->second._mat;
    }

    return find(materialName, center);
}

SGMaterialCache *SGMaterialLib::generateMatCache(SGVec2f center, const simgear::SGReaderWriterOptions* options, bool generateAtlas)
{

    SGMaterialCache* newCache = new SGMaterialCache();
    if (generateAtlas) newCache->setAtlas(SGMaterialLib::getOrCreateAtlas(landclasslib, center, options));
    std::lock_guard<std::mutex> g(d->mutex);
    material_map::const_reverse_iterator it = matlib.rbegin();
    for (; it != matlib.rend(); ++it) {
        newCache->insert(it->first, internalFind(it->first, center));
    }

    // Collapse down the mapping from landclasses to materials.
    const_landclass_map_iterator lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {
        SGMaterial* mat = internalFind(lc_iter->second._mat, center);
        newCache->insert(lc_iter->first, mat);
        SG_LOG(SG_TERRAIN, SG_DEBUG, "MatCache Landclass mapping: " << lc_iter->first << " : " << mat->get_names()[0]);
    }

    return newCache;
}

SGMaterialCache *SGMaterialLib::generateMatCache(SGGeod center, const simgear::SGReaderWriterOptions* options, bool generateAtlas)
{
	SGVec2f c = SGVec2f(center.getLongitudeDeg(), center.getLatitudeDeg());
	return SGMaterialLib::generateMatCache(c, options, generateAtlas);
}


// Destructor
SGMaterialLib::~SGMaterialLib ( void ) {
    SG_LOG( SG_TERRAIN, SG_DEBUG, "SGMaterialLib::~SGMaterialLib() size=" << matlib.size());
}

const SGMaterial *SGMaterialLib::findMaterial(const osg::Geode* geode)
{
    if (!geode)
        return 0;
    const simgear::EffectGeode* effectGeode;
    effectGeode = dynamic_cast<const simgear::EffectGeode*>(geode);
    if (!effectGeode)
        return 0;
    const simgear::Effect* effect = effectGeode->getEffect();
    if (!effect)
        return 0;
    const SGMaterialUserData* userData;
    userData = dynamic_cast<const SGMaterialUserData*>(effect->getUserData());
    if (!userData)
        return 0;
    return userData->getMaterial();
}

// Constructor
SGMaterialCache::SGMaterialCache ()
{
}

// Insertion into the material cache
void SGMaterialCache::insert(const std::string& name, SGSharedPtr<SGMaterial> material) {
	cache[name] = material;    
}

void SGMaterialCache::insert(int lc, SGSharedPtr<SGMaterial> material) {
	cache[getNameFromLandclass(lc)] = material;
}


// Search of the material cache
SGMaterial *SGMaterialCache::find(const string& material) const
{
    SGMaterialCache::material_cache::const_iterator it = cache.find(material);
    if (it == cache.end())
        return NULL;

    return it->second;
}

// Search of the material cache for a material code as an integer (e.g. from a VPB landclass texture).
SGMaterial *SGMaterialCache::find(int lc) const
{
    return find(getNameFromLandclass(lc));
}

osg::ref_ptr<Atlas> SGMaterialLib::getOrCreateAtlas(SGMaterialLib::landclass_map landclasslib, SGVec2f center, const simgear::SGReaderWriterOptions* const_options) {

    osg::ref_ptr<Atlas> atlas;
    // Non-VPB does not use the Atlas, so save some effort and return
    if (! SGSceneFeatures::instance()->getVPBActive()) return atlas;

    std::lock_guard<std::mutex> g(SGMaterialLib::_atlasCacheMutex);

    // A simple key to the atlas is just the list of textures.
    std::string id;
    const_landclass_map_iterator lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {
        SGMaterial* mat = find(lc_iter->second._mat, center);
        const std::string texture = mat->get_one_texture(0,0);
        id.append(texture);
        id.append(";");
    }

    atlas_map::iterator atlas_iter = SGMaterialLib::_atlasCache.find(id);
    if (atlas_iter != SGMaterialLib::_atlasCache.end()) return atlas_iter->second;

    // Cache lookup failure - generate a new atlas, but only if we have a chance of reading any textures
    if (const_options == 0) {
        return atlas;
    }

    osg::ref_ptr<SGReaderWriterOptions> options = SGReaderWriterOptions::copyOrCreate(const_options);
    options->setLoadOriginHint(SGReaderWriterOptions::LoadOriginHint::ORIGIN_MATERIAL_ATLAS);

    SG_LOG(SG_TERRAIN, SG_DEBUG, "Generating atlas " << (SGMaterialLib::_atlasCache.size() +1) << " of size " << landclasslib.size());
    if (landclasslib.size() > Atlas::MAX_MATERIALS) SG_LOG(SG_TERRAIN, SG_ALERT, "Too many landclass entries for uniform arrays:  " << landclasslib.size() << " > " << Atlas::MAX_MATERIALS);

    atlas = new Atlas(options);

    lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {
        // Add all the materials to the atlas
        int landclass = lc_iter->first;
        bool water = lc_iter->second._water;
        bool sea = lc_iter->second._sea;
        SGSharedPtr<SGMaterial> mat = find(lc_iter->second._mat, center);
        if (mat != NULL ) atlas->addMaterial(landclass, water, sea, mat);
    }

    SGMaterialLib::_atlasCache[id] = atlas;
    return atlas;
}

// Destructor
SGMaterialCache::~SGMaterialCache ( void ) {
    SG_LOG( SG_TERRAIN, SG_DEBUG, "SGMaterialCache::~SGMaterialCache() size=" << cache.size());
}

// Initalizer
SGMaterialLib::atlas_map SGMaterialLib::_atlasCache;
std::mutex SGMaterialLib::_atlasCacheMutex;