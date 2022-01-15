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
        
        // Verify that the landclass mapping exists before creating the mapping
        const_material_map_iterator it = matlib.find( mat );
        if ( it == end() ) {
            SG_LOG(SG_TERRAIN, SG_ALERT, "Unable to find material " << mat << " for landclass " << lc);
        } else {
            landclasslib[lc] = std::pair(mat, water);
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

        materialName = it->second.first;
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

        materialName = it->second.first;
    }

    return find(materialName, center);
}

SGMaterialCache *SGMaterialLib::generateMatCache(SGVec2f center, const simgear::SGReaderWriterOptions* options)
{

    SGMaterialCache* newCache = new SGMaterialCache(getMaterialTextureAtlas(center, options));

    std::lock_guard<std::mutex> g(d->mutex);

    material_map::const_reverse_iterator it = matlib.rbegin();
    for (; it != matlib.rend(); ++it) {
        newCache->insert(it->first, internalFind(it->first, center));
    }

    // Collapse down the mapping from landclasses to materials.
    const_landclass_map_iterator lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {
        newCache->insert(lc_iter->first, internalFind(lc_iter->second.first, center));
    }

    return newCache;
}

SGMaterialCache *SGMaterialLib::generateMatCache(SGGeod center, const simgear::SGReaderWriterOptions* options)
{
	SGVec2f c = SGVec2f(center.getLongitudeDeg(), center.getLatitudeDeg());
	return SGMaterialLib::generateMatCache(c, options);
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
SGMaterialCache::SGMaterialCache (Atlas atlas)
{
    _atlas = atlas;
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

// Generate a texture atlas for this location
SGMaterialCache::Atlas SGMaterialLib::getMaterialTextureAtlas(SGVec2f center, const simgear::SGReaderWriterOptions* const_options)
{
    SGMaterialCache::Atlas atlas;    
    
    // Non-VPB does not use the Atlas, so save some effort and return
    if (! SGSceneFeatures::instance()->getVPBActive()) return atlas;

    // A simple key to the atlas is just the list of textures.
    std::string id;
    const_landclass_map_iterator lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {
        SGMaterial* mat = find(lc_iter->second.first, center);
        const std::string texture = mat->get_one_texture(0,0);
        id.append(texture);
        id.append(";");        
    }

    SGMaterialLib::atlas_map::iterator atlas_iter = _atlasCache.find(id);

    if (atlas_iter != _atlasCache.end()) return atlas_iter->second;

    // Cache lookup failure - generate a new atlas, but only if we have a chance of reading any textures
    if (const_options == 0) {
        return atlas;
    }

    osg::ref_ptr<SGReaderWriterOptions> options = SGReaderWriterOptions::copyOrCreate(const_options);
    options->setLoadOriginHint(SGReaderWriterOptions::LoadOriginHint::ORIGIN_MATERIAL_ATLAS);
    
    atlas.image = new osg::Texture2DArray();

    SG_LOG(SG_TERRAIN, SG_DEBUG, "Generating atlas of size " << landclasslib.size());
    if (landclasslib.size() > SGMaterialCache::MAX_MATERIALS) SG_LOG(SG_TERRAIN, SG_ALERT, "Too many landclass entries for uniform arrays:  " << landclasslib.size() << " > " << SGMaterialCache::MAX_MATERIALS);
    
    atlas.textureLookup1 = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_textureLookup1", SGMaterialCache::MAX_MATERIALS);
    atlas.textureLookup2 = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_textureLookup2", SGMaterialCache::MAX_MATERIALS);
    atlas.dimensions = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_dimensionsArray", SGMaterialCache::MAX_MATERIALS);
    atlas.ambient = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_ambientArray", SGMaterialCache::MAX_MATERIALS);
    atlas.diffuse = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_diffuseArray", SGMaterialCache::MAX_MATERIALS);
    atlas.specular = new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_specularArray", SGMaterialCache::MAX_MATERIALS);
    atlas.materialParams1= new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_materialParams1", SGMaterialCache::MAX_MATERIALS);
    atlas.materialParams2= new osg::Uniform(osg::Uniform::Type::FLOAT_VEC4, "fg_materialParams2", SGMaterialCache::MAX_MATERIALS);

    atlas.image->setMaxAnisotropy(SGSceneFeatures::instance()->getTextureFilter());
    atlas.image->setResizeNonPowerOfTwoHint(false);

    atlas.image->setWrap(osg::Texture::WRAP_S,osg::Texture::REPEAT);
    atlas.image->setWrap(osg::Texture::WRAP_T,osg::Texture::REPEAT);

    unsigned int imageIndex = 0u; // Index into the image
    unsigned int materialLookupIndex = 0u; // Index into the material lookup
    lc_iter = landclasslib.begin();
    for (; lc_iter != landclasslib.end(); ++lc_iter) {

        int landclass = lc_iter->first;
        SGMaterial* mat = find(lc_iter->second.first, center);
        atlas.index[landclass] = materialLookupIndex;
        atlas.waterAtlas[landclass] = lc_iter->second.second;
        unsigned int textureList[SGMaterialCache::MAX_TEXTURES];

        if (mat != NULL) {

            if (mat->get_num_textures(0) > SGMaterialCache::MAX_TEXTURES) {
                SG_LOG(SG_GENERAL, SG_ALERT, "Unable to build texture atlas for landclass " 
                    << landclass << " aka " << mat->get_names()[0] 
                    << " too many textures: " << mat->get_num_textures(0) 
                    << " (maximum " << SGMaterialCache::MAX_TEXTURES << ")");
                continue;
            }

            atlas.dimensions->setElement(materialLookupIndex, osg::Vec4f(mat->get_xsize(), mat->get_ysize(), mat->get_shininess(), mat->get_parameter("edge-hardness")));
            atlas.ambient->setElement(materialLookupIndex, mat->get_ambient());
            atlas.diffuse->setElement(materialLookupIndex, mat->get_diffuse());
            atlas.specular->setElement(materialLookupIndex, mat->get_specular());

            // The following are material parameters that are normally built into the Effect as Uniforms.  In the WS30
            // case we need to pass them as an array, indexed against the material.
            atlas.materialParams1->setElement(materialLookupIndex, osg::Vec4f(mat->get_parameter("transition_model"), mat->get_parameter("hires_overlay_bias"), mat->get_parameter("grain_strength"), mat->get_parameter("intrinsic_wetness")));
            atlas.materialParams2->setElement(materialLookupIndex, osg::Vec4f(mat->get_parameter("dot_density"), mat->get_parameter("dot_size"), mat->get_parameter("dust_resistance"), mat->get_parameter("rock_strata")));

            // Similarly, there are specifically 7 textures that are defined in the materials that need to be passed into
            // the shader as an array based on the material lookup.
            //
            // The mapping from terrain-default.eff / terrain-overlay.eff is as follows
            //
            //  TEXTURE NAME texture-unit  Material texture index Default value
            //  Primary texure      0             0               n/a
            //  gradient_texture    2            13               Textures/Terrain/rock_alt.png
            //  dot_texture         3            15               Textures/Terrain/void.png
            //  grain_texture       4            14               Textures/Terrain/grain_texture.png
            //  mix_texture         5            12               Textures/Terrain/void.png
            //  detail_texture      7            11               Textures/Terrain/void.png
            //  overlayPrimaryTex   7            20               Textures/Terrain/void.png
            //  overlaySecondaryTex 8            21               Textures/Terrain/void.png

            for (unsigned int i = 0; i < SGMaterialCache::MAX_TEXTURES; i++) {
                std::string texture = mat->get_one_texture(0,i);

                if (texture.empty()) {
                    // This is a rather horrible hardcoded mapping of the default textures defined in
                    // terrain-default.eff and terrain-overlay.eff which are in effect defaults to
                    // the material definitions
                    if (i <  13) texture = std::string("Textures/Terrain/void.png");
                    if (i == 13) texture = std::string("Textures/Terrain/rock_alt.png");
                    if (i == 14) texture = std::string("Textures/Terrain/grain_texture.png");
                    if (i >  14) texture = std::string("Textures/Terrain/void.png");
                }

                SGPath texturePath = SGPath("Textures");
                std::string fullPath = SGModelLib::findDataFile(texture, options, texturePath);

                if (fullPath.empty()) {
                    SG_LOG(SG_GENERAL, SG_ALERT, "Cannot find texture \""
                            << texture << "\" in Textures folders when creating texture atlas");
                    texture = std::string("Textures/Terrain/void.png");
                    fullPath = SGModelLib::findDataFile(texture, options, texturePath);
                }

                if (atlas.textureMap.find(fullPath) == atlas.textureMap.end()) {
                    // Add any missing textures into the atlas image
                    // Copy the texture into the atlas in the appropriate place
                    osg::ref_ptr<osg::Image> subtexture = osgDB::readRefImageFile(fullPath, options);

                    if (subtexture && subtexture->valid()) {

                        if ((subtexture->s() != 2048) || (subtexture->t() != 2048)) {
                            subtexture->scaleImage(2048,2048,1);
                        }

                        atlas.image->setImage(imageIndex,subtexture);
                        atlas.textureMap[fullPath] = imageIndex;
                        ++imageIndex;
                    }
                }

                // At this point we know that the texture is present in the
                // atlas and referenced in the textureMap, so add it to the materialLookup
                textureList[i] = atlas.textureMap[fullPath];
            }

            // We now have a textureList containing the full set of textures.  Pack the relevant ones into the Vec4 of the index Uniform.
            // This is a bit of a hack to maintain compatibility with the WS2.0 material definitions, as the material definitions use the 
            // 11-15th textures for the various overlay textures for terrain-default.eff, we do the same for ws30.eff
            atlas.textureLookup1->setElement(materialLookupIndex, osg::Vec4f( (float) (textureList[0] / 255.0), (float) (textureList[11] / 255.0), (float) (textureList[12] / 255.0), (float) (textureList[13] / 255.0)));
            atlas.textureLookup2->setElement(materialLookupIndex, osg::Vec4f( (float) (textureList[14] / 255.0), (float) (textureList[15] / 255.0), (float) (textureList[20] / 255.0), (float) (textureList[21] / 255.0)));
        }

        ++materialLookupIndex;
    }

    // Cache for future lookups
    _atlasCache[id] = atlas;
    return atlas;
}

void SGMaterialCache::addAtlasUniforms(osg::StateSet* stateset) {
    stateset->addUniform(_atlas.dimensions);
    stateset->addUniform(_atlas.ambient);
    stateset->addUniform(_atlas.diffuse);
    stateset->addUniform(_atlas.specular);
    stateset->addUniform(_atlas.textureLookup1);
    stateset->addUniform(_atlas.textureLookup2);
    stateset->addUniform(_atlas.materialParams1);
    stateset->addUniform(_atlas.materialParams2);
}

// Destructor
SGMaterialCache::~SGMaterialCache ( void ) {
    SG_LOG( SG_TERRAIN, SG_DEBUG, "SGMaterialCache::~SGMaterialCache() size=" << cache.size());
}
