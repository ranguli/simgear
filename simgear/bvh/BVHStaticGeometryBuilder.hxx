// Copyright (C) 2008 - 2009  Mathias Froehlich - Mathias.Froehlich@web.de
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#ifndef BVHStaticGeometryBuilder_hxx
#define BVHStaticGeometryBuilder_hxx

#include <algorithm>
#include <map>
#include <set>

#include <simgear/structure/SGReferenced.hxx>
#include <simgear/structure/SGSharedPtr.hxx>

#include "BVHVisitor.hxx"
#include "BVHNode.hxx"
#include "BVHGroup.hxx"
#include "BVHTransform.hxx"

#include "BVHStaticData.hxx"

#include "BVHStaticNode.hxx"
#include "BVHStaticLeaf.hxx"
#include "BVHStaticTriangle.hxx"
#include "BVHStaticBinary.hxx"
#include "BVHStaticGeometry.hxx"

namespace simgear {

class BVHStaticGeometryBuilder : public SGReferenced {
public:
    BVHStaticGeometryBuilder() :
        _staticData(new BVHStaticData),
        _currentMaterial(0),
        _currentMaterialIndex(~0u)
    { }

    virtual ~BVHStaticGeometryBuilder()
    {
    }

    struct LeafRef {
        LeafRef(const BVHStaticLeaf* leaf, const BVHStaticData& data) :
            _leaf(leaf),
            _box(_leaf->computeBoundingBox(data)),
            _center(_leaf->computeCenter(data))
        { }
        SGSharedPtr<const BVHStaticLeaf> _leaf;
        SGBoxf _box;
        SGVec3f _center;
    };
    typedef std::list<LeafRef> LeafRefList;

    struct LeafRefLess {
        LeafRefLess(unsigned sortAxis) : _sortAxis(sortAxis) {}
        bool operator()(const LeafRef& x, const LeafRef& y)
        { return x._center[_sortAxis] < y._center[_sortAxis]; }
        unsigned _sortAxis;
    };

    SGSharedPtr<BVHStaticData> _staticData;
    LeafRefList _leafRefList;

    typedef std::map<SGVec3f, unsigned> VertexMap;
    VertexMap _vertexMap;

    typedef std::set<SGVec3<unsigned> > TriangleSet;
    TriangleSet _triangleSet;

    void setCurrentMaterial(const BVHMaterial* material)
    {
        _currentMaterial = material;
        _currentMaterialIndex = addMaterial(material);
    }
    const BVHMaterial* getCurrentMaterial() const
    {
        return _currentMaterial;
    }
    unsigned addMaterial(const BVHMaterial* material)
    {
        MaterialMap::const_iterator i = _materialMap.find(material);
        if (i != _materialMap.end())
            return i->second;
        unsigned index = _staticData->addMaterial(material);
        _materialMap[material] = index;
        return index;
    }

    typedef std::map<const BVHMaterial*, unsigned> MaterialMap;
    MaterialMap _materialMap;
    const BVHMaterial* _currentMaterial;
    unsigned _currentMaterialIndex;

    void addTriangle(const SGVec3f& v1, const SGVec3f& v2, const SGVec3f& v3, unsigned idx1=0, unsigned idx2=0, unsigned idx3=0)
    {
        std::tuple<unsigned, unsigned> indicePair[3];
        indicePair[0] = std::make_tuple(addVertex(v1), idx1);
        indicePair[1] = std::make_tuple(addVertex(v2), idx2);
        indicePair[2] = std::make_tuple(addVertex(v3), idx3);

        std::sort(indicePair, indicePair + 6);

        unsigned indices[3] = { std::get<0>(indicePair[0]), std::get<0>(indicePair[1]), std::get<0>(indicePair[2]) };
        unsigned original_indices[3] = { std::get<1>(indicePair[0]), std::get<1>(indicePair[1]), std::get<1>(indicePair[2]) };
        SGVec3<unsigned> indexKey(indices);
        if (_triangleSet.find(indexKey) != _triangleSet.end())
            return;
        _triangleSet.insert(indexKey);
        BVHStaticTriangle* staticTriangle;
        // REVIEW: Memory Leak - 11,680 bytes in 365 blocks are indirectly lost
        staticTriangle = new BVHStaticTriangle(_currentMaterialIndex, indices, original_indices);
        _leafRefList.push_back(LeafRef(staticTriangle, *_staticData));
    }
    unsigned addVertex(const SGVec3f& v)
    {
        VertexMap::const_iterator i = _vertexMap.find(v);
        if (i != _vertexMap.end())
            return i->second;
        unsigned index = _staticData->addVertex(v);
        _vertexMap[v] = index;
        return index;
    }

    BVHStaticGeometry* buildTree()
    {
        const BVHStaticNode* tree = buildTreeRecursive(_leafRefList);
        if (!tree)
            return 0;
        _staticData->trim();
        return new BVHStaticGeometry(tree, _staticData);
    }

private:
    static void
    centerSplitLeafs(unsigned splitAxis, const double& splitValue,
                     LeafRefList& leafs, LeafRefList split[2])
    {
        while (!leafs.empty()) {
            if (leafs.front()._center[splitAxis] < splitValue) {
                split[0].splice(split[0].begin(), leafs, leafs.begin());
            } else {
                split[1].splice(split[1].begin(), leafs, leafs.begin());
            }
        }
    }

    static void
    equalSplitLeafs(unsigned splitAxis, LeafRefList& leafs,
                    LeafRefList split[2])
    {
        leafs.sort(LeafRefLess(splitAxis));
        while (true) {
            if (leafs.empty())
                break;
            split[0].splice(split[0].begin(), leafs, leafs.begin());
            
            if (leafs.empty())
                break;
            split[1].splice(split[1].begin(), leafs, --leafs.end());
        }
    }
    
    static const BVHStaticNode* buildTreeRecursive(LeafRefList& leafs)
    {
        // recursion termination
        if (leafs.empty())
            return 0;
        // FIXME size is O(n)!!!
        //   if (leafs.size() == 1)
        if (++leafs.begin() == leafs.end())
            return leafs.front()._leaf;
        
        SGBoxf box;
        for (LeafRefList::const_iterator i = leafs.begin();
             i != leafs.end(); ++i)
            box.expandBy(i->_box);
        
        //   // FIXME ...
        //   if (length(box.getSize()) < 1)
        //     return new BVHBox(box);
        
        if (box.empty())
            return 0;
        
        unsigned splitAxis = box.getBroadestAxis();
        LeafRefList splitLeafs[2];
        double splitValue = box.getCenter()[splitAxis];
        centerSplitLeafs(splitAxis, splitValue, leafs, splitLeafs);
        
        if (splitLeafs[0].empty() || splitLeafs[1].empty()) {
            for (unsigned i = 0; i < 3 ; ++i) {
                if (i == splitAxis)
                    continue;
                
                leafs.swap(splitLeafs[0]);
                leafs.splice(leafs.begin(), splitLeafs[1]);
                splitValue = box.getCenter()[i];
                centerSplitLeafs(i, splitValue, leafs, splitLeafs);
                
                if (!splitLeafs[0].empty() && !splitLeafs[1].empty()) {
                    splitAxis = i;
                    break;
                }
            }
        }
        if (splitLeafs[0].empty() || splitLeafs[1].empty()) {
            leafs.swap(splitLeafs[0]);
            leafs.splice(leafs.begin(), splitLeafs[1]);
            equalSplitLeafs(splitAxis, leafs, splitLeafs);
        }
        
        const BVHStaticNode* child0 = buildTreeRecursive(splitLeafs[0]);
        const BVHStaticNode* child1 = buildTreeRecursive(splitLeafs[1]);
        if (!child0)
            return child1;
        if (!child1)
            return child0;
        
        return new BVHStaticBinary(splitAxis, child0, child1, box);
    }
};

}

#endif
