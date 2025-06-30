#include "MeshInfo.h"
#include <iostream>
#include <osgGA/GUIEventHandler>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/View>
#include <osg/Vec3>
#include <cover/VRViewer.h>
const float someThreshold = 0.1f; // Define a threshold for distance comparison

class PickHandler : public osgGA::GUIEventHandler {
    public:
        PickHandler(osg::Geometry* geometry) : _geometry(geometry) {}
        
        virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
            if (ea.getEventType() != osgGA::GUIEventAdapter::RELEASE || 
                ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
                return false;
            }
            
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (!view) return false;
            
            // Perform intersection test
            osgUtil::LineSegmentIntersector::Intersections intersections;
            if (view->computeIntersections(ea.getX(), ea.getY(), intersections)) {
                for (auto& hit : intersections) {
                    if (hit.drawable.get() == _geometry) {
                        processHit(hit, _geometry);
                        return true;
                    }
                }
            }
            return false;
        }
        
        void processHit(const osgUtil::LineSegmentIntersector::Intersection& hit, osg::Geometry* geometry) {
            // Get vertex array
            osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
            if (!vertices) return;
            
            // Get the primitive set that was hit
            osg::Geometry::PrimitiveSetList& primSets = geometry->getPrimitiveSetList();
            for (auto& primSet : primSets) {
                if (primSet->getMode() == osg::PrimitiveSet::POLYGON ||
                    primSet->getMode() == osg::PrimitiveSet::TRIANGLES || 
                    primSet->getMode() == osg::PrimitiveSet::QUADS) {
                    auto primitiveIndex = hit.primitiveIndex;
                    auto ps = dynamic_cast<osg::DrawElementsUInt*>(primSet.get());
                    auto vertexIndex = ps->at(primitiveIndex * 4);
                    for (size_t i = 0; i < 4; i++)
                    {
                        auto vertex = vertices->at(vertexIndex + i);
                        std::cerr << "Hit vertex: " << vertex.x() << ", " 
                                  << vertex.y() << ", " 
                                  << vertex.z() << std::endl;
                    }
                    
                    // // For each vertex in the primitive set
                    // std::cerr << "Hit primitive set: " << primSet->getType() << std::endl;
                    // for (unsigned int i = 0; i < primSet->getNumIndices(); ++i) {
                    //     unsigned int idx = primSet->index(i);
                    //     osg::Vec3 vertex = vertices->at(idx);
                    //     std::cerr << "Hit vertex: " << vertex.x() << ", " 
                    //               << vertex.y() << ", " 
                    //               << vertex.z() << std::endl;
                        

                        
                    //     // Check distance to hit point
                    //     float dist = (vertex - hit.getWorldIntersectPoint()).length();
                    //     if (dist < someThreshold) {
                    //         // Get next vertex in this primitive
                    //         unsigned int nextIdx = primSet->index((i + 1) % primSet->getNumIndices());
                    //         osg::Vec3 nextVertex = vertices->at(nextIdx);
                            
                    //         std::cout << "Next vertex: " << nextVertex.x() << ", " 
                    //                   << nextVertex.y() << ", " 
                    //                   << nextVertex.z() << std::endl;
                    //         return;
                    //     }
                    // }
                    std::cerr << std::endl;
                }
            }
        }
        
    private:
        osg::ref_ptr<osg::Geometry> _geometry;
    };

    osg::ref_ptr<osg::Geometry> createCubeGeometry(float size) {
        // Create a Geometry object
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    
        // Define the vertices of the cube
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        float halfSize = size / 2.0f;
    
        // Front face
        vertices->push_back(osg::Vec3(-halfSize, -halfSize, halfSize)); // Bottom-left
        vertices->push_back(osg::Vec3(halfSize, -halfSize, halfSize));  // Bottom-right
        vertices->push_back(osg::Vec3(halfSize, halfSize, halfSize));   // Top-right
        vertices->push_back(osg::Vec3(-halfSize, halfSize, halfSize));  // Top-left
    
        // Back face
        vertices->push_back(osg::Vec3(-halfSize, -halfSize, -halfSize)); // Bottom-left
        vertices->push_back(osg::Vec3(halfSize, -halfSize, -halfSize));  // Bottom-right
        vertices->push_back(osg::Vec3(halfSize, halfSize, -halfSize));   // Top-right
        vertices->push_back(osg::Vec3(-halfSize, halfSize, -halfSize));  // Top-left
    
        geometry->setVertexArray(vertices);
    
        // Define the normals for each face
        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
        normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));  // Front face
        normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f)); // Back face
        normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));  // Top face
        normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f)); // Bottom face
        normals->push_back(osg::Vec3(1.0f, 0.0f, 0.0f));  // Right face
        normals->push_back(osg::Vec3(-1.0f, 0.0f, 0.0f)); // Left face
    
        geometry->setNormalArray(normals, osg::Array::BIND_PER_PRIMITIVE_SET);
    
        // Define the indices for the cube's faces
        osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    
        // Front face
        indices->push_back(0);
        indices->push_back(1);
        indices->push_back(2);
        indices->push_back(3);
    
        // Back face
        indices->push_back(4);
        indices->push_back(5);
        indices->push_back(6);
        indices->push_back(7);
    
        // Left face
        indices->push_back(0);
        indices->push_back(4);
        indices->push_back(7);
        indices->push_back(3);
    
        // Right face
        indices->push_back(1);
        indices->push_back(5);
        indices->push_back(6);
        indices->push_back(2);
    
        // Top face
        indices->push_back(3);
        indices->push_back(2);
        indices->push_back(6);
        indices->push_back(7);
    
        // Bottom face
        indices->push_back(0);
        indices->push_back(1);
        indices->push_back(5);
        indices->push_back(4);
    
        geometry->addPrimitiveSet(indices);
    
        return geometry;
    }

void printChildren(osg::Group* node, int depth) {
    if (!node) return;
    
    for (int i = 0; i < depth; ++i) std::cout << "  ";
    std::cout << "Node: " << node->getName() << std::endl;
    
    for (unsigned int i = 0; i < node->getNumChildren(); ++i) {
        if(auto g = dynamic_cast<osg::Group*>(node->getChild(i))) {
            printChildren(g, depth + 1);
        } else if(auto geode = dynamic_cast<osg::Geode*>(node->getChild(i))) {
            std::cout << "  Geode: " << geode->getName() << std::endl;
        } else if(auto transform = dynamic_cast<osg::MatrixTransform*>(node->getChild(i))) {
            std::cout << "  Transform: " << transform->getName() << std::endl;
        } else {
            std::cout << "  Unknown type: " << node->getChild(i)->getName() << std::endl;
        }
    }
}

MeshInfo::MeshInfo()
: coVRPlugin(COVER_PLUGIN_NAME)
{
    auto geo = createCubeGeometry(1.0f); // Create a cube geometry with size 1.0f

    cover->getObjectsRoot()->addChild(geo);
    // Create the pick handler and add it to the event handler list
    osg::ref_ptr<PickHandler> pickHandler = new PickHandler(geo.get());
    VRViewer::instance()->addEventHandler(pickHandler);
    
    osg::ref_ptr<osg::Node> vistleRoot;
    for (size_t i = 0; i < cover->getObjectsRoot()->getNumChildren(); i++)
    {
        if(cover->getObjectsRoot()->getChild(i)->getName() == "Root")
        {
            vistleRoot = cover->getObjectsRoot()->getChild(i);
            break;
        }
    }
    if(!vistleRoot.valid())
    {
        std::cerr << "Vistle root not found" << std::endl;
        return;
    }
    printChildren(vistleRoot->asGroup(), 0);
    
    // Set up other plugin components as needed
}

COVERPLUGIN(MeshInfo);