/*  -*-c++-*- 
 *  Copyright (C) 2008 Cedric Pinson <cedric.pinson@plopbyte.net>
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/

#include "RigTransformHardware.h"

#include <osgAnimation/BoneMapVisitor>
#include <osgAnimation/RigGeometry>
#include <osgDB/ReadFile>
#include <sstream>
#include <cover/coVRShader.h>

osg::ref_ptr<osg::Program> CommonProgram;
using namespace osgAnimation;
bool coVRRigTransformHardware::init(osgAnimation::RigGeometry &rig)
{
        if(_perVertexInfluences.empty())
    {
        prepareData(rig);
        return false;
    }
    if(!rig.getSkeleton())
        return false;

    BoneMapVisitor mapVisitor;
    rig.getSkeleton()->accept(mapVisitor);
    BoneMap boneMap = mapVisitor.getBoneMap();

    if (!buildPalette(boneMap,rig) )
        return false;

    osg::Geometry& source = *rig.getSourceGeometry();
    osg::Vec3Array* positionSrc = dynamic_cast<osg::Vec3Array*>(source.getVertexArray());

    if (!positionSrc)
    {
        OSG_WARN << "RigTransformHardware no vertex array in the geometry " << rig.getName() << std::endl;
        return false;
    }

    // copy shallow from source geometry to rig
    rig.copyFrom(source);

    osg::ref_ptr<osg::Program> program ;
    osg::ref_ptr<osg::Shader> vertexshader;
    osg::ref_ptr<osg::StateSet> stateset = rig.getOrCreateStateSet();

    //grab geom source program and vertex shader if _shader is not set
    if(!_shader.valid() && (program = (osg::Program*)stateset->getAttribute(osg::StateAttribute::PROGRAM)))
    {
        for(unsigned int i = 0; i<program->getNumShaders(); ++i)
            if(program->getShader(i)->getType() == osg::Shader::VERTEX)
            {
                vertexshader = program->getShader(i);
                program->removeShader(vertexshader);
            }
    }
    else
    {
        program = new osg::Program;
        program->setName("HardwareSkinning");
    }
    //set default source if _shader is not user set
    if (!vertexshader.valid())
    {
        if (!_shader.valid())
        {
            auto shader = opencover::coVRShaderList::instance()->getUnique("skinning");
            vertexshader = shader->getVertexShader();
        }
        else vertexshader = _shader;
    }

    if (!vertexshader.valid())
    {
        OSG_WARN << "RigTransformHardware can't load VertexShader" << std::endl;
        return false;
    }

    // replace max matrix by the value from uniform
    {
        std::string str = vertexshader->getShaderSource();
        std::string toreplace = std::string("MAX_MATRIX");
        std::size_t start = str.find(toreplace);
        if (std::string::npos != start)
        {
            std::stringstream ss;
            ss << getMatrixPaletteUniform()->getNumElements();
            str.replace(start, toreplace.size(), ss.str());
            vertexshader->setShaderSource(str);
        }
        else
        {
            OSG_WARN<< "MAX_MATRIX not found in Shader! " << str << std::endl;
        }
        OSG_INFO << "Shader " << str << std::endl;
    }

    unsigned int nbAttribs = getNumVertexAttrib();
    for (unsigned int i = 0; i < nbAttribs; i++)
    {
        std::stringstream ss;
        ss << "boneWeight" << i;
        program->addBindAttribLocation(ss.str(), _minAttribIndex + i);
        rig.setVertexAttribArray(_minAttribIndex + i, getVertexAttrib(i));
        OSG_INFO << "set vertex attrib " << ss.str() << std::endl;
    }

    program->addShader(vertexshader.get());

    stateset->removeUniform("nbBonesPerVertex");
    stateset->addUniform(new osg::Uniform("nbBonesPerVertex",(int)_bonesPerVertex));

    stateset->removeUniform("matrixPalette");
    stateset->addUniform(_uniformMatrixPalette);

    stateset->setAttribute(program.get());

    _needInit = false;
    return true;
}
