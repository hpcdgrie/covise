#include "SolarPanel.h"

#include <cover/coVRShader.h>
#include <lib/core/utils/color.h>
#include <lib/core/utils/osgUtils.h>

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateAttribute>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <string>

using namespace core::utils::osgUtils;

void SolarPanel::init() { initDrawables(); }

void SolarPanel::initDrawables() {
  osg::ref_ptr<osg::Group> group = m_node->asGroup();
  if (!group) {
    std::cerr << "SolarPanel: m_node is not a group!" << std::endl;
    return;
  }
  auto geodes = getGeodes(group);
  std::string name = "SolarpanelPart";
  int i = 0;
  for (auto geode : *geodes) {
    geode->setName(name + std::to_string(i));
    ++i;
  }
  m_drawables.push_back(m_node);
}

void SolarPanel::updateDrawables() {}

void SolarPanel::updateColor(const osg::Vec4 &color) {
//   constexpr const char *VERTEX_EMISSION_SHADER =
//       "void main() { "
//       "   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; "
//       "   gl_TexCoord[0] = gl_MultiTexCoord0; "
//       "}";
//   constexpr const char *FRAGMENT_EMISSION_SHADER =
//       "uniform sampler2D emissionMap;"
//       "void main() { "
//       "   gl_FragColor = texture2D(emissionMap, gl_TexCoord[0]);"
//       "}";
  constexpr const char *VERTEX_EMISSION_SHADER = R"(
void main() {
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	// gl_TexCoord[0] = gl_MultiTextCoord0;
}
  )";
  constexpr const char *FRAGMENT_EMISSION_SHADER = R"(
// uniform sampler2D texUnit0;
void main(void) {
    vec2 st = gl_FragCoord.xy/vec2(1080.0, 960.0);
    gl_FragColor = vec4(st, 0.0, 1.0);
	// vec4 color;
	// color = gl_Color;
	// gl_FragColor = color * texture2D(texUnit0, gl_TexCoord[0]);
}
  )";
  osg::ref_ptr<osg::Program> shader = new osg::Program;
  osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX);
//   vertexShader->setShaderSource(VERTEX_EMISSION_SHADER);
  vertexShader->readShaderFile(osg::Shader::VERTEX, "/home/hpcmdjur/covise/src/OpenCOVER/plugins/hlrs/Energy/lib/core/utils/shader/solarpanel_osg.vert");
  osg::ref_ptr<osg::Shader> fragment = new osg::Shader(osg::Shader::FRAGMENT);
  fragment->readShaderFile(osg::Shader::FRAGMENT, "/home/hpcmdjur/covise/src/OpenCOVER/plugins/hlrs/Energy/lib/core/utils/shader/solarpanel_osg.frag");
//   fragment->setShaderSource(FRAGMENT_EMISSION_SHADER);
  fragment->setShaderSource(FRAGMENT_EMISSION_SHADER);
  shader->addShader(vertexShader);
  shader->addShader(fragment);
  //   auto shader = opencover::coVRShaderList::instance()->get("Phong");
  for (auto &node : m_drawables) {
    auto geode = dynamic_cast<osg::Geode *>(node.get());
    if (geode) {
      //   core::utils::osgUtils::createOutlineFX(geode, color, 0.01f);
      //   core::utils::osgUtils::applyOutlineShader(geode, color, 0.01f);
      core::utils::color::overrideGeodeColor(geode, color);
      continue;
    }

    auto group = dynamic_cast<osg::Group *>(node.get());
    if (group) {
      auto geodes = getGeodes(group);
      for (auto &geode : *geodes) {
        // core::utils::osgUtils::createOutlineFX(geode, color, 0.01f);
        // core::utils::osgUtils::applyOutlineShader(geode, color, 0.01f);
        osg::ref_ptr<osg::Material> mat = new osg::Material;
        osg::Material::Face face = osg::Material::FRONT;
        // auto brightDiffuse = ;
        // osg::Vec4 brighterColor(0.8f, 0.8f, 0.8f, 1.0f);
        mat->setDiffuse(face, color);
        // mat->setAmbient(face, osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
        // mat->setEmission(face, osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f));
        mat->setAmbient(face, color);
        mat->setEmission(face, color);
        // mat->setSpecular(face, color);
        // mat->setAlpha(face, 0.5f);
        // mat->setColorMode(osg::Material::EMISSION);
        // mat->setShininess(face, 0.5f);
        osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();
        if (!stateset) continue;
        stateset->setAttribute(mat, osg::StateAttribute::OVERRIDE);
        // stateset->setAttributeAndModes(shader, osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        // core::utils::color::overrideGeodeColor(geode, color);
        // auto textureList = stateset->getTextureAttributeList();
        // for (int i = 0; i < geode->getNumDrawables(); ++i) {
        //   osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(i);
        //   osg::ref_ptr<osg::Geometry> geom = drawable->asGeometry();
        //   if (!geom) continue;
        //   auto geoStateset = geom->getOrCreateStateSet();
        //   if (!geoStateset) continue;
        //   auto textureList = geoStateset->getTextureAttributeList();
        //   for (const auto &attributeMap : textureList) {
        //     for (const auto &pair : attributeMap) {
        //       const osg::StateAttribute::TypeMemberPair &typeMemberPair = pair.first;
        //       const osg::StateSet::RefAttributePair &refAttributePair = pair.second;
        //       osg::StateAttribute *attribute = refAttributePair.first.get();

        //       if (attribute &&
        //           typeMemberPair.first == osg::StateAttribute::TEXTURE) {
        //         unsigned int textureUnit = typeMemberPair.second;
        //         osg::Texture *texture = dynamic_cast<osg::Texture *>(attribute);
        //         if (texture) {
        //           std::cout << "Gefundene Textur auf Einheit: " << textureUnit
        //                     << std::endl;
        //           // textures.push_back(texture);
        //           int i = 0;
        //           if (osg::Texture2D *texture2D =
        //                   dynamic_cast<osg::Texture2D *>(texture)) {
        //             std::cout << "  Textur-Einheit: " << textureUnit << std::endl;
        //             std::cout << "  Textur-Name: " << texture2D->getName()
        //                       << std::endl;
        //             osg::Image *image = texture2D->getImage();
        //             if (image) {
        //               std::cout << "  Breite: " << image->s()
        //                         << ", Höhe: " << image->t()
        //                         << ", PixelFormat: " << image->getPixelFormat()
        //                         << std::endl;
        //             }
        //             stateset->setTextureAttributeAndModes(i, texture2D,
        //                                                   osg::StateAttribute::ON);
        //             stateset->addUniform(new osg::Uniform(
        //                 "emissionMap", i));  // Assuming texture unit 0
        //             ++i;
        //           }
        //         }
        //       }
        //     }
        //   }
        // }
      }
    }
  }
}
