/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

// **************************************************************************

#include "CsvPointCloud.h"
#include <config/CoviseConfig.h>
#include <cover/coVRAnimationManager.h>
#include <cover/coVRFileManager.h>
#include <cover/coVRPluginSupport.h>
#include <cover/coVRShader.h>
#include <cover/coVRTui.h>
#include <cover/coVRMSController.h>
#include <cover/coVRConfig.h>
#include <OpenVRUI/osg/mathUtils.h>
#include <osg/AlphaFunc>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/TemplatePrimitiveFunctor>
#include <osg/TemplatePrimitiveIndexFunctor>
#include <osg/io_utils>
#include <osg/TexEnv>
#include <osg/TexGen>

#include <vrml97/vrml/VrmlNode.h>
#include <vrml97/vrml/VrmlNodeTransform.h>
#include <vrml97/vrml/VrmlNodeType.h>
#include <vrml97/vrml/VrmlNamespace.h>

#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>

using namespace boost::timer;
using namespace osg;
using namespace covise;
using namespace opencover;
using namespace vrml;

static FileHandler handler[] = {{nullptr, CsvPointCloudPlugin::load, CsvPointCloudPlugin::unload, "csv"}, {nullptr, CsvPointCloudPlugin::load, CsvPointCloudPlugin::unload, "oct"}};

CsvPointCloudPlugin *CsvPointCloudPlugin::m_plugin = nullptr;

COVERPLUGIN(CsvPointCloudPlugin)

class MachineNode;
std::vector<MachineNode *> machineNodes;

static VrmlNode *creator(VrmlScene *scene);

class PLUGINEXPORT MachineNode : public vrml::VrmlNodeChild
{
public:
    static VrmlNode *creator(VrmlScene *scene)
    {
        return new MachineNode(scene);
    }
    MachineNode(VrmlScene *scene) : VrmlNodeChild(scene), m_index(machineNodes.size())
    {

        std::cerr << "vrml Machine node created" << std::endl;
        machineNodes.push_back(this);
    }
    ~MachineNode()
    {
        machineNodes.erase(machineNodes.begin() + m_index);
    }
    // Define the fields of XCar nodes
    static VrmlNodeType *defineType(VrmlNodeType *t = 0)
    {
        static VrmlNodeType *st = 0;

        if (!t)
        {
            if (st)
                return st; // Only define the type once.
            t = st = new VrmlNodeType("CsvPointCloud", creator);
        }

        VrmlNodeChild::defineType(t); // Parent class

        t->addEventOut("x", VrmlField::SFVEC3F);
        t->addEventOut("y", VrmlField::SFVEC3F);
        t->addEventOut("z", VrmlField::SFVEC3F);

        return t;
    }
    virtual VrmlNodeType *nodeType() const { return defineType(); };
    VrmlNode *cloneMe() const
    {
        return new MachineNode(*this);
    }
    void move(VrmlSFVec3f &position)
    {
        auto t = System::the->time();
        eventOut(t, "x", VrmlSFVec3f{-position.x(), 0, 0});
        eventOut(t, "y", VrmlSFVec3f{0, 0, -position.y()});
        eventOut(t, "z", VrmlSFVec3f{0, position.z(), 0});
    }

private:
    size_t m_index = 0;
};

VrmlNode *creator(VrmlScene *scene)
{
    return new MachineNode(scene);
}

namespace fs = boost::filesystem;

// Constructor
CsvPointCloudPlugin::CsvPointCloudPlugin()
    : ui::Owner("CsvPointCloud", cover->ui), m_CsvPointCloudMenu(new ui::Menu("CsvPointCloud", this)), m_dataScale(new ui::EditField(m_CsvPointCloudMenu, "Scale")), m_colorMenu(new ui::Menu(m_CsvPointCloudMenu, "ColorMenu")), m_coordTerms{{new ui::EditField(m_CsvPointCloudMenu, "X"), new ui::EditField(m_CsvPointCloudMenu, "Y"), new ui::EditField(m_CsvPointCloudMenu, "Z")}}, m_machinePositionsTerms{{new ui::EditField(m_CsvPointCloudMenu, "Right"), new ui::EditField(m_CsvPointCloudMenu, "Forward"), new ui::EditField(m_CsvPointCloudMenu, "Up")}}, m_colorTerm(new ui::EditField(m_CsvPointCloudMenu, "Color")), m_pointSizeSlider(new ui::Slider(m_CsvPointCloudMenu, "PointSize")), m_numPointsSlider(new ui::Slider(m_CsvPointCloudMenu, "NumPoints")), m_colorMapSelector(*m_CsvPointCloudMenu), m_applyBtn(new ui::Button(m_CsvPointCloudMenu, "Apply")), m_timeScaleIndicator(new ui::EditField(m_CsvPointCloudMenu, "TimeScaleIndicator")), m_pointReductionCriteria(new ui::EditField(m_CsvPointCloudMenu, "PointReductionCriteria")), m_delimiter(new ui::EditField(m_CsvPointCloudMenu, "Delimiter")), m_offset(new ui::EditField(m_CsvPointCloudMenu, "HeaderOffset")), m_colorsGroup(new ui::Group(m_CsvPointCloudMenu, "Colors")), m_colorBar(new opencover::ColorBar(m_colorMenu)), m_editFields{m_dataScale, m_coordTerms[0], m_coordTerms[1], m_coordTerms[2], m_machinePositionsTerms[0], m_machinePositionsTerms[1], m_machinePositionsTerms[2], m_colorTerm, m_timeScaleIndicator, m_delimiter, m_offset, m_pointReductionCriteria}
{
    coVRAnimationManager::instance()->setAnimationSkipMax(5000);
    m_dataScale->setValue("1");
    for (auto ef : m_editFields)
        ef->setShared(true);

    if (m_delimiter->value().empty())
        m_delimiter->setValue(";");

    m_pointSizeSlider->setBounds(0, 20);
    m_pointSizeSlider->setValue(4);
    m_pointSizeSlider->setCallback([this](ui::Slider::ValueType val, bool release)
                                   {
                                      if (m_pointCloud)
                                          dynamic_cast<osg::Point *>(m_pointCloud->getStateSet()->getAttribute(osg::StateAttribute::Type::POINT))->setSize(val);
                                      if (m_reducedPointCloud)
                                          dynamic_cast<osg::Point*>(m_reducedPointCloud->getStateSet()->getAttribute(osg::StateAttribute::Type::POINT))->setSize(val); });
    m_pointSizeSlider->setShared(true);

    m_numPointsSlider->setShared(true);
    m_numPointsSlider->setBounds(0, 1);
    m_numPointsSlider->setValue(1);

    m_applyBtn->setCallback([this](bool state)
                            {
                                (void)state;
                                if(m_currentGeode)
                                {
                                    auto parent = m_currentGeode->getParent(0);
                                    auto filename = m_currentGeode->getName();
                                    unloadFile(filename);
                                    load(filename.c_str(), parent, nullptr);
                                } });
    m_applyBtn->setShared(true);

    m_colorTerm->setCallback([this](const std::string &text) {
        if (m_dataTable)
        {
            Expression reduceExpression;

            if (!compileSymbol(*m_dataTable, m_pointReductionCriteria->value(), reduceExpression))
                return;
            auto colors = getColors(*m_dataTable, reduceExpression);
            if(m_pointCloud)
            {
                //auto vbo = m_pointCloud->getOrCreateVertexBufferObject();
                //vbo->setArray(1, colors.other);
                m_pointCloud->setColorArray(colors.other);
            }
            if (m_reducedPointCloud)
            {
                //m_reducedPointCloud->getOrCreateVertexBufferObject()->setArray(1, colors.other);
                m_reducedPointCloud->setColorArray(colors.reduced);
            }
        }
    });
}

const CsvPointCloudPlugin *CsvPointCloudPlugin::instance() const
{
    return m_plugin;
}

bool CsvPointCloudPlugin::init()
{
    if (m_plugin)
        return false;
    m_plugin = this;
    m_pointSizeSlider->setValue(coCoviseConfig::getFloat("COVER.Plugin.PointCloud.PointSize", pointSize()));

    coVRFileManager::instance()->registerFileHandler(&handler[0]);
    coVRFileManager::instance()->registerFileHandler(&handler[1]);
    VrmlNamespace::addBuiltIn(MachineNode::defineType());
    return true;
}

CsvPointCloudPlugin::~CsvPointCloudPlugin()
{

    coVRFileManager::instance()->unregisterFileHandler(&handler[0]);
    coVRFileManager::instance()->unregisterFileHandler(&handler[1]);
}

int CsvPointCloudPlugin::load(const char *filename, osg::Group *loadParent, const char *covise_key)
{
    osg::MatrixTransform *t = new osg::MatrixTransform;
    osg::Group *g = new osg::Group;
    loadParent->addChild(t);
    t->addChild(g);
    if (filename)
    {
        g->setName(filename);
    }
    assert(m_plugin);
    m_plugin->m_transform = t;
    m_plugin->createGeodes(g, filename);
    return 1;
}

int CsvPointCloudPlugin::unload(const char *filename, const char *covise_key)
{
    return m_plugin->unloadFile(filename);
}

void setStateSet(osg::Geometry *geo, float pointSize)
{
    // after test move stateset higher up in the tree
    auto *stateset = new StateSet();
    // stateset->setMode(GL_PROGRAM_POINT_SIZE_EXT, StateAttribute::ON);
    stateset->setMode(GL_LIGHTING, StateAttribute::OFF);
    stateset->setMode(GL_DEPTH_TEST, StateAttribute::ON);
    stateset->setMode(GL_ALPHA_TEST, StateAttribute::ON);
    stateset->setMode(GL_BLEND, StateAttribute::OFF);
    AlphaFunc *alphaFunc = new AlphaFunc(AlphaFunc::GREATER, 0.5);
    stateset->setAttributeAndModes(alphaFunc, StateAttribute::ON);

    osg::Point *pointstate = new osg::Point();
    pointstate->setSize(pointSize);
    stateset->setAttributeAndModes(pointstate, StateAttribute::ON);

    osg::PointSprite *sprite = new osg::PointSprite();
    stateset->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

    const char *mapName = opencover::coVRFileManager::instance()->getName("share/covise/icons/particle.png");
    if (mapName != NULL)
    {
        osg::Image *image = osgDB::readImageFile(mapName);
        osg::Texture2D *tex = new osg::Texture2D(image);

        tex->setTextureSize(image->s(), image->t());
        tex->setInternalFormat(GL_RGBA);
        tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
        tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
        stateset->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
        osg::TexEnv *texEnv = new osg::TexEnv;
        texEnv->setMode(osg::TexEnv::MODULATE);
        stateset->setTextureAttributeAndModes(0, texEnv, osg::StateAttribute::ON);

        osg::ref_ptr<osg::TexGen> texGen = new osg::TexGen();
        stateset->setTextureAttributeAndModes(0, texGen.get(), osg::StateAttribute::OFF);
    }
    geo->setStateSet(stateset);
}

bool CsvPointCloudPlugin::compileSymbol(DataTable &symbols, const std::string &symbol, Expression &expr)
{
    expr().register_symbol_table(symbols.symbols());
    if (!expr.parser.compile(symbol, expr()))
    {
        std::cerr << "failed to parse symbol " << symbol << std::endl;
        return false;
    }
    return true;
}

void CsvPointCloudPlugin::readSettings(const std::string &filename)
{
    auto fn = filename.substr(0, filename.find_last_of('.')) + ".txt";
    m_readSettingsTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::fstream f(fn);
    if (!f.is_open())
    {
        std::cerr << "csvPointCloud: could not read settings file " << fn << std::endl;
        return;
    }
    std::string line;

    while (std::getline(f, line))
    {
        std::string name = line.substr(0, line.find_first_of(" "));
        std::string value = line.substr(line.find_first_of('"') + 1, line.find_last_of('"') - line.find_first_of('"') - 1);
        auto setting = std::find_if(m_editFields.begin(), m_editFields.end(), [name](ui::EditField *ef)
                                    { return ef->name() == name; });
        if (setting != m_editFields.end())
            (*setting)->setValue(value);
        else if (name == m_pointSizeSlider->name())
            m_pointSizeSlider->setValue(std::stof(value));
        else if (name == m_numPointsSlider->name())
        {
            m_numPointsSlider->setBounds(0, std::stoi(value));
            m_numPointsSlider->setValue(std::stoi(value));
        }
        else if (name == "AnimationSpeed")
        {
            coVRAnimationManager::instance()->setAnimationSpeed(std::stof(value));
            m_animSpeedSet = true;
        }
        else if (name == "AnimationSkip")
        {
            coVRAnimationManager::instance()->setAnimationSkip(std::stoi(value));
            m_animSkipSet = true;
        }
    }
}

void CsvPointCloudPlugin::writeSettings(const std::string &filename)
{

    auto fn = filename.substr(0, filename.find_last_of('.')) + ".txt";
    // Settings file has changed since last read, don't override
    if (fs::last_write_time(fn) > m_readSettingsTime)
        return;

    std::ofstream f(fn);

    for (const auto ef : m_editFields)
    {
        f << ef->name() << " \"" << ef->value() << "\"\n";
    }
    f << m_pointSizeSlider->name() << " \"" << m_pointSizeSlider->value() << "\"\n";
    f << m_numPointsSlider->name() << " \"" << m_numPointsSlider->value() << "\"\n";
    if (m_animSpeedSet)
        f << "AnimationSpeed"
          << " \"" << coVRAnimationManager::instance()->getAnimationSpeed() << "\"\n";
    if (m_animSkipSet)
        f << "AnimationSkip"
          << " \"" << coVRAnimationManager::instance()->getAnimationSkip() << "\"\n";
}

float parseScale(const std::string &scale)
{
    exprtk::symbol_table<float> symbol_table;
    typedef exprtk::expression<float> expression_t;
    typedef exprtk::parser<float> parser_t;

    std::string expression_string = "z := x - (3 * y)";

    expression_t expression;
    expression.register_symbol_table(symbol_table);

    parser_t parser;

    if (!parser.compile(scale, expression))
    {
        std::cerr << "parseScale failed" << std::endl;
        return 1;
    }
    return expression.value();
}

void CsvPointCloudPlugin::updateColorMap(float min, float max)
{
    osg::Vec3 bottomLeft, hpr, offset;
    if (coVRMSController::instance()->isMaster() && coVRConfig::instance()->numScreens() > 0)
    {
        auto hudScale = covise::coCoviseConfig::getFloat("COVER.Plugin.ColorBar.HudScale", 0.5); // half screen height
        const auto &s0 = coVRConfig::instance()->screens[0];
        hpr = s0.hpr;
        auto sz = osg::Vec3(s0.hsize, 0., s0.vsize);
        osg::Matrix mat;
        MAKE_EULER_MAT_VEC(mat, hpr);
        bottomLeft = s0.xyz - sz * mat * 0.5;
        auto minsize = std::min(s0.hsize, s0.vsize);
        bottomLeft += osg::Vec3(minsize, 0., minsize) * mat * 0.02;
        offset = osg::Vec3(s0.vsize / 2.5, 0, 0) * mat * hudScale;
    }
    
    m_colorBar->setName(m_colorTerm->value().c_str());
    m_colorBar->show(true);
    m_colorBar->update(m_colorTerm->value(), min, max, m_colorMapSelector.selectedMap().a.size(), m_colorMapSelector.selectedMap().r.data(), m_colorMapSelector.selectedMap().g.data(), m_colorMapSelector.selectedMap().b.data(), m_colorMapSelector.selectedMap().a.data());
    m_colorBar->setHudPosition(bottomLeft, hpr, offset[0] / 480);
    m_colorBar->show(true);
}

CsvPointCloudPlugin::Colors CsvPointCloudPlugin::getColors(DataTable &symbols, Expression &reductionCriterium)
{
    Colors colors;
    Expression colorExporession;
    if (!compileSymbol(symbols, m_colorTerm->value(), colorExporession))
        return colors;


    colors.other = new Vec4Array();
    colors.reduced = new Vec4Array();
    std::vector<float> scalarData, reducedScalarData;
    auto scale = parseScale(m_dataScale->value());
    resetMachineSpeed();
    symbols.reset();
    for (size_t i = 0; i < symbols.size(); i++)
    {
        auto scalar = colorExporession().value();
        reductionCriterium().value() ? reducedScalarData.push_back(scalar) : scalarData.push_back(scalar);
        colors.min = std::min(colors.min, scalar);
        colors.max = std::max(colors.max, scalar);
        advanceMachineSpeed(i);
        symbols.advance();
    }
    for (size_t i = 0; i < scalarData.size(); i++)
        colors.other->push_back(m_colorMapSelector.getColor(scalarData[i], colors.min, colors.max));

    for (size_t i = 0; i < reducedScalarData.size(); i++)
        colors.reduced->push_back(m_colorMapSelector.getColor(reducedScalarData[i], colors.min, colors.max));
    updateColorMap(colors.min, colors.max);
    colors.other->setBinding(osg::Array::BIND_PER_VERTEX);
    colors.reduced->setBinding(osg::Array::BIND_PER_VERTEX);
    return colors;
}

CsvPointCloudPlugin::Coords CsvPointCloudPlugin::getCoords(DataTable &symbols, Expression &reductionCriterium)
{
    Coords coords;
    std::array<Expression, 3> coordExpressions;

    for (size_t i = 0; i < coordExpressions.size(); i++)
    {
        if (!compileSymbol(symbols, m_coordTerms[i]->value(), coordExpressions[i]))
            return coords;
    }
    auto scale = parseScale(m_dataScale->value());

    coords.other = new Vec3Array();
    coords.reduced = new Vec3Array();
    m_reducedIndices = std::vector<size_t>();
    symbols.reset();
    resetMachineSpeed();
    for (size_t i = 0; i < symbols.size(); i++)
    {
        osg::Vec3 coord;
        for (size_t j = 0; j < 3; j++)
        {
            coord[j] = coordExpressions[j]().value() * scale;
        }
        if (reductionCriterium().value())
        {
            coords.reduced->push_back(coord);
            m_reducedIndices.push_back(i);
        }
        else
        {
            coords.other->push_back(coord);
        }
        advanceMachineSpeed(i);
        symbols.advance();
    }
    coords.other->setBinding(osg::Array::BIND_PER_VERTEX);
    coords.reduced->setBinding(osg::Array::BIND_PER_VERTEX);
    return coords;
}

void CsvPointCloudPlugin::createOsgPoints(DataTable &symbols, std::ofstream &f)
{
    // compile parser
    cpu_timer timer;

    Expression reduceExpression;

    if (!compileSymbol(symbols, m_pointReductionCriteria->value(), reduceExpression))
        return;

    auto coords = getCoords(symbols, reduceExpression);
    auto colors = getColors(symbols, reduceExpression);
    if (!colors.other || !colors.reduced || !coords.other || !coords.reduced)
        return;
    // write cache file
    if (coVRMSController::instance()->isMaster())
    {
        write(f, coords.other->size());
        write(f, coords.reduced->size());
        write(f, colors.min);
        write(f, colors.max);
        if (coords.other->size())
        {
            f.write((const char *)&(*coords.other)[0], coords.other->size() * sizeof(osg::Vec3));
            f.write((const char *)&(*colors.other)[0], colors.other->size() * sizeof(osg::Vec4));
        }
        if (coords.reduced->size())
        {
            f.write((const char *)&(*coords.reduced)[0], coords.reduced->size() * sizeof(osg::Vec3));
            f.write((const char *)&(*colors.reduced)[0], colors.reduced->size() * sizeof(osg::Vec4));
            f.write((const char *)&m_reducedIndices[0], m_reducedIndices.size() * sizeof(size_t));
        }
    }
    m_pointCloud = createOsgPoints(coords.other, colors.other);
    m_reducedPointCloud = createOsgPoints(coords.reduced, colors.reduced);
}

osg::Geometry *CsvPointCloudPlugin::createOsgPoints(Vec3Array *points, Vec4Array *colors)
{
    // create geometry
    auto geo = new osg::Geometry();
    geo->setUseDisplayList(false);
    geo->setSupportsDisplayList(false);
    geo->setUseVertexBufferObjects(true);
    auto vertexBufferArray = geo->getOrCreateVertexBufferObject();

    vertexBufferArray->setArray(0, points);
    vertexBufferArray->setArray(1, colors);
    // bind color per vertex
    geo->setVertexArray(points);
    geo->setColorArray(colors);
    osg::Vec3Array *normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
    geo->setNormalArray(normals, osg::Array::BIND_OVERALL);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()));
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 0));

    setStateSet(geo, pointSize());

    return geo;
}

std::vector<VrmlSFVec3f> CsvPointCloudPlugin::readMachinePositions(DataTable &symbols)
{

    symbols.reset();
    std::vector<VrmlSFVec3f> retval;
    // compile parser
    std::array<Expression, 3> stringExpressions;
    auto scale = parseScale(m_dataScale->value());

    for (size_t i = 0; i < stringExpressions.size(); i++)
    {
        if (!compileSymbol(symbols, m_machinePositionsTerms[i]->value(), stringExpressions[i]))
            return retval;
    }
    for (size_t i = 0; i < symbols.size(); i++)
    {
        retval.emplace_back(stringExpressions[0]().value() * scale, stringExpressions[1]().value() * scale, stringExpressions[2]().value() * scale);
        symbols.advance();
    }
    return retval;
}

void CsvPointCloudPlugin::createGeodes(Group *parent, const std::string &filename)
{
    readSettings(filename);

    auto pointShader = opencover::coVRShaderList::instance()->get("Points");
    int offset = 0;
    try
    {
        offset = std::stoi(m_offset->value());
    }
    catch (const std::exception &e)
    {
        std::cerr << "header offset must be an integer" << std::endl;
        return;
    }
    size_t size = 0;
    if (auto cache = cacheFileUpToDate(filename))
    {
        std::cerr << "CsvPointCloud: using cache" << std::endl;
        auto unreducedSize = read<size_t>(*cache);
        auto reducedSize = read<size_t>(*cache);
        size = unreducedSize + reducedSize;
        auto min = read<float>(*cache);
        auto max = read<float>(*cache);
        if (unreducedSize)
        {
            auto points = new Vec3Array(unreducedSize);
            auto colors = new Vec4Array(unreducedSize);
            cache->read((char *)&(*points)[0], unreducedSize * sizeof(osg::Vec3));
            cache->read((char *)&(*colors)[0], unreducedSize * sizeof(osg::Vec4));
            m_pointCloud = createOsgPoints(points, colors);
        }
        if (reducedSize)
        {
            auto reducedPoints = new Vec3Array(reducedSize);
            auto reducedColors = new Vec4Array(reducedSize);
            cache->read((char *)&(*reducedPoints)[0], reducedSize * sizeof(osg::Vec3));
            cache->read((char *)&(*reducedColors)[0], reducedSize * sizeof(osg::Vec4));
            m_reducedPointCloud = createOsgPoints(reducedPoints, reducedColors);
            m_reducedIndices.resize(reducedSize);
            cache->read((char *)m_reducedIndices.data(), reducedSize * sizeof(size_t));
        }
        m_machinePositions.resize(size);
        cache->read((char *)m_machinePositions.data(), size * sizeof(vrml::VrmlSFVec3f));
    }
    else
    {
        auto cacheFileName = filename.substr(0, filename.find_last_of('.')) + ".cache";

        std::ofstream f(cacheFileName, std::ios::binary);
        writeCacheFileHeader(f);
        if (!m_dataTable)
        {
            auto binaryFile = filename.substr(0, filename.find_last_of('.')) + ".oct";
            if (filename == binaryFile)
                m_dataTable.reset(new DataTable(binaryFile));
            else
            {
                m_dataTable.reset(new DataTable(filename, m_timeScaleIndicator->value(), m_delimiter->value()[0], offset));
                m_dataTable->writeToFile(binaryFile);
            }
            addMachineSpeedSymbols();
        }
        m_machinePositions = readMachinePositions(*m_dataTable);
        size = m_dataTable->size();
        createOsgPoints(*m_dataTable, f);
        f.write((const char *)m_machinePositions.data(), m_machinePositions.size() * sizeof(vrml::VrmlSFVec3f));
    }
    if (m_pointCloud)
        m_pointCloud->setPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS));
    if (m_reducedPointCloud)
        m_reducedPointCloud->setPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 0));

    m_numPointsSlider->setBounds(0, size);
    if (m_numPointsSlider->value() == 1)
        m_numPointsSlider->setValue(size);
    m_numPointsSlider->setIntegral(true);

    m_currentGeode = new osg::Geode();
    m_currentGeode->setName(filename);
    parent->addChild(m_currentGeode);
    if (!m_pointCloud && !m_reducedPointCloud)
        return;
    if (m_pointCloud)
        m_currentGeode->addDrawable(m_pointCloud);
    if (m_reducedPointCloud)
        m_currentGeode->addDrawable(m_reducedPointCloud);
    if (pointShader != nullptr)
    {
        if (m_pointCloud)
            pointShader->apply(m_currentGeode, m_pointCloud);

        if (m_reducedPointCloud)
            pointShader->apply(m_currentGeode, m_reducedPointCloud);
    }
    coVRAnimationManager::instance()->setNumTimesteps(size, this);
}

void CsvPointCloudPlugin::addMachineSpeedSymbols()
{
    for (size_t i = 0; i < m_machineSpeed.size(); i++)
    {
        m_dataTable->symbols().add_variable(m_machineSpeed[i].first, m_machineSpeed[i].second);
    }
}

void CsvPointCloudPlugin::setTimestep(int t)
{
    static int lastTimestep = 0;
    static size_t reducedPointsUntilTimestep = 0;
    static size_t reducedPointsBetween = 0;
    static size_t lastNumFullDrawnPoints = 0;
    // show points until t

    if (lastTimestep > t)
    {
        reducedPointsUntilTimestep = 0;
        reducedPointsBetween = 0;
    }
    if (lastNumFullDrawnPoints != (size_t)m_numPointsSlider->value())
        reducedPointsBetween = 0;

    for (;; ++reducedPointsUntilTimestep)
    {
        if (reducedPointsUntilTimestep >= m_reducedIndices.size() || m_reducedIndices[reducedPointsUntilTimestep] > t)
            break;
    }
    size_t start = std::max(ui::Slider::ValueType{0}, t - m_numPointsSlider->value());
    for (;; ++reducedPointsBetween)
    {
        if (reducedPointsBetween >= m_reducedIndices.size() || m_reducedIndices[reducedPointsBetween] > start)
            break;
    }
    auto normalPointsUntilTimestep = t - reducedPointsUntilTimestep + 1;
    auto normalPointsUntilStart = start < reducedPointsBetween ? 0 : start - reducedPointsBetween;
    auto nomalPointsToDraw = normalPointsUntilTimestep - normalPointsUntilStart;
    if (m_pointCloud)
    {
        static_cast<osg::DrawArrays *>(m_pointCloud->getPrimitiveSet(0))->setFirst(normalPointsUntilStart);
        static_cast<osg::DrawArrays *>(m_pointCloud->getPrimitiveSet(0))->setCount(nomalPointsToDraw);
    }
    if (m_reducedPointCloud)
        static_cast<osg::DrawArrays *>(m_reducedPointCloud->getPrimitiveSet(0))->setCount(reducedPointsUntilTimestep);

    // move machine axis
    if (m_machinePositions.size() > t)
    {
        for (auto machineNode : machineNodes)
        {
            machineNode->move(m_machinePositions[t]);
        }
        // move the workpiece with the machine table
        osg::Vec3 v{-m_machinePositions[t].x(), 0, 0};
        osg::Matrix m;
        m.makeTranslate(v);
        if (m_transform)
            m_transform->setMatrix(m);
    }
    lastTimestep = t;
    lastNumFullDrawnPoints = m_numPointsSlider->value();
}

float CsvPointCloudPlugin::pointSize() const
{
    return m_pointSizeSlider->value();
}

int CsvPointCloudPlugin::unloadFile(const std::string &filename)
{
    if (m_currentGeode && m_currentGeode->getNumParents() > 0)
    {
        writeSettings(filename);
        m_currentGeode->getParent(0)->removeChild(m_currentGeode);
        m_pointCloud = nullptr;
        m_reducedPointCloud = nullptr;
        m_currentGeode = nullptr;
        m_transform = nullptr;
        for (size_t i = 0; i < m_machineSpeed.size(); i++)
            m_machineSpeed[i].second = 0;

        return 0;
    }
    return -1;
}

std::unique_ptr<std::ifstream> CsvPointCloudPlugin::cacheFileUpToDate(const std::string &filename)
{
    auto settingsFileName = filename.substr(0, filename.find_last_of('.')) + ".txt";
    auto cacheFileName = filename.substr(0, filename.find_last_of('.')) + ".cache";
    if (fs::exists(cacheFileName) && fs::last_write_time(cacheFileName) > fs::last_write_time(filename))
    {
        std::unique_ptr<std::ifstream> f{new std::ifstream{cacheFileName, std::ios::binary}};
        auto date = readString(*f);
        auto time = readString(*f);
        // if (date != __DATE__ || time != __TIME__)
        //     return nullptr;
        for (const auto editField : m_editFields)
        {
            auto s = readString(*f);
            if (s != editField->value())
                return nullptr;
        }
        return f;
    }
    return nullptr;
}

void CsvPointCloudPlugin::writeCacheFileHeader(std::ofstream &f)
{
    writeString(f, __DATE__);
    writeString(f, __TIME__);
    for (const auto editField : m_editFields)
    {
        writeString(f, editField->value());
    }
}

void CsvPointCloudPlugin::resetMachineSpeed()
{
    for (size_t i = 0; i < 3; i++)
    {
        m_machineSpeed[i].second = 0;
    }
}

void CsvPointCloudPlugin::advanceMachineSpeed(size_t i)
{
    if (i > 0)
    {
        auto speed = m_machinePositions[i];
        speed.subtract(&m_machinePositions[i - 1]);
        m_machineSpeed[0].second = speed.x();
        m_machineSpeed[1].second = speed.y();
        m_machineSpeed[2].second = speed.z();
    }
}