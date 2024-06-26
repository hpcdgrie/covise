# @file
# 
# CMakeLists.txt for application general
#
# @author Blasius Czink

PROJECT(general CXX C)

#ADD_SUBDIRECTORY(ReadTecplot)
ADD_SUBDIRECTORY(AddAttribute)
ADD_SUBDIRECTORY(ReadAtomProbe)
ADD_SUBDIRECTORY(ReadFieldView)
ADD_SUBDIRECTORY(ReadCalculix)
ADD_SUBDIRECTORY(ReadCSVPoly)
ADD_SUBDIRECTORY(ReadCGNS)
ADD_SUBDIRECTORY(ReadUFFData)
ADD_SUBDIRECTORY(Animate)
ADD_SUBDIRECTORY(AssembleUsg)
ADD_SUBDIRECTORY(BezierGrid)
ADD_SUBDIRECTORY(BlockCollect)
ADD_SUBDIRECTORY(BoundingBox)
ADD_SUBDIRECTORY(Calc)
ADD_SUBDIRECTORY(CellToVert)
ADD_SUBDIRECTORY(CheckObj)
ADD_SUBDIRECTORY(ClipInterval)
ADD_SUBDIRECTORY(Collect)
ADD_SUBDIRECTORY(ColorDistance)
ADD_SUBDIRECTORY(Colors)
ADD_SUBDIRECTORY(PredefinedColors)
ADD_SUBDIRECTORY(ComputeGrid)
ADD_SUBDIRECTORY(ComputeTrace)
ADD_SUBDIRECTORY(CropUsg)
ADD_SUBDIRECTORY(CoverDocument)
ADD_SUBDIRECTORY(CutGeometry)
ADD_SUBDIRECTORY(CuttingLine)
ADD_SUBDIRECTORY(CuttingSurface)
ADD_SUBDIRECTORY(CuttingSurfaceComp)
ADD_SUBDIRECTORY(DataToGrid)
ADD_SUBDIRECTORY(DerivOperators)
ADD_SUBDIRECTORY(PressureLevels)
ADD_SUBDIRECTORY(DisplaceUsg)
ADD_SUBDIRECTORY(DomainSurface)
ADD_SUBDIRECTORY(EliminatePolygons)
ADD_SUBDIRECTORY(FilterCrop)
ADD_SUBDIRECTORY(FixPart)
ADD_SUBDIRECTORY(FixUsg)
ADD_SUBDIRECTORY(FreeCut)
ADD_SUBDIRECTORY(GenDat)
ADD_SUBDIRECTORY(GenNormals)
ADD_SUBDIRECTORY(GetSetelem)
ADD_SUBDIRECTORY(GetSubset)
ADD_SUBDIRECTORY(Gnuplot)
ADD_SUBDIRECTORY(GridSurface)
ADD_SUBDIRECTORY(GridToData)
ADD_SUBDIRECTORY(IndexManifolds)
ADD_SUBDIRECTORY(Interpolate)
ADD_SUBDIRECTORY(IsoLines)
ADD_SUBDIRECTORY(IsoSurface)
ADD_SUBDIRECTORY(IsoSurfaceComp)
ADD_SUBDIRECTORY(LineReduce)
ADD_SUBDIRECTORY(LoadCadData)
ADD_SUBDIRECTORY(MakeOctTree)
ADD_SUBDIRECTORY(MakePlots)
ADD_SUBDIRECTORY(MakeTransient)
ADD_SUBDIRECTORY(MapDrape)
ADD_SUBDIRECTORY(Material)
ADD_SUBDIRECTORY(MeanValues)
ADD_SUBDIRECTORY(MergeSets)
ADD_SUBDIRECTORY(MinMax)
ADD_SUBDIRECTORY(OceanDepth)
ADD_SUBDIRECTORY(PartSelect)
ADD_SUBDIRECTORY(ParticleTrails)
ADD_SUBDIRECTORY(PerformerScene)
ADD_SUBDIRECTORY(PipelineCollect)
ADD_SUBDIRECTORY(PointsToLine)
ADD_SUBDIRECTORY(Probe3D)
ADD_SUBDIRECTORY(PurgeCoords)
ADD_SUBDIRECTORY(RadialScale)
ADD_SUBDIRECTORY(RandomNormals)
ADD_SUBDIRECTORY(RawConverter)
ADD_SUBDIRECTORY(ReadANSYS)
ADD_SUBDIRECTORY(ReadASCII)
ADD_SUBDIRECTORY(ReadCadmould)
ADD_SUBDIRECTORY(ReadCOMSOLData)
ADD_SUBDIRECTORY(ReadDyna3D)
ADD_SUBDIRECTORY(ReadEnsight)
ADD_SUBDIRECTORY(ReadFidap)
ADD_SUBDIRECTORY(ReadFamu)
ADD_SUBDIRECTORY(ReadFluent)
ADD_SUBDIRECTORY(ReadIv)
ADD_SUBDIRECTORY(ReadLat)
ADD_SUBDIRECTORY(ReadKiva)
ADD_SUBDIRECTORY(ReadModel)
ADD_SUBDIRECTORY(ReadMovieBYU)
ADD_SUBDIRECTORY(ReadN3s)
ADD_SUBDIRECTORY(ReadNas)
ADD_SUBDIRECTORY(ReadNasASC)
ADD_SUBDIRECTORY(ReadNastran)
ADD_SUBDIRECTORY(ReadObj)
ADD_SUBDIRECTORY(ReadPatran)
#ADD_SUBDIRECTORY(ReadPlot3D)
ADD_SUBDIRECTORY(ReadPLY)
ADD_SUBDIRECTORY(ReadPTV)
ADD_SUBDIRECTORY(ReadRFL)
ADD_SUBDIRECTORY(ReadStar)
ADD_SUBDIRECTORY(ReadStarDrop)
ADD_SUBDIRECTORY(ReadStl)
ADD_SUBDIRECTORY(ReadTascflowTDI)
ADD_SUBDIRECTORY(ReadTrajectory)
ADD_SUBDIRECTORY(ReadTrajectorySimple)
ADD_SUBDIRECTORY(ReadTRK)
ADD_SUBDIRECTORY(ReadVTF)
ADD_SUBDIRECTORY(ReduceSet)
ADD_SUBDIRECTORY(RWCoviseASCII)
ADD_SUBDIRECTORY(RW_AVS_TriMesh)
ADD_SUBDIRECTORY(ReadCalib)
ADD_SUBDIRECTORY(ReducePoints)
ADD_SUBDIRECTORY(RWCovise)
ADD_SUBDIRECTORY(RWCoviseGroup)
ADD_SUBDIRECTORY(Sample)
ADD_SUBDIRECTORY(Scalar2Vector)
ADD_SUBDIRECTORY(SelectUsg)
ADD_SUBDIRECTORY(SortPoints)
ADD_SUBDIRECTORY(ShiftValues)
ADD_SUBDIRECTORY(ShowFaceNormal)
ADD_SUBDIRECTORY(ShowGrid)
ADD_SUBDIRECTORY(ShowUsg)
ADD_SUBDIRECTORY(SimplifySurface)
ADD_SUBDIRECTORY(SmoothSurface)
ADD_SUBDIRECTORY(Solutions)
ADD_SUBDIRECTORY(Sphere)
ADD_SUBDIRECTORY(SplitGeometry)
ADD_SUBDIRECTORY(SplitUsg)
ADD_SUBDIRECTORY(StackSlices)
ADD_SUBDIRECTORY(StoU)
ADD_SUBDIRECTORY(StretchSet)
ADD_SUBDIRECTORY(SwitchData)
ADD_SUBDIRECTORY(TensorScal)
ADD_SUBDIRECTORY(TGenDat)
ADD_SUBDIRECTORY(Tracer)
ADD_SUBDIRECTORY(TracerComp)
ADD_SUBDIRECTORY(Transform)
ADD_SUBDIRECTORY(TriArea)
ADD_SUBDIRECTORY(Tube)
ADD_SUBDIRECTORY(VectorField)
ADD_SUBDIRECTORY(VectorScal)
ADD_SUBDIRECTORY(WriteASC)
ADD_SUBDIRECTORY(WriteCGNS)
ADD_SUBDIRECTORY(WriteObj)
ADD_SUBDIRECTORY(WritePatran)
ADD_SUBDIRECTORY(WritePolygon)
ADD_SUBDIRECTORY(VariantMarker)
ADD_SUBDIRECTORY(IsoCutter)
ADD_SUBDIRECTORY(PolyBoundaryLines)
ADD_SUBDIRECTORY(ReadCFX)
ADD_SUBDIRECTORY(ReadNetCDF)
ADD_SUBDIRECTORY(ReadFoam)
ADD_SUBDIRECTORY(ReadAbaqus)
ADD_SUBDIRECTORY(ReadVTK)
ADD_SUBDIRECTORY(Triangulate)
ADD_SUBDIRECTORY(ReadITK)
ADD_SUBDIRECTORY(Register)
ADD_SUBDIRECTORY(WriteITKSlices)
ADD_SUBDIRECTORY(WriteVTK)
ADD_SUBDIRECTORY(ImageToTexture)
ADD_SUBDIRECTORY(ReadPAM)
ADD_SUBDIRECTORY(ReadBIFBOF)

if (COVISE_USE_VIRVO)
   ADD_SUBDIRECTORY(ReadVolume)
   ADD_SUBDIRECTORY(VolumeCollect)
   ADD_SUBDIRECTORY(WriteVolume)
endif()
