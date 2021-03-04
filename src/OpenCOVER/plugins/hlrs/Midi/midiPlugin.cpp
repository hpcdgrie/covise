/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifdef WIN32
#include <SDKDDKVer.h>
#include <winsock2.h>
#include <windows.h>
#include <direct.h>
#include <stdio.h>
#include <conio.h>
#include <mmsystem.h>
#endif
#include "cover/OpenCOVER.h"
#include <cover/coVRPluginSupport.h>
#include <cover/coVRConfig.h>
#include <cover/coVRFileManager.h>
#include <cover/coVRMSController.h>
#include <PluginUtil/coSphere.h>
#include <cover/coVRTui.h>
#include <config/CoviseConfig.h>
#include "midiPlugin.h"
#include <cover/coVRShader.h>
#include <cover/VRSceneGraph.h>

#include <osg/GL>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Switch>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/CullFace>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/Depth>
#include <osgDB/ReadFile>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Point>
#include <osg/ShadeModel>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/LineWidth>

#include <OpenVRUI/coRowMenu.h>
#include <OpenVRUI/coSubMenuItem.h>
#include <OpenVRUI/coCheckboxMenuItem.h>
#include <OpenVRUI/coCheckboxGroup.h>
#include <OpenVRUI/coButtonMenuItem.h>
#include <OpenVRUI/coSliderMenuItem.h>

#include <osgGA/GUIEventAdapter>
#include <SDL_audio.h>

#ifndef WIN32
 // for chdir
#include <unistd.h>
#endif

#ifdef _WINDOWS
#include <direct.h>
#include <mmeapi.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
//Maximum number of supported recording devices
const int MAX_RECORDING_DEVICES = 20;

//Maximum recording time
const int MAX_RECORDING_SECONDS = 5;

//Maximum recording time plus padding
const int RECORDING_BUFFER_SECONDS = MAX_RECORDING_SECONDS + 1;
/*
class Vector3d {  // this is a pretty standard vector class
public:
	double x, y, z;
	...
}

void subdivide(const Vector3d &v1, const Vector3d &v2, const Vector3d &v3, vector<Vector3d> &sphere_points, const unsigned int depth) {
	if (depth == 0) {
		sphere_points.push_back(v1);
		sphere_points.push_back(v2);
		sphere_points.push_back(v3);
		return;
	}
	const Vector3d v12 = (v1 + v2).norm();
	const Vector3d v23 = (v2 + v3).norm();
	const Vector3d v31 = (v3 + v1).norm();
	subdivide(v1, v12, v31, sphere_points, depth - 1);
	subdivide(v2, v23, v12, sphere_points, depth - 1);
	subdivide(v3, v31, v23, sphere_points, depth - 1);
	subdivide(v12, v23, v31, sphere_points, depth - 1);
}

void initialize_sphere(vector<Vector3d> &sphere_points, const unsigned int depth) {
	const double X = 0.525731112119133606;
	const double Z = 0.850650808352039932;
	const Vector3d vdata[12] = {
		{-X, 0.0, Z}, { X, 0.0, Z }, { -X, 0.0, -Z }, { X, 0.0, -Z },
		{ 0.0, Z, X }, { 0.0, Z, -X }, { 0.0, -Z, X }, { 0.0, -Z, -X },
		{ Z, X, 0.0 }, { -Z, X, 0.0 }, { Z, -X, 0.0 }, { -Z, -X, 0.0 }
	};
	int tindices[20][3] = {
		{0, 4, 1}, { 0, 9, 4 }, { 9, 5, 4 }, { 4, 5, 8 }, { 4, 8, 1 },
		{ 8, 10, 1 }, { 8, 3, 10 }, { 5, 3, 8 }, { 5, 2, 3 }, { 2, 7, 3 },
		{ 7, 10, 3 }, { 7, 6, 10 }, { 7, 11, 6 }, { 11, 0, 6 }, { 0, 1, 6 },
		{ 6, 1, 10 }, { 9, 0, 11 }, { 9, 11, 2 }, { 9, 2, 5 }, { 7, 2, 11 }
	};
	for (int i = 0; i < 20; i++)
		subdivide(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], sphere_points, depth);
}*/
AudioInStream::AudioInStream(std::string deviceName)
{

	if (coVRMSController::instance()->isMaster())
	{

		SDL_memset(&want, 0, sizeof(want)); /* or SDL_zero(want) */
		want.freq = 48000;
		want.format = AUDIO_F32;
		want.channels = 2;
		want.samples = 1024;
		//want.samples = 4096;
		want.callback = readData; /* you wrote this function elsewhere -- see SDL_AudioSpec for details */
		want.userdata = this;
		const char *devName = NULL;
		if (deviceName.length() > 0)
			devName = deviceName.c_str();

		dev = SDL_OpenAudioDevice(devName, 1, &want, &have, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
		if (dev == 0) {
			SDL_Log("Failed to open audio: %s", SDL_GetError());
		}
		else {
			if (have.format != want.format) { /* we let this one thing change. */
				SDL_Log("We didn't get Float32 audio format.");
			}

			//Calculate per sample bytes
			bytesPerSample = have.channels * (SDL_AUDIO_BITSIZE(have.format) / 8);

			//Calculate bytes per second
			int bytesPerSecond = have.freq * bytesPerSample;

			//Calculate buffer size
			gBufferByteSize = RECORDING_BUFFER_SECONDS * bytesPerSecond;

			//Calculate max buffer use
			gBufferByteMaxPosition = MAX_RECORDING_SECONDS * bytesPerSecond;

			//Allocate and initialize byte buffer
			gRecordingBuffer = new Uint8[gBufferByteSize];
			memset(gRecordingBuffer, 0, gBufferByteSize);

			SDL_PauseAudioDevice(dev, SDL_FALSE); /* start audio playing. */
		}
	}
	inputSize = BINSIZE;
	outputSize = inputSize / 2 + 1;

	ddata = new double[inputSize];
	magnitudes = new double[outputSize];
	SDL_memset(ddata, 0, inputSize * sizeof(double));
	SDL_memset(magnitudes, 0, outputSize * sizeof(double));

	ifft_result = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * outputSize);
	SDL_memset(ifft_result, 0, outputSize * sizeof(fftw_complex));

	plan = fftw_plan_dft_r2c_1d(BINSIZE, ddata, ifft_result, FFTW_ESTIMATE);

}
AudioInStream::~AudioInStream()
{
	SDL_CloseAudioDevice(dev);
	delete[] gRecordingBuffer;
	delete[] ddata;
	delete[] magnitudes;
	fftw_destroy_plan(plan);
	fftw_free(ifft_result);
}

void AudioInStream::update()
{
	int bytesProcessed = 0;
	if (have.format == AUDIO_F32LSB && bytesPerSample >0)
	{
#ifdef WIN32
		while (((gBufferBytePosition - bytesProcessed) / bytesPerSample >= inputSize))
#else
		if (((gBufferBytePosition - bytesProcessed) / bytesPerSample >= inputSize))
#endif
		{
			int sample = 0;
			for (; (bytesProcessed < gBufferBytePosition && sample < inputSize); bytesProcessed += bytesPerSample)
			{
				//Hanning window
				double m = 0.5 * (1 - cos(2 * M_PI*sample / (inputSize - 1)));
				ddata[sample] = m * (*((float*)&(gRecordingBuffer[bytesProcessed])));
				sample++;

			}
			fftw_execute(plan);
			for (int i = 0; i < outputSize; i++) {
				magnitudes[i] = sqrt((ifft_result[i][0] * ifft_result[i][0]) + (ifft_result[i][1] * ifft_result[i][1]));// mag=sqrt(real^2+img^2)
			}
		}
	}
	//fprintf(stderr,"%d %d\n",gBufferBytePosition,bytesProcessed);
	if (coVRMSController::instance()->isMaster())
	{
		coVRMSController::instance()->sendSlaves((char *)ddata, inputSize * sizeof(double));

	}
	else
	{
		coVRMSController::instance()->readMaster((char *)ddata, inputSize * sizeof(double));
		fftw_execute(plan);
		for (int i = 0; i < outputSize; i++) {
			magnitudes[i] = sqrt((ifft_result[i][0] * ifft_result[i][0]) + (ifft_result[i][1] * ifft_result[i][1]));// mag=sqrt(real^2+img^2)
		}
	}
	if (bytesProcessed > 0)
	{

		memmove(gRecordingBuffer, gRecordingBuffer + bytesProcessed, gBufferBytePosition - bytesProcessed);
		gBufferBytePosition -= bytesProcessed;
	}
	//fprintf(stderr, "%03.3f\n", (float)magnitudes[20]);

}

void AudioInStream::readData(Uint8 * stream, int len)
{
	//fprintf(stderr, "read %d\n", len);
	if ((gBufferBytePosition + len) < gBufferByteSize)
	{
		//Copy audio from stream
		memcpy(&gRecordingBuffer[gBufferBytePosition], stream, len);

		//Move along buffer
		gBufferBytePosition += len;
	}
}

void AudioInStream::readData(void *userdata, Uint8 * stream, int len)
{
	AudioInStream * as = (AudioInStream *)userdata;
	as->readData(stream, len);
}

class MIDIMessage
{
public:
	int			mTime;
	unsigned char		mStatus;
	unsigned char		mParam1;
	unsigned char		mParam2;
};
#ifdef WIN32
void CALLBACK MidiInProc(HMIDIIN hMidiIn, UINT wMsg, DWORD dwInstance, DWORD dwParam1, DWORD dwParam2)
{
	switch (wMsg) {
	case MIM_OPEN:
		printf("wMsg=MIM_OPEN\n");
		break;
	case MIM_CLOSE:
		printf("wMsg=MIM_CLOSE\n");
		break;
	case MIM_MOREDATA:
	case MIM_DATA:
	{
		//printf("wMsg=MIM_DATA, dwInstance=%08x, dwParam1=%08x, dwParam2=%08x\n", dwInstance, dwParam1, dwParam2);
		MIDIMessage md;
		md.mTime = dwParam2;
		md.mStatus = (UCHAR)(dwParam1 & 0xFF);
		md.mParam1 = (UCHAR)((dwParam1 >> 8) & 0xFF);
		md.mParam2 = (UCHAR)((dwParam1 >> 16) & 0xFF);
		MidiEvent me(md.mStatus, md.mParam1, md.mParam2);

		TokenBuffer tb;
		tb << int(dwInstance);
		tb << me.getP0();
		tb << me.getP1();
		tb << me.getP2();
		tb << me.getP3();
		covise::UdpMessage um(tb, covise::MIDI_STREAM);
		if(me.isNoteOn() || me.isNoteOff() || (me.isController() && me.getP1()>2))
		{
		cover->sendVrbMessage(&um);
		}
		MidiPlugin::instance()->addEvent(me, dwInstance);
		if (MidiPlugin::instance()->hMidiDeviceOut != NULL)
		{
			//midiInMessage(MidiPlugin::plugin->hMidiDeviceOut,wMsg,dwParam1,dwParam2);
			int flag;
			flag = midiOutShortMsg(MidiPlugin::instance()->hMidiDeviceOut, dwParam1);
			if (flag != MMSYSERR_NOERROR) {
				printf("Warning: MIDI Output is not open.\n");
			}
			if (!((dwParam1 & 0xff) == 0x99 || (dwParam1 & 0xff) == 0x89 || (dwParam1 & 0xff) == 0xfe || (dwParam1 & 0xff) == 0xf8))
			{
				if (wMsg == MIM_MOREDATA)
					printf("wMsg=MIM_MOREDATA, dwInstance=%08x, dwParam1=%08x, dwParam2=%08x\n", dwInstance, dwParam1, dwParam2);
				//else
					//printf("dwParam1=%08x\n", (dwParam1 & 0xff) == 0xfe);
				//printf("wMsg=MIM_DATA, dwInstance=%08x, dwParam1=%08x, dwParam2=%08x\n", dwInstance, dwParam1, dwParam2);
			}

		}
	}
	break;
	case MIM_LONGDATA:
		printf("wMsg=MIM_LONGDATA\n");
		break;
	case MIM_ERROR:
		printf("wMsg=MIM_ERROR\n");
		break;
	case MIM_LONGERROR:
		printf("wMsg=MIM_LONGERROR\n");
		break;
		printf("wMsg=MIM_MOREDATA\n");
		break;
	default:
		printf("wMsg = unknown\n");
		break;
	}
	return;
}
#endif
MidiPlugin *MidiPlugin::plugin = NULL;

void playerUnavailableCB()
{
	MidiPlugin::instance()->player = NULL;
}


static FileHandler handlers[] = {
	{ NULL,
	  MidiPlugin::loadMidi,
	  MidiPlugin::loadMidi,
	  MidiPlugin::unloadMidi,
	  "MID" },
	{ NULL,
	  MidiPlugin::loadMidi,
	  MidiPlugin::loadMidi,
	  MidiPlugin::unloadMidi,
	  "mid" }
};

void MidiPlugin::UDPmessage(UdpMessage* msg)
{
	if(msg->type== covise::MIDI_STREAM)
	{
	MidiEvent me;
	int dummy;
	int channel;
	TokenBuffer tb(msg);
	tb >> channel;
	me.setChannel(channel);
	tb >> dummy;
	me.setP0(dummy);
	tb >> dummy;
	me.setP1(dummy);
	tb >> dummy;
	me.setP2(dummy);
	//tb >> dummy;
	//me.setP3(dummy);
	fprintf(stderr, "received UDP Message %d %d", me.isNoteOn(), me.isNoteOff());
	addEvent(me, channel);
	}
	else
	{
		fprintf(stderr, "received UDP Message %d ", msg->type);
	}
}
int MidiPlugin::loadMidi(const char *filename, osg::Group *parent, const char *)
{

	cerr << "Read the file " << filename << endl;
	return plugin->loadFile(filename, parent);
}

int MidiPlugin::unloadMidi(const char *filename, const char *)
{

	cerr << "unload the file " << filename << endl;
	return plugin->unloadFile();
}

int MidiPlugin::loadFile(const char *filename, osg::Group *parent)
{

	midifile.read(filename);
	tempo = 60.0;

	int numTracks = midifile.getTrackCount();
	cout << "TPQ: " << midifile.getTicksPerQuarterNote() << endl;
	char text[1000];
	snprintf(text, 1000, "numTracks = %d", numTracks);
	infoLabel->setText(text);
	if (numTracks > 1) {
		for (int i = 0; i < midifile.getNumEvents(0); i++) {

			// check for tempo indication
			if (midifile[0][i][0] == 0xff &&
				midifile[0][i][1] == 0x51) {
				setTempo(i);
			}
		}
		//fprintf(stderr,"Tempo %f\n",tempo);


	}
	for (int track = 1; track < numTracks; track++) {
		tracks.push_back(new Track(track));
		/* if (tracks > 1) {
			cout << "\nTrack " << track << endl;
		 }
		 for (int event=0; event < midifile[track].size(); event++) {
			cout << dec << midifile[track][event].tick;
			cout << '\t' << hex;
			for (int i=0; i<midifile[track][event].size(); i++) {
			   cout << (int)midifile[track][event][i] << ' ';
			}
			cout << endl;
		 }*/
	}
	return 1;
}

int MidiPlugin::unloadFile()
{

	return 0;
}

//------------------------------------------------------------------------------
void MidiPlugin::key(int type, int keySym, int mod)
{
	if (type == osgGA::GUIEventAdapter::KEYDOWN)
	{
		MidiEvent me;
		me.makeNoteOn(11, keySym, 1);

		TokenBuffer tb;
		tb << (int)0;
		tb << me.getP0();
		tb << me.getP1();
		tb << me.getP2();
		tb << me.getP3();
		UdpMessage um(tb, covise::MIDI_STREAM);
		cover->sendVrbMessage(&um);
		addEvent(me, 0);
		fprintf(stdout,"--- coVRKey called (KeyPress, keySym=%d, mod=%d)\n",
			keySym,mod);
		return;
		//}else{
		//fprintf(stdout,"--- coVRKey called (KeyRelease, keySym=%d)\n",keySym);
	}
	else if (type == osgGA::GUIEventAdapter::KEYUP)
	{
		MidiEvent me;
		me.makeNoteOff(11, keySym, 0);
 		TokenBuffer tb;
		tb << (int)0;
		tb << me.getP0();
		tb << me.getP1();
		tb << me.getP2();
		tb << me.getP3();
		UdpMessage um(tb, covise::MIDI_STREAM);
		cover->sendVrbMessage(&um);
		addEvent(me, 0);
		fprintf(stdout, "--- coVRKey called (KeyPress, keySym=%d, mod=%d)\n",
			keySym, mod);
	}

	switch (keySym)
	{

	case ('r'): /* r: reset animation */
		break;

	}
}

//-----------------------------------------------------------------------------
MidiPlugin::MidiPlugin()
	: ui::Owner("MidiPlugin", cover->ui)
{
	plugin = this;
	player = NULL;

	MIDITab = NULL;
	startTime = 0;
	//Initialize SDL
	
    udp = new UDPComm("localhost", 51322, 51324);
	if (coVRMSController::instance()->isMaster())
	{
		gRecordingDeviceCount = -1;
		if (SDL_Init(SDL_INIT_AUDIO) < 0)
		{
			printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
		}
		else
		{
			//Get capture device count
			gRecordingDeviceCount = SDL_GetNumAudioDevices(SDL_TRUE);

			//No recording devices
			if (gRecordingDeviceCount < 1)
			{
				printf("Unable to get audio capture device! SDL Error: %s\n", SDL_GetError());
			}
			//At least one device connected
			else
			{
				//Cap recording device count
				if (gRecordingDeviceCount > MAX_RECORDING_DEVICES)
				{
					gRecordingDeviceCount = MAX_RECORDING_DEVICES;
				}
				//Render device names
				for (int i = 0; i < gRecordingDeviceCount; ++i)
				{
					osg::MatrixTransform *mt = new osg::MatrixTransform();
					mt->setMatrix(osg::Matrix::rotate(0, 0, 1, 0));
					cover->getObjectsRoot()->addChild(mt);
					osg::MatrixTransform *mt2 = new osg::MatrixTransform();
					mt2->setMatrix(osg::Matrix::rotate(M_PI, 0, 1, 0));
					cover->getObjectsRoot()->addChild(mt2);
					fprintf(stderr, "AudioDevice %d:%s\n", i, (SDL_GetAudioDeviceName(i, SDL_TRUE)));
					//Get capture device name
					AudioInStream *stream = new AudioInStream(SDL_GetAudioDeviceName(i, SDL_TRUE));
					audioStreams.push_back(stream);
					frequencySurface = new FrequencySurface(mt2, stream);
					frequencySurface->setType(FrequencySurface::SurfaceCylinder);
					waveSurfaces.push_back(frequencySurface);
					amplitudeSurface = new AmplitudeSurface(mt, stream);
					amplitudeSurface->setType(FrequencySurface::SurfaceCylinder);
					waveSurfaces.push_back(amplitudeSurface);
				}
			}
		}
		coVRMSController::instance()->sendSlaves((char *)&gRecordingDeviceCount, sizeof(gRecordingDeviceCount));
	}
	if (!coVRMSController::instance()->isMaster())
	{
		coVRMSController::instance()->readMaster((char *)&gRecordingDeviceCount, sizeof(gRecordingDeviceCount));
		for (int i = 0; i < gRecordingDeviceCount; ++i)
		{

			osg::MatrixTransform *mt = new osg::MatrixTransform();
			mt->setMatrix(osg::Matrix::rotate(0, 0, 1, 0));
			cover->getObjectsRoot()->addChild(mt);
			osg::MatrixTransform *mt2 = new osg::MatrixTransform();
			mt2->setMatrix(osg::Matrix::rotate(M_PI, 0, 1, 0));
			cover->getObjectsRoot()->addChild(mt2);
			//Get capture device name
			AudioInStream *stream = new AudioInStream("slaveName");
			audioStreams.push_back(stream);
			frequencySurface = new FrequencySurface(mt2, stream);
			frequencySurface->setType(FrequencySurface::SurfaceCylinder);
			waveSurfaces.push_back(frequencySurface);
			amplitudeSurface = new AmplitudeSurface(mt, stream);
			amplitudeSurface->setType(FrequencySurface::SurfaceCylinder);
			waveSurfaces.push_back(amplitudeSurface);
		}

	}

	globalmtl = new osg::Material;
	globalmtl->ref();
	globalmtl->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	globalmtl->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2f, 0.2f, 0.2f, 1.0));
	globalmtl->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.9f, 0.9f, 0.0f, 1.0));
	globalmtl->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.9f, 0.9f, 0.9f, 1.0));
	globalmtl->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0));
	globalmtl->setShininess(osg::Material::FRONT_AND_BACK, 16.0f);

	shadedStateSet = new osg::StateSet();
	shadedStateSet->ref();
	shadedStateSet->setAttributeAndModes(globalmtl.get(), osg::StateAttribute::ON);
	shadedStateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	shadedStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
	//for transparency, we need a transparent bin
	shadedStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	shadedStateSet->setNestRenderBins(false);
	shadeModel = new osg::ShadeModel;
	shadeModel->setMode(osg::ShadeModel::SMOOTH);
	shadedStateSet->setAttributeAndModes(shadeModel, osg::StateAttribute::ON);

	lineStateSet = new osg::StateSet();
	lineStateSet->ref();
	lineStateSet->setAttributeAndModes(globalmtl.get(), osg::StateAttribute::ON);
	lineStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	lineStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF);
	lineStateSet->setNestRenderBins(false);
	//lineStateSet->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	lineStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::LineWidth *lineWidth = new osg::LineWidth(4);
	lineStateSet->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

}
MidiPlugin * MidiPlugin::instance()
{
	return plugin;
}


NoteInfo::NoteInfo(int nN)
{
	noteNumber = nN;
}
void NoteInfo::createGeom()
{
	if (modelName.length() > 0)
	{
		osg::MatrixTransform* mt = new osg::MatrixTransform();
		mt->setName(modelName.c_str());
		mt->setMatrix(osg::Matrix::scale(modelScale, modelScale, modelScale) * osg::Matrix::rotate(M_PI_2, 0, 0, 1));
		mt->addChild(osgDB::readNodeFile(modelName.c_str()));
		osg::StateSet *geoState = mt->getOrCreateStateSet();

	osg::Material *colorMaterial = new osg::Material;
		colorMaterial->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
		colorMaterial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2f, 0.2f, 0.2f, 1.0));
		colorMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, color);
		colorMaterial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.4f, 0.4f, 0.4f, 1.0));
		colorMaterial->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0));
		colorMaterial->setShininess(osg::Material::FRONT_AND_BACK, 16.0f);

	geoState->setAttributeAndModes(colorMaterial, osg::StateAttribute::ON);
	
	
	
	
	
		geometry = mt;
	}
	if (geometry == NULL)
	{
		osg::Geode* geode;

		osg::Sphere* mySphere = new osg::Sphere(osg::Vec3(0, 0, 0), 20.0*modelScale);
		osg::ShapeDrawable* mySphereDrawable = new osg::ShapeDrawable(mySphere, MidiPlugin::instance()->hint.get());
		mySphereDrawable->setColor(color);
		geode = new osg::Geode();
		geode->addDrawable(mySphereDrawable);
		geode->setStateSet(MidiPlugin::instance()->shadedStateSet.get());
		geometry = geode;
	}
}

MidiInstrument::MidiInstrument(std::string name,int id)
{
	noteInfos.resize(128);
	for (int i = 0; i < 128; i++)
		noteInfos[i] = NULL;
	type = coCoviseConfig::getEntry("type", "COVER.Plugin.Midi." + name, "none");
	channel = coCoviseConfig::getInt("channel", "COVER.Plugin.Midi." + name, 0);

	coCoviseConfig::ScopeEntries InstrumentEntries = coCoviseConfig::getScopeEntries("COVER.Plugin.Midi."+name, "Key");
	const char** it = InstrumentEntries.getValue();
	while (it && *it)
	{
		string n = *it;

		std::string configName = "COVER.Plugin.Midi." + name + "." + n;

		int keyNumber = coCoviseConfig::getInt("name", configName, 0);
		NoteInfo *noteInfo = new NoteInfo(keyNumber);
		float r, g, b, a;
		r = coCoviseConfig::getFloat("r", configName, 1.0);
		g = coCoviseConfig::getFloat("g", configName, 1.0);
		b = coCoviseConfig::getFloat("b", configName, 1.0);
		a = coCoviseConfig::getFloat("a", configName,  1.0);
		if (r > 1.0)
			r = r / 255.0;
		if (g > 1.0)
			g = g / 255.0;
		if (b > 1.0)
			b = b / 255.0;
		if (a > 1.0)
			a = a / 255.0;
		noteInfo->color = osg::Vec4(r,g,b,a);
		noteInfo->initialPosition.set(coCoviseConfig::getFloat("x", configName,  0.0), coCoviseConfig::getFloat("y", configName, 0.0), coCoviseConfig::getFloat("z", configName,  0.0));
		noteInfo->modelScale = coCoviseConfig::getFloat("modelScale", configName,  1.0);
		noteInfo->modelName = coCoviseConfig::getEntry("modelName", configName,  "");
		noteInfos[keyNumber] = noteInfo;

		it++;// value
		it++;// next key
	}

	int i = 0;
	for (auto& noteInfo : noteInfos)
	{
		if (noteInfo)
		{
			noteInfo->createGeom();
			float angle = ((float)i / noteInfos.size()) * 2.0 * M_PI * 2;
			float radius = 800.0;
			if (noteInfo->initialPosition == osg::Vec3(0, 0, 0))
			{
				noteInfo->initialPosition.set(sin(angle) * radius, 0, cos(angle) * radius);
				noteInfo->initialVelocity.set(sin(angle) * 100.0, 1000, cos(angle) * 100.0);
				noteInfo->initialVelocity.set(sin(angle) * 100.0, 1000, cos(angle) * 100.0);
			}
			else
			{
				osg::Vec3 v = noteInfo->initialPosition;
				v.normalize();
				v[1] = 1000.0;
				noteInfo->initialVelocity = v;
			}
			i++;
		}
	}

}
MidiInstrument::~MidiInstrument()
{
}

MidiDevice::MidiDevice(std::string n, int id)
{
	name = n;
	ID = id;

	std::string configName = "COVER.Plugin.Midi." + name;
	int instID = coCoviseConfig::getInt("instrument", configName, 0);
	if( MidiPlugin::instance()->instruments.size()>instID)
	{
		instrument = MidiPlugin::instance()->instruments[instID].get();
	}
	else
	{
		instrument = NULL;
	}
}

MidiDevice::~MidiDevice()
{
}

bool MidiPlugin::init()
{
	currentTrack = 0;

	for (int i = 0; i < NUMMidiStreams; i++)
	{
#ifdef WIN32
		hMidiDevice[i] = NULL;
#else
#endif
		midifd[i] = -1;
		inputDevice[i] = NULL;

		lTrack[i] = NULL;
		lTrack[i] = new Track(i, true);
	}
	coCoviseConfig::ScopeEntries InstrumentEntries = coCoviseConfig::getScopeEntries("COVER.Plugin.Midi", "Instrument");
	const char** it = InstrumentEntries.getValue();
	while (it && *it)
	{
		string n = *it;
		std::unique_ptr<MidiInstrument> instrument = std::unique_ptr<MidiInstrument>(new MidiInstrument(n,instruments.size()));
		lTrack[instrument->channel]->instrument = instrument.get();

		instruments.push_back(std::move(instrument));

		it++;
		it++;
	}
	coCoviseConfig::ScopeEntries DeviceEntries = coCoviseConfig::getScopeEntries("COVER.Plugin.Midi", "Device");
	it = DeviceEntries.getValue();
	while (it && *it)
	{
		string n = *it;
		std::unique_ptr<MidiDevice> device = std::unique_ptr<MidiDevice>(new MidiDevice(n, devices.size()));
		devices.push_back(std::move(device));

		it++;
		it++;
	}


	int instrument = coCoviseConfig::getInt("InPort", "COVER.Plugin.Midi.Instrument", 0);
	int midiPortOut = coCoviseConfig::getInt("OutPort", "COVER.Plugin.Midi", 1);


	coVRFileManager::instance()->registerFileHandler(&handlers[0]);
	coVRFileManager::instance()->registerFileHandler(&handlers[1]);
	//----------------------------------------------------------------------------
  /*  if (player == NULL)
	{
		player = cover->usePlayer(playerUnavailableCB);
		if (player == NULL)
		{
			cover->unusePlayer(playerUnavailableCB);
			cover->addPlugin("Vrml97");
			player = cover->usePlayer(playerUnavailableCB);
			if (player == NULL)
			{
				cerr << "sorry, no VRML, no Sound support " << endl;
			}
		}
	}*/

	MIDIRoot = new osg::Group;
	MIDIRoot->setName("MIDIRoot");
	cover->getScene()->addChild(MIDIRoot.get());
	for (int i = 0; i < NUMMidiStreams; i++)
	{
		MIDITrans[i] = new osg::MatrixTransform();
		std::string name = "MIDITrans_";
		name += std::to_string(i);
		MIDITrans[i]->setName(name);
		MIDIRoot->addChild(MIDITrans[i].get());
		float angle = ((M_PI*2.0) / NUMMidiStreams) * i;
		//MIDITrans[i]->setMatrix(osg::Matrix::rotate(angle, osg::Vec3(0, 0, 1)));
	}


	hint = new osg::TessellationHints();
	hint->setDetailRatio(1.0);





	if (coVRMSController::instance()->isMaster())
	{
		int midiPort = coCoviseConfig::getInt("InPort", "COVER.Plugin.Midi", 0);
		int midiPortOut = coCoviseConfig::getInt("OutPort", "COVER.Plugin.Midi", 1);
                int n=0;
		for (int i = 0; i < NUMMidiStreams; i++)
		{
			while(openMidiIn(i, midiPort + n)==false)
			{
			        if(n>50)
			          break;
				fprintf(stderr, "OpenMidiIn %d failed\n", midiPort+ n);
			        n++;
			}
			fprintf(stderr, "OpenMidiIn Stream %d device %d succeeded\n", i,midiPort+ n);
			if(n>50)
			    break;
			 n++;
		}
		fprintf(stderr, "OpenMidiOut %d\n", midiPortOut);
		if (openMidiOut(midiPortOut))
		{
			fprintf(stderr, "OpenMidiOut %d failed\n", midiPortOut);
		}
	}
	MIDItab_create();
	return true;
}

bool MidiPlugin::openMidiIn(int streamNum, int device)
{
#ifndef WIN32
	char devName[100];
	if (device == 0)
		sprintf(devName, "/dev/midi");
	else
		sprintf(devName, "/dev/midi%d", device );
	midifd[streamNum] = open(devName, O_RDONLY | O_NONBLOCK);
	if (midifd[streamNum] <= 0)
	{
		sprintf(devName, "/dev/midi%d", device);
		midifd[streamNum] = open(devName, O_RDONLY | O_NONBLOCK);
	}
	fprintf(stderr, "open %s %d\n", devName, midifd[streamNum]);
	if (midifd[streamNum] <= 0)
		return false;
#else
	UINT nMidiDeviceNum;
	nMidiDeviceNum = midiInGetNumDevs();
	if (nMidiDeviceNum == 0) {
		fprintf(stderr, "midiInGetNumDevs() return 0...");
		return false;
	}
	else
	{
		MMRESULT rv;
		UINT nMidiPort = (uint)device;

		rv = midiInOpen(&hMidiDevice[streamNum], nMidiPort, (DWORD_PTR)MidiInProc, streamNum, CALLBACK_FUNCTION);
		if (rv != MMSYSERR_NOERROR) {
			fprintf(stderr, "midiInOpen() failed...rv=%d", rv);
			return false;
		}
		else
		{
			midiInStart(hMidiDevice[streamNum]);
		}

	}

#endif

	return true;
}

bool MidiPlugin::openMidiOut(int device)
{
#ifndef WIN32
	char devName[100];
	sprintf(devName, "-/dev/midi%d", device + 1);
	midiOutfd = open(devName, O_WRONLY | O_NONBLOCK);
	fprintf(stderr, "open /dev/midi%d %d", device + 1, midiOutfd);
	if (midiOutfd <= 0)
		return false;
#else
	UINT nMidiDeviceNum;
	nMidiDeviceNum = midiInGetNumDevs();
	if (nMidiDeviceNum == 0) {
		fprintf(stderr, "midiInGetNumDevs() return 0...");
		return false;
	}
	else
	{
		MMRESULT rv;
		UINT nMidiPort = (uint)device;

		rv = midiOutOpen(&hMidiDeviceOut, nMidiPort, 0, 0, CALLBACK_NULL);
		if (rv != MMSYSERR_NOERROR) {
			fprintf(stderr, "midiOutOpen() failed...rv=%d", rv);
			return false;
		}
		fprintf(stderr, "midiOutOpen() succeded...rv=%d nMidiPort %d", rv, nMidiPort);

	}

#endif
	return true;
}


//------------------------------------------------------------------------------
MidiPlugin::~MidiPlugin()
{
	for (int i = 0; i < NUMMidiStreams; i++)
	{
		delete lTrack[i];
#ifdef WIN32
		if (hMidiDevice && hMidiDevice[i])
			midiInClose(hMidiDevice[i]);
#else
		if(midifd[i]>=0)
		close(midifd[i]);
#endif
	}

	SDL_Quit();
}

void MidiPlugin::addEvent(MidiEvent &me, int MidiStream)
{
	eventqueue[MidiStream].push_back(me);
}
bool MidiPlugin::destroy()
{

	coVRFileManager::instance()->unregisterFileHandler(&handlers[0]);
	coVRFileManager::instance()->unregisterFileHandler(&handlers[1]);

	cover->getObjectsRoot()->removeChild(MIDIRoot.get());

	MIDItab_delete();

	return true;
}

bool MidiPlugin::update()
{
	for (auto it = audioStreams.begin(); it != audioStreams.end(); it++)
	{
		(*it)->update();
	}
	for (auto it = waveSurfaces.begin(); it != waveSurfaces.end(); it++)
	{
		(*it)->update();
	}
	struct UDPPacket
	{
	    unsigned char command;
	    unsigned char key;
	    unsigned char velocity;
	    unsigned char channel;
	};
	int status = 0;
	UDPPacket packet;





	MidiEvent me;

	if (coVRMSController::instance()->isMaster())
	{
	    do{
		if (udp)
		{
		    status = udp->receive(&packet, sizeof(packet),0.001);
		}
		else
		{
		    status = -1;
		}

		me.setP0(0);
		me.setP1(0);
		me.setP2(0);
		if (status == sizeof(packet))
		{
		    fprintf(stdout,"command %d, key %d, velocity %d, channel %d\n",(int)packet.command,(int)packet.key,(int)packet.velocity,(int)packet.channel);
		    if(packet.command == 1 && packet.channel == 9)
		    {
			me.makeNoteOn(packet.channel, packet.key,packet.velocity);

			TokenBuffer tb;
			tb << (int)packet.channel;
			tb << me.getP0();
			tb << me.getP1();
			tb << me.getP2();
			tb << me.getP3();
			UdpMessage um(tb, covise::MIDI_STREAM);
			cover->sendVrbMessage(&um);
			cerr << "sending MIDI comming from UDP" << endl;

			addEvent(me, packet.channel);
			signed char buf[4];
			buf[0] = me.getP0();
			buf[1] = me.getP1();
			buf[2] = me.getP2();
			coVRMSController::instance()->sendSlaves((char *)buf, 4);
		    }
		    //if(packet.command == 2)
		    //me.makeNoteOff(packet.channel, packet.key,packet.velocity);
		    //if(packet.command == 3)
		    //me.makeController(packet.channel, packet.key,packet.velocity);
		    //	keySym,mod);
		}
		else if (status == -1)
		{
		//std::cerr << "FlightGear::update: error while reading data" << std::endl;
		//initUDP();
		//return false;
		}
		else
		{
		//std::cerr << "FlightGear::update: received invalid no. of bytes: recv=" << status << ", got=" << status << std::endl;
		//initUDP();
		//return false;
		}
	    } while(status == sizeof(packet));

	    signed char buf[4];
	    buf[0] = me.getP0();
	    buf[1] = me.getP1();
	    buf[2] = me.getP2();
	    coVRMSController::instance()->sendSlaves((char *)buf, 4);
	}
	else
	{
	     do{

		  signed char buf[4];
		  coVRMSController::instance()->readMaster((char *)buf, 4);
		  me.setP0(buf[0]);
		  me.setP1(buf[1]);
		  me.setP2(buf[2]);
		  if(me.getP0() != 0 || me.getP1() != 0)
		  {
		       addEvent(me, packet.channel);
		  }

	     } while(me.getP0() != 0 || me.getP1() != 0);
	}
	
	
	
	return true;
}

//------------------------------------------------------------------------------
void MidiPlugin::preFrame()
{


		if (startTime == 0.0)
		{
	for (int i = 0; i < NUMMidiStreams; i++)
	{
			lTrack[i]->reset();
			startTime = cover->frameTime();
			lTrack[i]->setVisible(true);
			fprintf(stderr,"visible %d\n",i);
	}
		}
	for (int i = 0; i < NUMMidiStreams; i++)
	{
		while (eventqueue[i].size() > 0)
		{
			MidiEvent me = *eventqueue[i].begin();
			eventqueue[i].pop_front();
			if (me.getKeyNumber() == 31) // special reset key (drumpad)
			{
				if (me.getVelocity() < 50)
				{
					for (int i = 0; i < NUMMidiStreams; i++)
					{
						lTrack[i]->reset();
					}
				}
				else
				{
					for (int i = 0; i < NUMMidiStreams; i++)
					{
						MidiInstrument* inst = lTrack[i]->instrument;
					lTrack[i]->store();
					lTrack[i] = new Track(tracks.size(), true);
					lTrack[i]->reset();
					lTrack[i]->instrument = inst;
					lTrack[i]->setVisible(true);
					}
				}
			}
			else
			{
				int channel = me.getChannel();
				if (lTrack[channel]->instrument != nullptr)
				{
					lTrack[channel]->handleEvent(me);
				}
				else
				{
					if (me.getKeyNumber() > 0)
					fprintf(stderr, "unconfigured instrument channel: %d  key: %02d velo %03d \n", me.getChannel(), me.getKeyNumber(), me.getVelocity());

				}
			}
		}
		lTrack[i]->update();
	}
	//fprintf(stderr,"tracks %d\n",tracks.size());
	if (tracks.size() > 0)
	{
		if (startTime == 0.0)
		{
			tracks[currentTrack]->reset();
			startTime = cover->frameTime();
			tracks[currentTrack]->setVisible(true);
		}
		tracks[currentTrack]->update();
	}
}
//------------------------------------------------------------------------------
void MidiPlugin::postFrame()
{
	// we do not need to care about animation (auto or step) here,
	// because it's in the main program
}

//----------------------------------------------------------------------------
//
//--------------------------------------------------------------------
void MidiPlugin::setTimestep(int t)
{
}

void MidiPlugin::handleController(MidiEvent& me)
{
	fprintf(stderr, "Controller Nr.%d, value %d\n", me.getP1(), me.getP2());
	int controllerID = me.getP1();
	int value = me.getP2();
	if (controllerID == 5)
	{
		if (value < 10)
		{
			VRSceneGraph::instance()->setWireframe(VRSceneGraph::Points);
			fprintf(stderr, "Points\n");
		}
		else if (value < 40)
			{
				VRSceneGraph::instance()->setWireframe(VRSceneGraph::HiddenLineBlack);
			}
		else if (value < 80)
		{
			VRSceneGraph::instance()->setWireframe(VRSceneGraph::Disabled);
		}
		else if (value < 100)
		{
			VRSceneGraph::instance()->setWireframe(VRSceneGraph::HiddenLineWhite);
		}
		else
		{
			VRSceneGraph::instance()->setWireframe(VRSceneGraph::Enabled);
		}

	}
	if (controllerID == 65)
	{
		if (value > 64)
		{

			spiralSpeed += 0.05;
			if (spiralSpeed > 5)
				spiralSpeed = 5;
			spiralSpeedSlider->setValue(spiralSpeed);
			
		}
		else
		{
			spiralSpeed -= 0.05;
			if (spiralSpeed < -5)
				spiralSpeed = -5;
			spiralSpeedSlider->setValue(spiralSpeed);
		}
	}
	if (controllerID == 55)
	{
		frequencySurface->radius1 = value;
		amplitudeSurface->radius1 = value;
	}
	if (controllerID == 60)
	{
		frequencySurface->radius2 = value;
		amplitudeSurface->radius2 = value;
	}
	if ((controllerID == 61) || (controllerID == 34))
	{
		sphereScale = 0.1+((value/127.0)*10.0);
		sphereScaleSlider->setValue(sphereScale);
	}
	if (controllerID == 56)
	{
		frequencySurface->yStep = value;
		amplitudeSurface->yStep = value;
	}
	if (controllerID == 57)
	{
		frequencySurface->amplitudeFactor = (value-63)/12.0;
		//amplitudeSurface->amplitudeFactor = (value - 63) / 12.0;
	}
	if (controllerID == 62)
	{
		//frequencySurface->frequencyFactor = (value - 63) / 12.0;
		amplitudeSurface->frequencyFactor = (value - 63) / 12.0;
	}
	if ((controllerID == 52) || (controllerID == 33)) // slider left
	{
		float sliderValue = ((float)(value - 64) / 64.0)*5.0;
		osg::Vec3 rotSpeed;
		rotSpeed.set(sliderValue, sliderValue / 2.0, 0);

		fprintf(stderr, "rot%f, %f, %f\n", rotSpeed[0], rotSpeed[1], rotSpeed[2]);
		for (int i = 0; i < NUMMidiStreams; i++)
		{
			lTrack[i]->setRotation(rotSpeed);
		}
	}
	if ((controllerID == 53)|| (controllerID == 32)) // slider right
	{
		float sliderValue = ((float)(value - 64) / 64.0)*0.3;
		rAcceleration = sliderValue;
		raccelSlider->setValue(sliderValue);
	}
	if ((controllerID == 63) || (controllerID == 15)) // distance sensor
	{
		float sliderValue = value / 127.0;
		speedFactor = (100.0 * sliderValue) + 1.0;
	}
	if (controllerID == 1) // PitchBendUp
	{
		float sliderValue = ((float)(value ) / 127.0) * 5.0;
		osg::Vec3 rotSpeed;
		rotSpeed.set(sliderValue, sliderValue / 2.0, 0);

		fprintf(stderr, "rot%f, %f, %f\n", rotSpeed[0], rotSpeed[1], rotSpeed[2]);
		for (int i = 0; i < NUMMidiStreams; i++)
		{
			lTrack[i]->setRotation(rotSpeed);
		}
	}
	if (controllerID == 2) // PitchBendDown
	{
		float sliderValue = ((float)(value) / 127.0) * -5.0;
		osg::Vec3 rotSpeed;
		rotSpeed.set(sliderValue, sliderValue / 2.0, 0);

		fprintf(stderr, "rot%f, %f, %f\n", rotSpeed[0], rotSpeed[1], rotSpeed[2]);
		for (int i = 0; i < NUMMidiStreams; i++)
		{
			lTrack[i]->setRotation(rotSpeed);
		}
	}


}

//--------------------------------------------------------------------
void MidiPlugin::MIDItab_create(void)
{

	MIDITab = new ui::Menu("Midi", this);
	reset = new ui::Button(MIDITab, "Reset");
	reset->setText("Reset");
	reset->setCallback([this](bool) {
		for (int i = 0; i < NUMMidiStreams; i++)
		{
			lTrack[i]->reset();
		}
	});

	clearStoreButton = new ui::Button(MIDITab, "ClearStore");
	clearStoreButton->setText("Clear Store");
	clearStoreButton->setCallback([this](bool) {
		clearStore();
		});

	radius1Slider = new ui::Slider(MIDITab, "Radius1");
	radius1Slider->setText("Radius1");
	radius1Slider->setBounds(1, 100);
	radius1Slider->setValue(40.0);
	radius1Slider->setCallback([this](float value, bool) {
		frequencySurface->radius1 = value;
		amplitudeSurface->radius1 = value;
	});
	radius2Slider = new ui::Slider(MIDITab, "Radius2");
	radius2Slider->setText("Radius2");
	radius2Slider->setBounds(1, 100);
	radius2Slider->setValue(20.0);
	radius2Slider->setCallback([this](float value, bool) {
		frequencySurface->radius2 = value;
		amplitudeSurface->radius2 = value;
	});
	yStepSlider = new ui::Slider(MIDITab, "yStep");
	yStepSlider->setText("yStep");
	yStepSlider->setValue(0.6);
	yStepSlider->setBounds(0.02, 10);
	yStepSlider->setCallback([this](float value, bool) {
		frequencySurface->yStep = value;
		amplitudeSurface->yStep = value;
	});
	amplitudeFactor = new ui::Slider(MIDITab, "Amplitude");
	amplitudeFactor->setText("Amplitude");
	amplitudeFactor->setBounds(-50, 50);
	amplitudeFactor->setValue(-10);
	amplitudeFactor->setCallback([this](float value, bool) {
		frequencySurface->amplitudeFactor = value;
		amplitudeSurface->amplitudeFactor = value;
	});
	frequencyFactor = new ui::Slider(MIDITab, "Frequency");
	frequencyFactor->setText("Frequency");
	frequencyFactor->setBounds(-50, 50);
	frequencyFactor->setValue(-1);
	frequencyFactor->setCallback([this](float value, bool) {
		frequencySurface->frequencyFactor = value;
		amplitudeSurface->frequencyFactor = value;
	});

	accelSlider = new ui::Slider(MIDITab, "Acceleration");
	accelSlider->setText("Acceleration");
	accelSlider->setBounds(-1000, 100);
	accelSlider->setValue(acceleration);
	accelSlider->setCallback([this](float value, bool) {
		acceleration = value;
	});
	raccelSlider = new ui::Slider(MIDITab, "rAcceleration");
	raccelSlider->setText("rAcceleration");
	raccelSlider->setBounds(-1, 1);
	raccelSlider->setValue(rAcceleration);
	raccelSlider->setCallback([this](float value, bool) {
		rAcceleration = value;
	});
	spiralSpeedSlider = new ui::Slider(MIDITab, "spiralSpeed");
	spiralSpeedSlider->setText("spiralSpeed");
	spiralSpeedSlider->setBounds(-5, 5);
	spiralSpeedSlider->setValue(0.1);
	spiralSpeedSlider->setCallback([this](float value, bool) {
		spiralSpeed = value;
	});

	sphereScaleSlider = new ui::Slider(MIDITab, "SphereScale");
	sphereScaleSlider->setText("sphereScale");
	sphereScaleSlider->setBounds(0.1, 10);
	sphereScaleSlider->setValue(1.0);
	sphereScaleSlider->setCallback([this](float value, bool) {
		sphereScale = value;
		});
	
	trackNumber = new  ui::EditField(MIDITab, "trackNumber");
	trackNumber->setValue(0);
	trackNumber->setCallback([this](std::string newVal) {
		if (newVal.length() > 0)
		{
			currentTrack = std::stoi(newVal);
			if (currentTrack >= 0 && currentTrack < tracks.size())
			{
			}
			else
			{
				currentTrack = 0;
			}
			if (currentTrack >= 0 && currentTrack < tracks.size())
			{
				tracks[currentTrack]->setVisible(false);

				if (currentTrack >= 0)
				{
					tracks[currentTrack]->reset();
					startTime = cover->frameTime();
					tracks[currentTrack]->setVisible(true);
				}
			}
		}
	});
	outputDevice = new ui::SelectionList(MIDITab, "outputDevices");
	for (int i = 0; i < NUMMidiStreams; i++)
	{
		char name[500];
		sprintf(name, "inputDevices_%d", i);
		inputDevice[i] = new ui::SelectionList(MIDITab, name);

#ifdef WIN32
		const UINT_PTR nMidiIn = midiInGetNumDevs();
		for (UINT_PTR iMidiIn = 0; iMidiIn < nMidiIn; ++iMidiIn) {
			MIDIINCAPS mic{};
			midiInGetDevCaps(iMidiIn, &mic, sizeof(mic));
			printf("MIDI Inp#%2d [%s]\n", (int)iMidiIn, mic.szPname);
			inputDevice[i]->append(mic.szPname);
		}
#else

		inputDevice[i]->append(std::to_string(i));

#endif
		inputDevice[i]->setCallback([this, i](int newInDev) {
#ifdef WIN32
			if (hMidiDevice)
				midiInClose(hMidiDevice[i]);
#else
			close(midifd[i]);
#endif
			openMidiIn(i, newInDev);
			fprintf(stderr, "openMidiIn %d failed\n", newInDev);
		});
	}

#ifdef WIN32
	const UINT_PTR nMidiOut = midiOutGetNumDevs();
	for (UINT_PTR iMidiOut = 0; iMidiOut < nMidiOut; ++iMidiOut) {
		MIDIOUTCAPS moc{};
		midiOutGetDevCaps(iMidiOut, &moc, sizeof(moc));
		printf("MIDI Outp#%2d [%s]\n", (int)iMidiOut, moc.szPname);
		outputDevice->append(moc.szPname);
	}
#else
	outputDevice->append("0");
#endif

	outputDevice->setCallback([this](int newOutDev) {
#ifdef WIN32
		if (hMidiDeviceOut)
			midiOutClose(hMidiDeviceOut);
#else
		close(midiOutfd);
#endif
		openMidiOut(newOutDev);

	});

	outputDevice = new ui::SelectionList(MIDITab, "outputDevicest");


	infoLabel = new ui::Label(MIDITab, "MIDI Version 1.0");


}

void MidiPlugin::setTempo(int index) {
	double newtempo = 0.0;
	static int count = 0;
	count++;

	MidiEvent& mididata = midifile[0][index];

	int microseconds = 0;
	microseconds = microseconds | (mididata[3] << 16);
	microseconds = microseconds | (mididata[4] << 8);
	microseconds = microseconds | (mididata[5] << 0);

	newtempo = 60.0 / microseconds * 1000000.0;
	if (count <= 1) {
		tempo = newtempo;
	}
	else if (tempo != newtempo) {
		cout << "; WARNING: change of tempo from " << tempo
			<< " to " << newtempo << " ignored" << endl;
	}
}



//--------------------------------------------------------------------
void MidiPlugin::MIDItab_delete(void)
{
	if (MIDITab)
	{
		delete infoLabel;

		delete MIDITab;
	}
}

//--------------------------------------------------------------------

COVERPLUGIN(MidiPlugin)


Track::Track(int tn, bool l)
{
	life = l;
	rotationSpeed.set(0, 0, 0);
	TrackRoot = new osg::MatrixTransform();
	TrackRoot->setName("TrackRoot"+std::to_string(tn));
	trackNumber = tn;
	streamNum = trackNumber % MidiPlugin::instance()->NUMMidiStreams;

	if(streamNum < MidiPlugin::instance()->devices.size())
	{
	  instrument = MidiPlugin::instance()->devices[streamNum]->instrument;
	}
	else
	{
		if(MidiPlugin::instance()->devices.size()>0)
		{
		    instrument = MidiPlugin::instance()->devices[0]->instrument;
		}
	}

	char soundName[200];
	snprintf(soundName, 200, "RENDERS/S%d.wav", tn);
	//trackAudio = new vrml::Audio(soundName);
	trackAudio = NULL;
	trackSource = NULL;
	if (trackAudio != NULL && MidiPlugin::instance()->player)
	{
		trackSource = MidiPlugin::instance()->player->newSource(trackAudio);
		if (trackSource)
		{
			trackSource->setLoop(false);
			trackSource->setPitch(1);
			trackSource->stop();
			trackSource->setIntensity(1.0);
			trackSource->setSpatialize(false);
			trackSource->setVelocity(0, 0, 0);

		}
	}

	geometryLines = createLinesGeometry();
	TrackRoot->addChild(geometryLines);
	lastNum = 0;
	lastPrimitive = 0;
	eventNumber = 0;
}
Track::~Track()
{
}
void Track::addNote(Note *n)
{
	fprintf(stderr,"add: %02d velo %03d chan %d\n", n->event.getKeyNumber(), n->event.getVelocity(), n->event.getChannel());
	n->spin = rotationSpeed;
	if (n->track->instrument->type == "keyboard")
	{
		notes.push_back(n);
		n->setInactive(false);
		n->vertNum = lineVert->size();
		lineVert->push_back(n->transform->getMatrix().getTrans());
		lineColor->push_back(osg::Vec4(1, 0, 0, 1));

		Note* currentNote = new Note(n->event, this);
		notes.push_back(currentNote);
		currentNote->spin = -rotationSpeed;
		currentNote->vertNum = lineVert->size();
		lineVert->push_back(currentNote->transform->getMatrix().getTrans());
		lineColor->push_back(osg::Vec4(1, 0, 0, 1));
		//int numLines = linePrimitives->size();
		linePrimitives->push_back(2);
	}
	else
	{
		Note* lastNode = NULL;
		int num = notes.size() - 1;
		if (num >= 0)
		{
			std::list<Note*>::iterator it;
			it = notes.end();
			it--;
			lastNode = *it;
		}
		notes.push_back(n);
		n->setInactive(false);
		if (lastNode == NULL || ((n->event.seconds - lastNode->event.seconds) > 1.0))
		{
			if (lastNode != NULL && num - lastNum == 0)
			{
				fprintf(stderr, "new zero line\n");
				linePrimitives->push_back(1);
			}
			lastNum = num;
			fprintf(stderr, "%d\n", (num - lastNum) + 1);
			fprintf(stderr, "td\n");
		}
		//fprintf(stderr, "num %d lastNum %d\n", num, lastNum);
		if (num - lastNum == 1)
		{
			fprintf(stderr, "new line\n");
			lastPrimitive = linePrimitives->size();
			linePrimitives->push_back((num - lastNum) + 1);
		}
		if (num - lastNum > 0)
		{
			//fprintf(stderr, "addLineVert%d\n", (num - lastNum) + 1);
			(*linePrimitives)[lastPrimitive] = (num - lastNum) + 1;
		}
		lineVert->push_back(n->transform->getMatrix().getTrans());
		lineColor->push_back(osg::Vec4(1, 0, 0, 1));
	}
}

void Track::endNote(MidiEvent& me)
{
	Note *note = NULL;
	fprintf(stderr,"end: %02d velo %03d chan %d\n", me.getKeyNumber(), me.getVelocity(), me.getChannel());

	// find key press for this release
	for (auto it = notes.end(); it != notes.begin(); )
	{
		it--;
		if ((*it)->event.getKeyNumber() == me.getKeyNumber())
		{
			note = *it;
			//printf("foundNoteOn: %02d velo %03d chan %d device %d\n", me.getKeyNumber(), me.getVelocity(), me.getChannel());
			if (note->track->instrument->type == "keyboard")
			{
				Note* lastNode = *it;
				if (it != notes.begin())
					it--;
				note = *it;
				me.setVelocity(note->event.getVelocity());
				lastNode->setInactive(false);
			}
			break;
		}
	}
	if (note != NULL)
	{

	}
	
}

void Track::setRotation(osg::Vec3& rotSpeed)
{
	rotationSpeed = rotSpeed;
}

osg::Geode *Track::createLinesGeometry()
{
	osg::Geode *geode;


	geode = new osg::Geode();
	geode->setStateSet(MidiPlugin::instance()->lineStateSet.get());


	osg::Geometry *geom = new osg::Geometry();
	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(false);

	// set up geometry
	lineVert = new osg::Vec3Array;
	lineColor = new osg::Vec4Array;

	linePrimitives = new osg::DrawArrayLengths(osg::PrimitiveSet::LINE_STRIP);
	linePrimitives->push_back(0);
	geom->setVertexArray(lineVert);
	geom->setColorArray(lineColor);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(linePrimitives);

	geode->addDrawable(geom);

	return geode;
}
void Track::handleEvent(MidiEvent& me)
{
	if (me.isNoteOn())
	{
		addNote(new Note(me, this));
	}
	else if (me.isNoteOff())
	{
		endNote(me);
	}
	else if (me.isController())
	{
		MidiPlugin::instance()->handleController(me);
	}
}
void Track::store()
{
	int pos = MidiPlugin::instance()->storedTracks.size();
	MidiPlugin::instance()->storedTracks.push_back(this);
	TrackRoot->getParent(0)->removeChild(TrackRoot);
	cover->getObjectsRoot()->addChild(TrackRoot);
	osg::Matrix mat;
	int xp = pos % 6;
	int yp = pos / 6;
	mat = osg::Matrix::scale(0.00003, 0.00003, 0.00003)*osg::Matrix::translate(0.5 * xp, 0.5, (0.5*yp) + 0.5);
	TrackRoot->setMatrix(mat);
}
void MidiPlugin::clearStore()
{
	for (auto it = storedTracks.begin(); it != storedTracks.end(); it++)
	{
		(*it)->clearStore();
	}
	storedTracks.clear();
}
void Track::clearStore()
{
	TrackRoot->getParent(0)->removeChild(TrackRoot);
}
void Track::reset()
{
	eventNumber = 0;
	if (trackSource != NULL)
	{
		trackSource->play();
	}
	for (std::list<Note *>::iterator it = notes.begin(); it != notes.end(); it++)
	{
		delete *it;
	}
	notes.clear();
	lineVert->resize(0);
	lineColor->resize(0);
	linePrimitives->resize(0);

	lineVert->dirty();
	lineColor->dirty();
	linePrimitives->dirty();

	lastNum = 0;
	lastPrimitive = 0;
}


void Track::update()
{
	double speed = MidiPlugin::instance()->midifile.getTicksPerQuarterNote();
	double time = cover->frameTime() - MidiPlugin::instance()->startTime;
	MidiEvent me;
	if (life)
	{
		char buf[1000];
		int numRead = 1;
		while(numRead > 0)
		{	
		
			me.setP0(0);
			me.setP1(0);
			me.setP2(0);
			if (coVRMSController::instance()->isMaster())
			{
				if (MidiPlugin::instance()->midifd[streamNum] > 0)
				{
					numRead = read(MidiPlugin::instance()->midifd[streamNum], buf, 1);
					if (numRead > 0)
						fprintf(stderr, "numRead %d streamnum %d buf[0] %d\n", numRead, streamNum, buf[0]);

				}
				else
				{
					numRead = -1;
				}
				if (numRead > 0)
				{
					if (buf[0] == 127 || buf[0] == -112 || buf[0] == -119 || buf[0] == -103 || buf[0] == -80)
					{
						numRead = read(MidiPlugin::instance()->midifd[streamNum], buf + 1, 2);
						fprintf(stderr, "numRead %d buf[0] %d\n", numRead, buf[0]);
						fprintf(stderr, "numRead %d buf[1] %d\n", numRead, buf[1]);
						if (numRead < 2)
						{
							fprintf(stderr, "oopps %d %d\n", (int)buf[0], numRead);
						}

						me.setP0(buf[0]);
						me.setP1(buf[1]);
						me.setP2(buf[2]);
					TokenBuffer tb;
					tb << me.getChannel();
					tb << me.getP0();
					tb << me.getP1();
					tb << me.getP2();
					tb << me.getP3();
					UdpMessage um(tb, covise::MIDI_STREAM);
					cover->sendVrbMessage(&um);
					cerr << "sent:" << me.isNoteOn() << " " << me.getKeyNumber() << endl;
					fprintf(stderr, "sent: %01d %02d velo %03d chan %d numRead %d streamnum %d\n", me.isNoteOn(), me.getKeyNumber(), me.getVelocity(), me.getChannel(), numRead, streamNum);


					}
					else
					{
						if (buf[0] != -2 && buf[0] != -8)
						{
							fprintf(stderr, "unknown message %d %d\n", (int)buf[0], numRead);
						}
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
					}
				}



				signed char buf[4];
				buf[0] = me.getP0();
				buf[1] = me.getP1();
				buf[2] = me.getP2();
				buf[3] = numRead;
				coVRMSController::instance()->sendSlaves((char*)buf, 4);


				if (numRead > 0)
				{
					/*TokenBuffer tb;
					tb << me.getChannel();
					tb << me.getP0();
					tb << me.getP1();
					tb << me.getP2();
					tb << me.getP3();
					UdpMessage um(tb, vrb::MIDI_STREAM);
					cover->sendVrbMessage(&um);
					cerr << "sent:" << me.isNoteOn() << " " << me.getKeyNumber() << endl;
					fprintf(stderr, "sent: %01d %02d velo %03d chan %d numRead %d streamnum %d\n", me.isNoteOn(), me.getKeyNumber(), me.getVelocity(), me.getChannel(), numRead, streamNum);*/
				}
			}
			else
			{
				signed char buf[4];
				coVRMSController::instance()->readMaster((char *)buf, 4);
				me.setP0(buf[0]);
				me.setP1(buf[1]);
				me.setP2(buf[2]);
				numRead = buf[3];
	//fprintf(stderr,"received: %01d %02d velo %03d chan %d numRead %d\n", me.isNoteOn(),me.getKeyNumber(), me.getVelocity(), me.getChannel(),numRead);
			}
			if(numRead > 0 &&  me.getP0()!=0)
			{

				int channel = me.getChannel();
				if (MidiPlugin::instance()->lTrack[channel]->instrument != nullptr)
				{
					MidiPlugin::instance()->lTrack[channel]->handleEvent(me);
				}
				else
				{
					if(me.getKeyNumber()>0)
					fprintf(stderr, "unconfigured instrument channel: %d  key: %02d velo %03d \n", me.getChannel(), me.getKeyNumber(), me.getVelocity());

				}
			}
		}
	}
	else
	{
		if (eventNumber < MidiPlugin::instance()->midifile[trackNumber].size())
		{
			me = MidiPlugin::instance()->midifile[trackNumber][eventNumber];
		}
		while ((eventNumber < MidiPlugin::instance()->midifile[trackNumber].size()) && ((me.tick*60.0 / MidiPlugin::instance()->tempo / speed) < time))
		{
			me = MidiPlugin::instance()->midifile[trackNumber][eventNumber];
			me.getDurationInSeconds();
			handleEvent(me);
			eventNumber++;
		}
	}
	int vNum = 0;
	int numNotes = notes.size();
	for (std::list<Note *>::iterator it = notes.begin(); it != notes.end(); it++)
	{

		(*it)->integrate(cover->frameTime() - oldTime);
		if (vNum < lineVert->size())
		{
			osg::Vec3 v = ((*it)->transform->getMatrix().getTrans());
			(*lineVert)[vNum] = v;
			float len = v.length();
			v.normalize();
			osg::Vec4 c;
			c[0] = v[0];
			c[1] = v[1];
			c[2] = v[2];
			c[3] = 1.0;
			(*lineColor)[vNum] = c;
			vNum++;
		}
	}
	lineVert->dirty();
	lineColor->dirty();
	linePrimitives->dirty();
	oldTime = cover->frameTime();
}
void Track::setVisible(bool state)
{
	if (state)
	{
		MidiPlugin::instance()->MIDITrans[trackNumber%MidiPlugin::instance()->NUMMidiStreams]->addChild(TrackRoot);
		if (trackSource != NULL)
		{
			trackSource->play();
		}
	}
	else
	{
		MidiPlugin::instance()->MIDIRoot->removeChild(TrackRoot);
		if (trackSource != NULL)
		{
			trackSource->stop();
		}
	}
}

Note::Note(MidiEvent &me, Track *t)
{
	event = me;
	track = t;
	spin.set(0, 0, 0);
	rot.set(0, 0, 0);
	NoteInfo *ni = track->instrument->noteInfos[me.getKeyNumber()];
	transform = new osg::MatrixTransform();
	noteScale = MidiPlugin::instance()->sphereScale * event.getVelocity() / 10.0;
	if (ni == NULL)
	{
		fprintf(stderr, "no NoteInfo for Key %d\n", me.getKeyNumber());
		for (int i = 0; i < 128; i++)
		{
			if ((ni = track->instrument->noteInfos[i]) != NULL)
			{
				break;
			}
		}
		
		//event.setKeyNumber(0);
	}
	transform->setMatrix(osg::Matrix::scale(noteScale, noteScale, noteScale) * osg::Matrix::translate(ni->initialPosition));
	if (ni->geometry != NULL)
	{
		transform->addChild(ni->geometry);
	}
	velo = ni->initialVelocity*event.getVelocity() / 100.0;
	velo[1] = (event.getVelocity() - 32) * 40;
	t->TrackRoot->addChild(transform.get());

}
Note::~Note()
{
	track->TrackRoot->removeChild(transform.get());
}
void Note::integrate(double time)
{
	if (inactive)
		return;
	osg::Matrix nm = transform->getMatrix();
	osg::Vec3 pos = nm.getTrans();
	osg::Vec3 spiral;
	osg::Vec3 toCenter;
	spiral[0] = pos[2];
	spiral[1] = 0.0;
	spiral[2] = -pos[0];
	spiral *= MidiPlugin::instance()->spiralSpeed;
	toCenter[0] = -pos[0];
	toCenter[1] = 0.0;
	toCenter[2] = -pos[2];
	toCenter *= MidiPlugin::instance()->rAcceleration;
	osg::Vec3 a = osg::Vec3(0, -300.0, 0);
	//a = pos;
	a.normalize();
	a *= MidiPlugin::instance()->acceleration;
	velo = velo + a * time;
	pos += (velo + spiral+toCenter) * time;
	rot += spin*time* MidiPlugin::instance()->speedFactor;
	osg::Quat currentRot = osg::Quat(rot[0], osg::X_AXIS, rot[1] * time, osg::Y_AXIS, rot[2] * time, osg::Z_AXIS);
	nm.setTrans(pos);
	nm.setRotate(currentRot);
	nm= osg::Matrix::scale(noteScale, noteScale, noteScale)*nm;
	transform->setMatrix(nm);
}
void Note::setInactive(bool state)
{
	inactive = state;
}

FrequencySurface::FrequencySurface(osg::Group * parent, AudioInStream *s) :WaveSurface(parent, s, s->outputSize / 4)
{
	lastMagnitudes = new double[width];
	memset(lastMagnitudes, 0, (width) * sizeof(double));
}
FrequencySurface::~FrequencySurface()
{
	delete[] lastMagnitudes;
}

bool FrequencySurface::update()
{

	coVRShader *wsShader = coVRShaderList::instance()->get("WaveSurface");
	if (wsShader)
	{
		wsShader->apply(geode);
	}

	createNormals();
	moveOneLine();
	for (int n = 0; n < width; n++)
	{
		//(*vert)[n].z() = stream->ddata[n * 2] + stream->ddata[(n * 2) + 1];
		double val = (stream->magnitudes[n] + lastMagnitudes[n]) / 2.0;

		if (val > 20.0)
			val = 20.0;
		(*texCoord)[n].x() = val;
		if (st == SurfacePlane)
		{
			(*vert)[n].z() = (val) * 5;
		}
		else if (st == SurfaceCylinder)
		{
			float angle = (float)n / (float)(width - 1) * M_PI;
			float sa = sin(angle);
			float ca = cos(angle);
			(*vert)[n] = osg::Vec3(ca*(radius1 + (val*frequencyFactor)), 0.0, sa*(radius2 + (val*frequencyFactor)));
			(*normals)[n] = osg::Vec3(ca, 0, sa);
		}
		else if (st == SurfaceSphere)
		{
			float angle = (float)n / (float)(width - 1) * M_PI;
			float sa = sin(angle);
			float ca = cos(angle);
			(*vert)[n] = osg::Vec3(ca*(radius1 + val), 0.0, sa*(radius2 + val));
			(*normals)[n] = osg::Vec3(ca, 0, sa);
		}
		lastMagnitudes[n] = val;

	}

	vert->dirty();
	texCoord->dirty();
	normals->dirty();
	return true;
}

AmplitudeSurface::AmplitudeSurface(osg::Group * parent, AudioInStream *s) :WaveSurface(parent, s, s->inputSize / 2)
{
	lastAmplitudes = new double[width];
	memset(lastAmplitudes, 0, width * sizeof(double));
}
AmplitudeSurface::~AmplitudeSurface()
{
	delete[] lastAmplitudes;
}

bool AmplitudeSurface::update()
{
	coVRShader *wsShader = coVRShaderList::instance()->get("WaveSurfaceA");
	if (wsShader)
	{
		wsShader->apply(geode);
	}

	createNormals();
	moveOneLine();
	for (int n = 0; n < width; n++)
	{
		double val = (stream->ddata[n * 2] + stream->ddata[(n * 2) + 1] + lastAmplitudes[n]) / 2.0;
		if (val > 2.0)
			val = 2.0;
		(*texCoord)[n].x() = val;
		if (st == SurfacePlane)
		{
			(*vert)[n].z() = (val) * 10;
		}
		else if (st == SurfaceCylinder)
		{
			float angle = (float)n / (float)(width - 1) * M_PI;
			float sa = sin(angle);
			float ca = cos(angle);
			(*vert)[n] = osg::Vec3(ca*(radius1 + (val*amplitudeFactor)), 0.0, sa*(radius2 + (val*amplitudeFactor)));
			(*normals)[n] = osg::Vec3(ca, 0, sa);
		}
		else if (st == SurfaceSphere)
		{
			float angle = (float)n / (float)(width - 1) * M_PI;
			float sa = sin(angle);
			float ca = cos(angle);
			(*vert)[n] = osg::Vec3(ca*(radius1 + val), 0.0, sa*(radius2 + val));
			(*normals)[n] = osg::Vec3(ca, 0, sa);
		}
		lastAmplitudes[n] = val;

	}

	vert->dirty();
	texCoord->dirty();
	normals->dirty();
	return true;
}
WaveSurface::WaveSurface(osg::Group * parent, AudioInStream *s, int w)
{
	stream = s;
	width = w;
	radius1 = 40.0;
	radius2 = 20.0;

	yStep = 0.6;
	geode = new osg::Geode();
	geode->setName("WaveSurface");
	parent->addChild(geode.get());
	osg::Geometry *geom = new osg::Geometry();
	geode->addDrawable(geom);

	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(true);

	vert = new osg::Vec3Array;
	normals = new osg::Vec3Array;
	texCoord = new osg::Vec2Array;
	vert->reserve(depth*width);
	osg::DrawElementsUInt *primitives = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS);
	primitives->reserve((depth - 1)*(width - 1) * 4);
	for (int i = 1; i < depth; i++)
	{
		for (int n = 1; n < width; n++)
		{
			primitives->push_back(i*width + n);
			primitives->push_back(i*width + n - 1);
			primitives->push_back((i - 1)*width + n - 1);
			primitives->push_back((i - 1)*width + n);
		}
	}
	for (int i = 0; i < depth; i++)
	{
		for (int n = 0; n < width; n++)
		{

			texCoord->push_back(osg::Vec2(0, 0));
			vert->push_back(osg::Vec3(n*(20.0 / width), i*yStep, 0.0));
			normals->push_back(osg::Vec3(0, 0, 1));
		}
	}
	geom->addPrimitiveSet(primitives);
	geom->setVertexArray(vert);
	geom->setNormalArray(normals);
	geom->setTexCoordArray(0, texCoord);
	geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	osg::StateSet *geoState = geode->getOrCreateStateSet();

	if (globalDefaultMaterial.get() == NULL)
	{
		globalDefaultMaterial = new osg::Material;
		globalDefaultMaterial->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
		globalDefaultMaterial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2f, 0.2f, 0.2f, 1.0));
		globalDefaultMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0));
		globalDefaultMaterial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.4f, 0.4f, 0.4f, 1.0));
		globalDefaultMaterial->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0));
		globalDefaultMaterial->setShininess(osg::Material::FRONT_AND_BACK, 16.0f);
	}

	geoState->setAttributeAndModes(globalDefaultMaterial.get(), osg::StateAttribute::ON);
	
	osg::BoundingBox *boundingBox = new osg::BoundingBox(-radius1*2, -radius1*20, -radius1*2,radius1*2, radius1*20, radius1*2);
        geom->setInitialBound(*boundingBox);
}


void WaveSurface::setType(surfaceType s)
{
	st = s;
	if (st == SurfacePlane)
	{
		for (int i = 0; i < depth; i++)
		{
			for (int n = 0; n < width; n++)
			{
				(*vert)[i*width + n] = osg::Vec3(n*(20.0 / width), i*yStep, 0.0);
				(*normals)[i*width + n] = osg::Vec3(0, 0, 1);
			}
		}
	}
	else if (st == SurfaceCylinder)
	{
		for (int i = 0; i < depth; i++)
		{
			for (int n = 0; n < width; n++)
			{
				float angle = (float)n / (float)(width - 1) * M_PI;
				float sa = sin(angle);
				float ca = cos(angle);
				(*vert)[i*width + n] = osg::Vec3(ca*radius1, i*yStep, sa*radius2);
				(*normals)[i*width + n] = osg::Vec3(ca, 0, sa);
			}
		}
	}
	else if (st == SurfaceSphere)
	{
		for (int i = 0; i < depth; i++)
		{
			float angle2 = (float)i / (float)(depth - 1) * M_PI;
			float sa2 = sin(angle2);
			float ca2 = cos(angle2);
			for (int n = 0; n < width; n++)
			{
				float angle = (float)n / (float)(width - 1) * M_PI;
				float sa = sin(angle);
				float ca = cos(angle);
				(*vert)[i*width + n] = osg::Vec3(ca*(radius1), 0.0, sa*(radius1));
				(*normals)[i*width + n] = osg::Vec3(ca, 0, sa);
			}
		}
	}
}
osg::ref_ptr <osg::Material >WaveSurface::globalDefaultMaterial = NULL;

WaveSurface::~WaveSurface()
{
	while (geode->getParent(0))
		geode->getParent(0)->removeChild(geode);
}

void WaveSurface::moveOneLine()
{
	osg::Matrix trans = osg::Matrix::translate(0, yStep, 0);
	for (int i = (depth - 1); i > 0; i--)
	{
		for (int n = 0; n < width; n++)
		{
			(*texCoord)[i*width + n] = (*texCoord)[(i - 1)*width + n];
			(*vert)[i*width + n] = ((*vert)[(i - 1)*width + n])*trans;
			(*normals)[i*width + n] = (*normals)[(i - 1)*width + n];
		}
	}
}
void WaveSurface::createNormals()
{
	for (int n = 1; n < width; n++)
	{
		float nx = (*vert)[n].z() - (*vert)[n - 1].z() / ((*vert)[n - 1].x() - (*vert)[n].x());
		float ny = (*vert)[width + n].z() - (*vert)[n].z() / ((*vert)[width + n].y() - (*vert)[n].y());
		float dz = (*vert)[width + n].z() - (*vert)[n].z();
		float nz = 1.0;
		if (dz != 0)
		{
			nz = ((*vert)[n].y() - (*vert)[width + n].y()) / dz;
		}
		(*normals)[n].z() = 1;
		(*normals)[n].x() = nx;
		(*normals)[n].y() = ny;
		(*normals)[n].normalize();
	}

}
bool WaveSurface::update()
{
	return false;
}
