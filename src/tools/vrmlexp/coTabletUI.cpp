/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
#include <coTabletUIMessages.h>
#include "coTabletUI.h"
#include "covise_msg.h"
#include "covise_host.h"
#include "covise_connect.h"
#include <future>

#include <iostream>


using namespace std;
//#define FILEBROWSER_DEBUG

coTUIButton::coTUIButton(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_BUTTON)
{
}

coTUIButton::coTUIButton(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_BUTTON)
{
}


coTUIButton::~coTUIButton()
{
}

void coTUIButton::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_PRESSED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_RELEASED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

//TABLET_FILEBROWSER_BUTTON
coTUIFileBrowserButton::coTUIFileBrowserButton(const char* n, int pID)
	: coTUIElement(n, pID, TABLET_FILEBROWSER_BUTTON)
{
	mData = NULL;
	this->mVRBCId = 0;
	mAGData = NULL;
	mMode = coTUIFileBrowserButton::OPEN;

#ifdef FB_USE_AG
	AGData* locAGData = new AGData(this);
	mAGData = locAGData;
#endif

}

coTUIFileBrowserButton::coTUIFileBrowserButton(coTabletUI* tui, const char* n, int pID)
	: coTUIElement(tui, n, pID, TABLET_FILEBROWSER_BUTTON)
{
}

coTUIFileBrowserButton::~coTUIFileBrowserButton()
{
	this->mFileList.clear();
	this->mDirList.clear();
}

void coTUIFileBrowserButton::setClientList(Message& msg)
{
	//transmits a list of vrb clients as received by VRBServer
	//to the filebrowser gui
	TokenBuffer tb(&msg);
	//std::string entry;
	char* entry = NULL;
	int subtype;
	int size;
	int id;

	tb >> subtype;
	tb >> id;
	tb >> size;

	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_CLIENTS;
	rt << ID;
	rt << size;
	this->mClientList.clear();

	for (int i = 0; i < size; i++)
	{
		tb >> entry;

		this->mClientList.push_back(entry);
	}

	for (int i = 0; i < size; i++)
	{
		tb >> entry;

		std::string locString = (this->mClientList.at(i));
		locString = locString + " - ";
		locString = locString + entry;
		entry = (char*)(this->mClientList.at(i)).c_str();
		rt << locString.c_str();
	}

	tui()->send(rt);
}

void coTUIFileBrowserButton::parseMessage(TokenBuffer& tb)
{

	//Variable declaration
	int i;
	int locId = 0;

	locId = this->getID();
	tb >> i; //Which event occured?

	std::string backupLocation = this->mLocation; //Backup

	if (i == TABLET_PRESSED)
	{
		if (listener)
			listener->tabletPressEvent(this);
	}
	else if (i == TABLET_RELEASED)
	{
		if (listener)
			listener->tabletReleaseEvent(this);
	}
	else if (i == TABLET_FB_FILE_SEL)
	{
		//File selected for opening in OpenCOVER
		char* cstrFile = NULL;
		char* cstrDirectory = NULL;
		bool bLoadAll = false;
		std::string protocol;

		tb >> cstrFile;
		std::string strFile = cstrFile;
		tb >> cstrDirectory;
		tb >> bLoadAll;
		std::string strDirectory = cstrDirectory;

		listener->tabletEvent(this);
	}
	else if (i == TABLET_REQ_FILELIST)
	{
		//Retrieve new filelist based on filter and location
		//use instance of IData
	}
	else if (i == TABLET_REQ_DIRLIST)
	{
		//Retrieve new directory based on location
		//use instance of IData
	}
	else if (i == TABLET_REQ_FILTERCHANGE)
	{
		// TUI indicates that the selected filters in the dialog have changed
		// and therefore the content of the filter member in the data objects
		// is adjusted accordingly.
		// This also triggers a refresh of the file list in the filebrowser
	}
	else if (i == TABLET_REQ_DIRCHANGE)
	{
	}
	else if (i == TABLET_REQ_CLIENTS)
	{
	}
	else if (i == TABLET_REQ_MASTER)
	{
	}
	else if (i == TABLET_REQ_LOCATION)
	{
		TokenBuffer rtb;
		rtb << TABLET_SET_VALUE;
		rtb << TABLET_SET_LOCATION;
		rtb << ID;
		rtb << this->mLocation;

#ifdef FILEBROWSER_DEBUG
		std::cerr << "Host to be used for file lists: = " << this->mLocation.c_str() << std::endl;
#endif

		tui()->send(rtb);
	}
	else if (i == TABLET_SET_LOCATION)
	{
		char* location = NULL;
		tb >> location;
#ifdef FILEBROWSER_DEBUG
		std::cerr << "Setting new location!" << std::endl;
		std::cerr << " New location = " << location << std::endl;
#endif
	}
	else if (i == TABLET_REQ_HOME)
	{
	}
	else if (i == TABLET_FB_PATH_SELECT)
	{
		

		this->listener->tabletEvent(this);
	}
	else if (i == TABLET_REQ_DRIVES)
	{
		
	}
	else if (i == TABLET_REQ_VRBSTAT)
	{
		// Signals the availability of a VRB server to file dialog
		// used for enabling of RemoteClient button in FileBrowser
		TokenBuffer rtb;
		rtb << TABLET_SET_VALUE;
		rtb << TABLET_SET_VRBSTAT;
		rtb << ID;

		
		tui()->send(rtb);
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIFileBrowserButton::resend(bool create)
{
	coTUIElement::resend(create);
	TokenBuffer rt;

	// Send current Directory
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_CURDIR;
	rt << ID;tui()->send(rt);
}

void coTUIFileBrowserButton::setFileList(Message& msg)
{
	TokenBuffer tb(&msg);
	//std::string entry;
	char* entry = NULL;
	int subtype;
	int size;
	int id;

	tb >> subtype;
	tb >> id;
	tb >> size;

	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_FILELIST;
	rt << ID;
	rt << size;
	this->mFileList.clear();

	for (int i = 0; i < size; i++)
	{
		tb >> entry;

		rt << entry;
		this->mFileList.push_back(entry);
	}

	tui()->send(rt);
}

IData* coTUIFileBrowserButton::getData(std::string protocol)
{

	if (protocol.compare("") != 0)
	{
		return mDataRepo[protocol];
	}

	return mData;
}

IData* coTUIFileBrowserButton::getVRBData()
{
	return this->getData("vrb");
}

void coTUIFileBrowserButton::setDirList(Message& msg)
{
	TokenBuffer tb(&msg);
	char* entry;
	int subtype;
	int size;
	int id;

	tb >> subtype;
	tb >> id;
	tb >> size;

	this->mDirList.clear();

	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_DIRLIST;
	rt << ID;
	rt << size;

	for (int i = 0; i < size; i++)
	{
		tb >> entry;

		rt << entry;
		this->mDirList.push_back(entry);
	}

	tui()->send(rt);
}

void coTUIFileBrowserButton::setDrives(Message& ms)
{
	TokenBuffer tb(&ms);
	char* entry;
	int subtype;
	int size;
	int id;

	tb >> subtype;
	tb >> id;
	tb >> size;

	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_DRIVES;
	rt << ID;
	rt << size;

	for (int i = 0; i < size; i++)
	{
		tb >> entry;
		rt << entry;
	}

	tui()->send(rt);
}

void coTUIFileBrowserButton::setCurDir(Message& msg)
{
	TokenBuffer tb(&msg);
	int subtype = 0;
	int id = 0;
	char* dirValue = NULL;

	tb >> subtype;
	tb >> id;
	tb >> dirValue;

	//Save current path to Data object
	this->setCurDir(dirValue);
}

void coTUIFileBrowserButton::setCurDir(const char* dir)
{
	//Save current path to Data object
	std::string sdir = std::string(dir);
}

void coTUIFileBrowserButton::sendList(TokenBuffer& /*tb*/)
{
}

std::string coTUIFileBrowserButton::getFilename(const std::string url)
{
	IData* locData = NULL;

	std::string::size_type pos = 0;
	pos = url.find(':');
	std::string protocol = url.substr(0, pos);

	locData = mDataRepo[protocol];

	return std::string("");
}

void* coTUIFileBrowserButton::getFileHandle(bool sync)
{
	return NULL;
}

void coTUIFileBrowserButton::setMode(DialogMode mode)
{
	mMode = mode;
	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_MODE;
	rt << ID;
	rt << (int)mMode;

	tui()->send(rt);
}

// Method which is called from external of coTabletUI to allow
// OpenCOVER to initially set the range of available filter extensions
// used in the file dialog in the TUI.
void coTUIFileBrowserButton::setFilterList(std::string filterList)
{
	mFilterList = filterList;
	TokenBuffer rt;
	rt << TABLET_SET_VALUE;
	rt << TABLET_SET_FILTERLIST;
	rt << ID;
	rt << filterList.c_str();

	tui()->send(rt);
}

std::string coTUIFileBrowserButton::getSelectedPath()
{
	return "";
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIColorTriangle::coTUIColorTriangle(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_COLOR_TRIANGLE)
{
	red = 1.0;
	green = 1.0;
	blue = 1.0;
}


coTUIColorTriangle::~coTUIColorTriangle()
{
}

void coTUIColorTriangle::parseMessage(TokenBuffer& tb)
{
	int i, j;
	tb >> i;
	tb >> j;

	if (i == TABLET_RGBA)
	{
		tb >> red;
		tb >> green;
		tb >> blue;

		if (j == TABLET_RELEASED)
		{
			if (listener)
				listener->tabletReleaseEvent(this);
		}
		if (j == TABLET_PRESSED)
		{
			if (listener)
				listener->tabletEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIColorTriangle::setColor(float r, float g, float b)
{
	red = r;
	green = g;
	blue = b;
	setVal(TABLET_RED, r);
	setVal(TABLET_GREEN, g);
	setVal(TABLET_BLUE, b);
}

void coTUIColorTriangle::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_RED, red);
	setVal(TABLET_GREEN, green);
	setVal(TABLET_BLUE, blue);
}

coTUIColorButton::coTUIColorButton(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_COLOR_BUTTON)
{
	red = 1.0;
	green = 1.0;
	blue = 1.0;
	alpha = 1.0;
}


coTUIColorButton::~coTUIColorButton()
{
}

void coTUIColorButton::parseMessage(TokenBuffer& tb)
{
	int i, j;
	tb >> i;
	tb >> j;

	if (i == TABLET_RGBA)
	{
		tb >> red;
		tb >> green;
		tb >> blue;
		tb >> alpha;

		if (j == TABLET_RELEASED)
		{
			if (listener)
				listener->tabletReleaseEvent(this);
		}
		if (j == TABLET_PRESSED)
		{
			if (listener)
				listener->tabletEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIColorButton::setColor(float r, float g, float b, float a)
{
	red = r;
	green = g;
	blue = b;
	alpha = a;
	TokenBuffer t;
	t << TABLET_SET_VALUE;
	t << TABLET_RGBA;
	t << ID;
	t << red;
	t << green;
	t << blue;
	t << alpha;
	tui()->send(t);
}

void coTUIColorButton::resend(bool create)
{
	coTUIElement::resend(create);

	TokenBuffer t;
	t << TABLET_SET_VALUE;
	t << TABLET_RGBA;
	t << ID;
	t << red;
	t << green;
	t << blue;
	t << alpha;
	tui()->send(t);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIColorTab::coTUIColorTab(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_COLOR_TAB)
{
	red = 1.0;
	green = 1.0;
	blue = 1.0;
	alpha = 1.0;
}

coTUIColorTab::~coTUIColorTab()
{
}

void coTUIColorTab::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_RGBA)
	{
		tb >> red;
		tb >> green;
		tb >> blue;
		tb >> alpha;

		if (listener)
			listener->tabletEvent(this);
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIColorTab::setColor(float r, float g, float b, float a)
{
	red = r;
	green = g;
	blue = b;
	alpha = a;

	TokenBuffer t;
	t << TABLET_SET_VALUE;
	t << TABLET_RGBA;
	t << ID;
	t << r;
	t << g;
	t << b;
	t << a;
	tui()->send(t);
}

void coTUIColorTab::resend(bool create)
{
	coTUIElement::resend(create);
	setColor(red, green, blue, alpha);
}

//----------------------------------------------------------
//---------------------------------------------------------

coTUINav::coTUINav(const char* n, int pID)
	: coTUIElement(n, pID, TABLET_NAV_ELEMENT)
{
	down = false;
	x = 0;
	y = 0;
}

coTUINav::~coTUINav()
{
}

void coTUINav::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_PRESSED)
	{
		tb >> x;
		tb >> y;
		if (listener)
			listener->tabletPressEvent(this);
		down = true;
	}
	else if (i == TABLET_RELEASED)
	{
		tb >> x;
		tb >> y;
		if (listener)
			listener->tabletReleaseEvent(this);
		down = false;
	}
	else if (i == TABLET_POS)
	{
		tb >> x;
		tb >> y;
		if (listener)
			listener->tabletEvent(this);
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIBitmapButton::coTUIBitmapButton(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_BITMAP_BUTTON)
{
}

coTUIBitmapButton::coTUIBitmapButton(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_BITMAP_BUTTON)
{
}

coTUIBitmapButton::~coTUIBitmapButton()
{
}

void coTUIBitmapButton::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_PRESSED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_RELEASED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUILabel::coTUILabel(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_TEXT_FIELD)
{
	//color = Qt::black;
}

coTUILabel::coTUILabel(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_TEXT_FIELD)
{
	//color = Qt::black;
}

coTUILabel::~coTUILabel()
{
}

void coTUILabel::resend(bool create)
{
	coTUIElement::resend(create);
	//setColor(color);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUITabFolder::coTUITabFolder(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_TAB_FOLDER)
{
}

coTUITabFolder::coTUITabFolder(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_TAB_FOLDER)
{
}


coTUITabFolder::~coTUITabFolder()
{
}

void coTUITabFolder::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}


//----------------------------------------------------------
//----------------------------------------------------------

coTUIUITab::coTUIUITab(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_UI_TAB)
{
}

coTUIUITab::coTUIUITab(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_UI_TAB)
{

}

coTUIUITab::~coTUIUITab()
{
}

void coTUIUITab::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else if (i == TABLET_UI_COMMAND)
	{
		std::string target;
		uint64_t commandSize;

		tb >> target;
		tb >> commandSize;
		//command = QString::fromUtf16((const ushort*)tb.getBinary(commandSize));

	}
	else
	{
		cerr << "coTUIUITab::parseMessage err: unknown event " << i << endl;
	}
}


//----------------------------------------------------------
//----------------------------------------------------------

coTUITab::coTUITab(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_TAB)
{
}

coTUITab::coTUITab(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_TAB)
{

}
coTUITab::~coTUITab()
{
}

void coTUITab::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIAnnotationTab::coTUIAnnotationTab(const char* n, int pID)
	: coTUIElement(n, pID, TABLET_ANNOTATION_TAB)
{
}

coTUIAnnotationTab::~coTUIAnnotationTab()
{
}

void coTUIAnnotationTab::parseMessage(TokenBuffer& tb)
{
	listener->tabletDataEvent(this, tb);
}

void coTUIAnnotationTab::setNewButtonState(bool state)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ANNOTATION_CHANGE_NEW_BUTTON_STATE;
	tb << ID;
	tb << (char)state;

	tui()->send(tb);
}

void coTUIAnnotationTab::addAnnotation(int id)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ANNOTATION_NEW;
	tb << ID;
	tb << id;

	tui()->send(tb);
}

void coTUIAnnotationTab::deleteAnnotation(int mode, int id)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ANNOTATION_DELETE;
	tb << ID;
	tb << mode;
	tb << id;

	tui()->send(tb);
}

void coTUIAnnotationTab::setSelectedAnnotation(int id)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ANNOTATION_SET_SELECTION;
	tb << ID;
	tb << id;

	tui()->send(tb);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIFunctionEditorTab::coTUIFunctionEditorTab(const char* tabName, int pID)
	: coTUIElement(tabName, pID, TABLET_FUNCEDIT_TAB)
{
	tfDim = 1;
	histogramData = NULL;
}

coTUIFunctionEditorTab::~coTUIFunctionEditorTab()
{
	if (histogramData)
		delete[] histogramData;
}

int coTUIFunctionEditorTab::getDimension() const
{
	return tfDim;
	;
}

void coTUIFunctionEditorTab::setDimension(int dim)
{
	tfDim = dim;
}

void coTUIFunctionEditorTab::resend(bool create)
{
	coTUIElement::resend(create);

	//resend the transfer function information
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_TF_WIDGET_LIST;
	tb << ID;

	//send dimension
	int dim = getDimension();
	tb << dim;

	switch (dim)
	{
	case 1:
	{
		tb << (uint32_t)colorPoints.size();
		for (uint32_t i = 0; i < colorPoints.size(); ++i)
		{
			tb << colorPoints[i].r;
			tb << colorPoints[i].g;
			tb << colorPoints[i].b;
			tb << colorPoints[i].x;
		}

		// then, alpha widgets
		tb << (uint32_t)alphaPoints.size();
		for (uint32_t i = 0; i < alphaPoints.size(); ++i)
		{
			tb << alphaPoints[i].kind;

			switch (alphaPoints[i].kind)
			{
			case TF_PYRAMID:
			{
				tb << alphaPoints[i].alpha;
				tb << alphaPoints[i].xPos;
				tb << alphaPoints[i].xParam1; //xb
				tb << alphaPoints[i].xParam2; //xt
			}
			break;

			case TF_FREE:
			{
				tb << (uint32_t)alphaPoints[i].additionalDataElems; //data lenght;

				//every elem has a position and an alpha value
				for (int j = 0; j < alphaPoints[i].additionalDataElems * 2; j += 2)
				{
					tb << alphaPoints[i].additionalData[j];
					tb << alphaPoints[i].additionalData[j + 1];
				}
			}
			break;
			default:
				break;
			}
		}
	}
	break;

	case 2:
	{
		tb << (uint32_t)colorPoints.size();
		for (uint32_t i = 0; i < colorPoints.size(); ++i)
		{
			tb << colorPoints[i].r;
			tb << colorPoints[i].g;
			tb << colorPoints[i].b;
			tb << colorPoints[i].x;
			tb << colorPoints[i].y;
		}

		// then, alpha widgets
		tb << (uint32_t)alphaPoints.size();
		for (uint32_t i = 0; i < alphaPoints.size(); ++i)
		{
			tb << alphaPoints[i].kind;

			switch (alphaPoints[i].kind)
			{
			case TF_PYRAMID:
			{
				tb << alphaPoints[i].alpha;
				tb << alphaPoints[i].xPos;
				tb << alphaPoints[i].xParam1; //xb
				tb << alphaPoints[i].xParam2; //xt
				tb << alphaPoints[i].yPos;
				tb << alphaPoints[i].yParam1; //xb
				tb << alphaPoints[i].yParam2; //xt

				tb << alphaPoints[i].ownColor;
				if (alphaPoints[i].ownColor)
				{
					tb << alphaPoints[i].r;
					tb << alphaPoints[i].g;
					tb << alphaPoints[i].b;
				}
			}
			break;

			case TF_BELL:
			{
				tb << alphaPoints[i].alpha;
				tb << alphaPoints[i].xPos;
				tb << alphaPoints[i].xParam1; //xb
				tb << alphaPoints[i].yPos;
				tb << alphaPoints[i].yParam1; //xb

				tb << alphaPoints[i].ownColor;
				if (alphaPoints[i].ownColor)
				{
					tb << alphaPoints[i].r;
					tb << alphaPoints[i].g;
					tb << alphaPoints[i].b;
				}
			}
			break;

			case TF_CUSTOM_2D:
			{
				tb << alphaPoints[i].alpha;
				tb << alphaPoints[i].alpha; //alpha2
				tb << alphaPoints[i].xPos;
				tb << alphaPoints[i].yPos;
				//tb << extrude; //TODO!
				tb << 1;

				tb << alphaPoints[i].ownColor;
				if (alphaPoints[i].ownColor)
				{
					tb << alphaPoints[i].r;
					tb << alphaPoints[i].g;
					tb << alphaPoints[i].b;
				}

				tb << alphaPoints[i].additionalDataElems;
				for (int j = 0; j < alphaPoints[i].additionalDataElems * 3; j += 3)
				{
					tb << alphaPoints[i].additionalData[j];
					tb << alphaPoints[i].additionalData[j + 1];
					tb << alphaPoints[i].additionalData[j + 2];
				}
			}
			break;

			case TF_MAP:
			{
				tb << alphaPoints[i].alpha;
				tb << alphaPoints[i].xPos;
				tb << alphaPoints[i].xParam1; //xb
				tb << alphaPoints[i].yPos;
				tb << alphaPoints[i].yParam1; //xb

				tb << alphaPoints[i].ownColor;
				if (alphaPoints[i].ownColor)
				{
					tb << alphaPoints[i].r;
					tb << alphaPoints[i].g;
					tb << alphaPoints[i].b;
				}

				// store map info
				tb << alphaPoints[i].additionalDataElems;
				for (int j = 0; j < alphaPoints[i].additionalDataElems; ++j)
				{
					tb << alphaPoints[i].additionalData[j];
				}
			}
			break;

			default:
				break;
			}
		}
	}
	break;

	default:
		// we do not handle higher dimension for now
		break;
	}

	// send TFE info
	tui()->send(tb);

	// send histogram
	sendHistogramData();
}

void coTUIFunctionEditorTab::sendHistogramData()
{
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_TF_HISTOGRAM;
	tb << ID;

	tb << tfDim;

	if (tfDim == 1)
	{
		if (histogramData == NULL)
			tb << 0;
		else
		{
			tb << (uint32_t)histogramBuckets;
			for (uint32_t i = 0; i < histogramBuckets; ++i)
			{
				tb << histogramData[i];
			}
		}
	}
	else //if (tfDim == 2) send anyway for volume dimensions > 2
	{
		if (histogramData == NULL)
		{
			tb << 0;
			tb << 0;
		}
		else
		{
			tb << (uint32_t)histogramBuckets;
			tb << (uint32_t)histogramBuckets;
			for (uint32_t i = 0; i < histogramBuckets * histogramBuckets; ++i)
			{
				tb << histogramData[i];
			}
		}
	}

	tui()->send(tb);
}

void coTUIFunctionEditorTab::parseMessage(TokenBuffer& tb)
{
	int type;
	tb >> type;

	if (type == TABLET_TF_WIDGET_LIST)
	{
		// get TF size (1 = 1D, 2 = 2D)
		tb >> tfDim;

		// if the dimensionality is different (do not match that of data)
		// adjust that.

		// This is done in VolumePlugin::tabletPressEvent

		int numPoints;
		colorPoints.clear();
		for (size_t i = 0; i < alphaPoints.size(); ++i)
			if (alphaPoints[i].additionalDataElems > 0)
				delete[] alphaPoints[i].additionalData;

		alphaPoints.clear();

		if (tfDim == 1)
		{
			//1) color points
			tb >> numPoints;
			for (int i = 0; i < numPoints; ++i)
			{
				// for each entry: r, g, b channels (float), pos (float)
				// but the updateColorMap function expects rgbax, so lets
				// add an opaque alpha component. We deal with alpha below

				colorPoint cp;

				tb >> cp.r;
				tb >> cp.g;
				tb >> cp.b;
				tb >> cp.x;
				cp.y = -1.0f;

				colorPoints.push_back(cp);
			}

			// 2) the alpha widgets
			tb >> numPoints;

			for (int i = 0; i < numPoints; ++i)
			{
				int widgetType;
				//TF_PYRAMID == 1, TF_CUSTOM == 4
				//
				tb >> widgetType;
				switch (widgetType)
				{
				case TF_PYRAMID:
				{
					alphaPoint ap;
					ap.kind = widgetType;
					tb >> ap.alpha;
					tb >> ap.xPos;
					tb >> ap.xParam1;
					tb >> ap.xParam2;
					ap.yPos = -1.0f;
					ap.additionalDataElems = 0;
					ap.additionalData = NULL;
					alphaPoints.push_back(ap);
				}
				break;

				case TF_FREE:
				{
					alphaPoint ap;
					ap.kind = widgetType;
					ap.alpha = 1.0f;
					ap.xPos = 0.5f;
					ap.xParam1 = 1.0f;
					ap.xParam2 = 1.0f;
					ap.yPos = -1.0f;
					tb >> ap.additionalDataElems;

					// each "element" has 2 components (pos and alpha)
					if (ap.additionalDataElems > 0)
					{
						ap.additionalData = new float[ap.additionalDataElems * 2];
						for (int j = 0; j < ap.additionalDataElems * 2; j += 2)
						{
							float x, alpha;
							tb >> x; //pos
							tb >> alpha; //alpha value;
							ap.additionalData[j] = x;
							ap.additionalData[j + 1] = alpha;
						}
					}
					else
					{
						ap.additionalData = NULL;
					}
					alphaPoints.push_back(ap);
				}
				break;
				}
			}
		}
		else //dim == 2
		{
			//1) color points
			tb >> numPoints;
			for (int i = 0; i < numPoints; ++i)
			{
				// for each entry: r, g, b channels (float), pos (float)
				// but the updateColorMap function expects rgbax, so lets
				// add an opaque alpha component. We deal with alpha below

				colorPoint cp;

				tb >> cp.r;
				tb >> cp.g;
				tb >> cp.b;
				tb >> cp.x;
				tb >> cp.y;

				colorPoints.push_back(cp);
			}

			// 2) the alpha widgets
			tb >> numPoints;

			for (int i = 0; i < numPoints; ++i)
			{
				int widgetType;
				//TF_PYRAMID == 1, TF_CUSTOM == 4
				//
				tb >> widgetType;
				switch (widgetType)
				{
				case TF_PYRAMID:
				{
					alphaPoint ap;
					ap.kind = widgetType;
					tb >> ap.alpha;
					tb >> ap.xPos;
					tb >> ap.xParam1;
					tb >> ap.xParam2;
					tb >> ap.yPos;
					tb >> ap.yParam1;
					tb >> ap.yParam2;

					tb >> ap.ownColor;
					if (ap.ownColor)
					{
						tb >> ap.r;
						tb >> ap.g;
						tb >> ap.b;
					}

					ap.additionalDataElems = 0;
					ap.additionalData = NULL;
					alphaPoints.push_back(ap);
				}
				break;

				case TF_BELL:
				{
					alphaPoint ap;
					ap.kind = widgetType;
					tb >> ap.alpha;
					tb >> ap.xPos;
					tb >> ap.xParam1;
					tb >> ap.yPos;
					tb >> ap.yParam1;

					tb >> ap.ownColor;
					if (ap.ownColor)
					{
						tb >> ap.r;
						tb >> ap.g;
						tb >> ap.b;
					}

					ap.additionalDataElems = 0;
					ap.additionalData = NULL;
					alphaPoints.push_back(ap);
				}
				break;

				case TF_CUSTOM_2D:
					//assert(false && "TODO!");
					break;

				case TF_MAP:
				{
					alphaPoint ap;
					ap.kind = widgetType;
					tb >> ap.alpha;
					tb >> ap.xPos;
					tb >> ap.xParam1;
					tb >> ap.yPos;
					tb >> ap.yParam1;

					tb >> ap.ownColor;
					if (ap.ownColor)
					{
						tb >> ap.r;
						tb >> ap.g;
						tb >> ap.b;
					}

					// store map info
					tb >> ap.additionalDataElems;
					if (ap.additionalDataElems > 0)
					{
						ap.additionalData = new float[ap.additionalDataElems];

						for (int j = 0; j < ap.additionalDataElems; ++j)
							tb >> ap.additionalData[j];
					}
					else
					{
						ap.additionalData = NULL;
					}

					alphaPoints.push_back(ap);
				}
				break;
				}
			}
		}
	}

	listener->tabletPressEvent(this);
}

//----------------------------------------------------------
//----------------------------------------------------------
coTUISplitter::coTUISplitter(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_SPLITTER)
{
	shape = coTUIFrame::StyledPanel;
	style = coTUIFrame::Sunken;
	setShape(shape);
	setStyle(style);
	setOrientation(orientation);
}


coTUISplitter::~coTUISplitter()
{
}

void coTUISplitter::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUISplitter::resend(bool create)
{
	coTUIElement::resend(create);
	setShape(shape);
	setStyle(style);
	setOrientation(orientation);
}

void coTUISplitter::setShape(int s)
{
	TokenBuffer tb;
	shape = s;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SHAPE;
	tb << ID;
	tb << shape;
	tui()->send(tb);
}

void coTUISplitter::setStyle(int t)
{
	TokenBuffer tb;
	style = t;
	tb << TABLET_SET_VALUE;
	tb << TABLET_STYLE;
	tb << ID;
	tb << (style | shape);
	tui()->send(tb);
}

void coTUISplitter::setOrientation(int orient)
{
	TokenBuffer tb;
	orientation = orient;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ORIENTATION;
	tb << ID;
	tb << orientation;
	tui()->send(tb);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIFrame::coTUIFrame(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_FRAME)
{
	style = Sunken;
	shape = StyledPanel;
	setShape(shape);
	setStyle(style);
}

coTUIFrame::coTUIFrame(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_FRAME)
{
	style = Sunken;
	shape = StyledPanel;
	setShape(shape);
	setStyle(style);
}


coTUIFrame::~coTUIFrame()
{
}

void coTUIFrame::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIFrame::setShape(int s)
{
	TokenBuffer tb;
	shape = s;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SHAPE;
	tb << ID;
	tb << shape;
	tui()->send(tb);
}

void coTUIFrame::setStyle(int t)
{
	TokenBuffer tb;
	style = t;
	tb << TABLET_SET_VALUE;
	tb << TABLET_STYLE;
	tb << ID;
	tb << (style | shape);
	tui()->send(tb);
}

void coTUIFrame::resend(bool create)
{
	coTUIElement::resend(create);
	setShape(shape);
	setStyle(style);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIToggleButton::coTUIToggleButton(const std::string& n, int pID, bool s)
	: coTUIElement(n, pID, TABLET_TOGGLE_BUTTON)
{
	state = s;
	setVal(state);
}

coTUIToggleButton::coTUIToggleButton(coTabletUI* tui, const std::string& n, int pID, bool s)
	: coTUIElement(tui, n, pID, TABLET_TOGGLE_BUTTON)
{
	state = s;
	setVal(state);
}


coTUIToggleButton::~coTUIToggleButton()
{
}

void coTUIToggleButton::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		state = true;
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		state = false;
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIToggleButton::setState(bool s)
{
	if (s != state) // don't send unnecessary state changes
	{
		state = s;
		setVal(state);
	}
}

bool coTUIToggleButton::getState() const
{
	return state;
}

void coTUIToggleButton::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(state);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIToggleBitmapButton::coTUIToggleBitmapButton(const std::string& n, const std::string& down, int pID, bool state)
	: coTUIElement(n, pID, TABLET_BITMAP_TOGGLE_BUTTON)
{
	bmpUp = n;
	bmpDown = down;

	setVal(bmpDown);
	setVal(state);
}

coTUIToggleBitmapButton::~coTUIToggleBitmapButton()
{
}

void coTUIToggleBitmapButton::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		state = true;
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		state = false;
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}

void coTUIToggleBitmapButton::setState(bool s)
{
	if (s != state) // don't send unnecessary state changes
	{
		state = s;
		setVal(state);
	}
}

bool coTUIToggleBitmapButton::getState() const
{
	return state;
}

void coTUIToggleBitmapButton::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(bmpDown);
	setVal(state);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIMessageBox::coTUIMessageBox(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_MESSAGE_BOX)
{
}


coTUIMessageBox::~coTUIMessageBox()
{
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIEditField::coTUIEditField(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_EDIT_FIELD)
{
	this->text = name;
	immediate = false;
}

coTUIEditField::coTUIEditField(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_EDIT_FIELD)
{
	this->text = name;
	immediate = false;
}

coTUIEditField::~coTUIEditField()
{
}

void coTUIEditField::setImmediate(bool i)
{
	immediate = i;
	setVal(immediate);
}

void coTUIEditField::parseMessage(TokenBuffer& tb)
{
	char* m;
	tb >> m;
	text = m;
	if (listener)
		listener->tabletEvent(this);
}

void coTUIEditField::setPasswordMode(bool b)
{
	setVal(TABLET_ECHOMODE, (int)b);
}

void coTUIEditField::setIPAddressMode(bool b)
{
	setVal(TABLET_IPADDRESS, (int)b);
}

void coTUIEditField::setText(const std::string& t)
{
	text = t;
	setVal(text);
}

const std::string& coTUIEditField::getText() const
{
	return text;
}

void coTUIEditField::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(text);
	setVal(immediate);
}

//----------------------------------------------------------
//----------------------------------------------------------
//##########################################################

coTUIEditTextField::coTUIEditTextField(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_TEXT_EDIT_FIELD)
{
	text = name;
	immediate = false;
}

coTUIEditTextField::coTUIEditTextField(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_TEXT_EDIT_FIELD)
{
	text = name;
	immediate = false;
}


coTUIEditTextField::~coTUIEditTextField()
{
}

void coTUIEditTextField::setImmediate(bool i)
{
	immediate = i;
	setVal(immediate);
}

void coTUIEditTextField::parseMessage(TokenBuffer& tb)
{
	char* m;
	tb >> m;
	text = m;
	if (listener)
		listener->tabletEvent(this);
}

void coTUIEditTextField::setText(const std::string& t)
{
	text = t;
	setVal(text);
}

const std::string& coTUIEditTextField::getText() const
{
	return text;
}

void coTUIEditTextField::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(text);
	setVal(immediate);
}

//##########################################################
//----------------------------------------------------------
//----------------------------------------------------------

coTUIEditIntField::coTUIEditIntField(const std::string& n, int pID, int def)
	: coTUIElement(n, pID, TABLET_INT_EDIT_FIELD)
{
	value = def;
	immediate = 0;
	setVal(value);
}

coTUIEditIntField::coTUIEditIntField(coTabletUI* tui, const std::string& n, int pID, int def)
	: coTUIElement(tui, n, pID, TABLET_INT_EDIT_FIELD)
{
	value = def;
	immediate = 0;
	setVal(value);
}


coTUIEditIntField::~coTUIEditIntField()
{
}

void coTUIEditIntField::setImmediate(bool i)
{
	immediate = i;
	setVal(immediate);
}

void coTUIEditIntField::parseMessage(TokenBuffer& tb)
{
	tb >> value;
	if (listener)
		listener->tabletEvent(this);
}

std::string coTUIEditIntField::getText() const
{
	return "";
}

void coTUIEditIntField::setMin(int min)
{
	//cerr << "coTUIEditIntField::setMin " << min << endl;
	this->min = min;
	setVal(TABLET_MIN, min);
}

void coTUIEditIntField::setMax(int max)
{
	//cerr << "coTUIEditIntField::setMax " << max << endl;
	this->max = max;
	setVal(TABLET_MAX, max);
}

void coTUIEditIntField::setValue(int val)
{
	if (value != val)
	{
		value = val;
		setVal(value);
	}
}

void coTUIEditIntField::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MIN, min);
	setVal(TABLET_MAX, max);
	setVal(value);
	setVal(immediate);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIEditFloatField::coTUIEditFloatField(const std::string& n, int pID, float def)
	: coTUIElement(n, pID, TABLET_FLOAT_EDIT_FIELD)
{
	value = def;
	setVal(value);
	immediate = 0;
}

coTUIEditFloatField::coTUIEditFloatField(coTabletUI* tui, const std::string& n, int pID, float def)
	: coTUIElement(tui, n, pID, TABLET_FLOAT_EDIT_FIELD)
{
	value = def;
	setVal(value);
	immediate = 0;
}


coTUIEditFloatField::~coTUIEditFloatField()
{
}

void coTUIEditFloatField::setImmediate(bool i)
{
	immediate = i;
	setVal(immediate);
}

void coTUIEditFloatField::parseMessage(TokenBuffer& tb)
{
	tb >> value;
	if (listener)
		listener->tabletEvent(this);
}

void coTUIEditFloatField::setValue(float val)
{
	if (value != val)
	{
		value = val;
		setVal(value);
	}
}

void coTUIEditFloatField::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(value);
	setVal(immediate);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUISpinEditfield::coTUISpinEditfield(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_SPIN_EDIT_FIELD)
{
	actValue = 0;
	minValue = 0;
	maxValue = 100;
	step = 1;
	setVal(actValue);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_STEP, step);
}


coTUISpinEditfield::~coTUISpinEditfield()
{
}

void coTUISpinEditfield::parseMessage(TokenBuffer& tb)
{
	tb >> actValue;
	if (listener)
		listener->tabletEvent(this);
}

void coTUISpinEditfield::setPosition(int newV)
{

	if (actValue != newV)
	{
		actValue = newV;
		setVal(actValue);
	}
}

void coTUISpinEditfield::setStep(int newV)
{
	step = newV;
	setVal(TABLET_STEP, step);
}

void coTUISpinEditfield::setMin(int minV)
{
	minValue = minV;
	setVal(TABLET_MIN, minValue);
}

void coTUISpinEditfield::setMax(int maxV)
{
	maxValue = maxV;
	setVal(TABLET_MAX, maxValue);
}

void coTUISpinEditfield::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_STEP, step);
	setVal(actValue);
}

//----------------------------------------------------------
//----------------------------------------------------------
coTUITextSpinEditField::coTUITextSpinEditField(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_TEXT_SPIN_EDIT_FIELD)
{
	text = "";
	minValue = 0;
	maxValue = 100;
	step = 1;
	setVal(text);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_STEP, step);
}


coTUITextSpinEditField::~coTUITextSpinEditField()
{
}

void coTUITextSpinEditField::parseMessage(TokenBuffer& tb)
{
	char* m;
	tb >> m;
	text = m;
	if (listener)
		listener->tabletEvent(this);
}

void coTUITextSpinEditField::setStep(int newV)
{
	step = newV;
	setVal(TABLET_STEP, step);
}

void coTUITextSpinEditField::setMin(int minV)
{
	minValue = minV;
	setVal(TABLET_MIN, minValue);
}

void coTUITextSpinEditField::setMax(int maxV)
{
	maxValue = maxV;
	setVal(TABLET_MAX, maxValue);
}

void coTUITextSpinEditField::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_STEP, step);
	setVal(text);
}

void coTUITextSpinEditField::setText(const std::string& t)
{
	text = t;
	setVal(text);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIProgressBar::coTUIProgressBar(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_PROGRESS_BAR)
{
	actValue = 0;
	maxValue = 100;
}

coTUIProgressBar::~coTUIProgressBar()
{
}

void coTUIProgressBar::setValue(int newV)
{
	if (actValue != newV)
	{
		actValue = newV;
		setVal(actValue);
	}
}

void coTUIProgressBar::setMax(int maxV)
{
	maxValue = maxV;
	setVal(TABLET_MAX, maxValue);
}

void coTUIProgressBar::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MAX, maxValue);
	setVal(actValue);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIFloatSlider::coTUIFloatSlider(const std::string& n, int pID, bool s)
	: coTUIElement(n, pID, TABLET_FLOAT_SLIDER)
{
	label = "";
	actValue = 0;
	minValue = 0;
	maxValue = 0;
	ticks = 10;

	orientation = s;
	setVal(orientation);
}

coTUIFloatSlider::coTUIFloatSlider(coTabletUI* tui, const std::string& n, int pID, bool s)
	: coTUIElement(tui, n, pID, TABLET_FLOAT_SLIDER)
{
	label = "";
	actValue = 0;
	minValue = 0;
	maxValue = 0;
	ticks = 10;

	orientation = s;
	setVal(orientation);
}

coTUIFloatSlider::~coTUIFloatSlider()
{
}

void coTUIFloatSlider::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	tb >> actValue;
	if (i == TABLET_PRESSED)
	{
		if (listener)
		{
			listener->tabletPressEvent(this);
			listener->tabletEvent(this);
		}
	}
	else if (i == TABLET_RELEASED)
	{
		if (listener)
		{
			listener->tabletReleaseEvent(this);
			listener->tabletEvent(this);
		}
	}
	else
	{
		if (listener)
			listener->tabletEvent(this);
	}
}

void coTUIFloatSlider::setValue(float newV)
{
	if (actValue != newV)
	{
		actValue = newV;
		setVal(actValue);
	}
}

void coTUIFloatSlider::setTicks(int newV)
{
	if (ticks != newV)
	{
		ticks = newV;
		setVal(TABLET_NUM_TICKS, ticks);
	}
}

void coTUIFloatSlider::setMin(float minV)
{
	if (minValue != minV)
	{
		minValue = minV;
		setVal(TABLET_MIN, minValue);
	}
}

void coTUIFloatSlider::setMax(float maxV)
{
	if (maxValue != maxV)
	{
		maxValue = maxV;
		setVal(TABLET_MAX, maxValue);
	}
}

void coTUIFloatSlider::setRange(float minV, float maxV)
{
	minValue = minV;
	maxValue = maxV;
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
}

void coTUIFloatSlider::setOrientation(bool o)
{
	orientation = o;
	setVal(orientation);
}

void coTUIFloatSlider::setLogarithmic(bool val)
{
	logarithmic = val;
	setVal(TABLET_SLIDER_SCALE, logarithmic ? TABLET_SLIDER_LOGARITHMIC : TABLET_SLIDER_LINEAR);
}

void coTUIFloatSlider::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_NUM_TICKS, ticks);
	setVal(TABLET_SLIDER_SCALE, logarithmic ? TABLET_SLIDER_LOGARITHMIC : TABLET_SLIDER_LINEAR);
	setVal(actValue);
	setVal(orientation);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUISlider::coTUISlider(const std::string& n, int pID, bool s)
	: coTUIElement(n, pID, TABLET_SLIDER)
	, actValue(0)
{
	label = "";
	actValue = 0;
	minValue = 0;
	maxValue = 0;
	orientation = s;
	setVal(orientation);
}

coTUISlider::coTUISlider(coTabletUI* tui, const std::string& n, int pID, bool s)
	: coTUIElement(tui, n, pID, TABLET_SLIDER)
	, actValue(0)
{
	label = "";
	actValue = 0;
	minValue = 0;
	maxValue = 0;
	orientation = s;
	setVal(orientation);
}


coTUISlider::~coTUISlider()
{
}

void coTUISlider::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	tb >> actValue;
	if (i == TABLET_PRESSED)
	{
		if (listener)
		{
			listener->tabletPressEvent(this);
			listener->tabletEvent(this);
		}
	}
	else if (i == TABLET_RELEASED)
	{
		if (listener)
		{
			listener->tabletReleaseEvent(this);
			listener->tabletEvent(this);
		}
	}
	else
	{
		if (listener)
			listener->tabletEvent(this);
	}
}

void coTUISlider::setValue(int newV)
{

	if (actValue != newV)
	{
		actValue = newV;
		setVal(actValue);
	}
}

void coTUISlider::setTicks(int newV)
{
	if (ticks != newV)
	{
		ticks = newV;
		setVal(TABLET_NUM_TICKS, ticks);
	}
}

void coTUISlider::setMin(int minV)
{
	minValue = minV;
	setVal(TABLET_MIN, minValue);
}

void coTUISlider::setMax(int maxV)
{
	maxValue = maxV;
	setVal(TABLET_MAX, maxValue);
}

void coTUISlider::setRange(int minV, int maxV)
{
	minValue = minV;
	maxValue = maxV;
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
}

void coTUISlider::setOrientation(bool o)
{
	orientation = o;
	setVal(orientation);
}

void coTUISlider::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(TABLET_MIN, minValue);
	setVal(TABLET_MAX, maxValue);
	setVal(TABLET_NUM_TICKS, ticks);
	setVal(actValue);
	setVal(orientation);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIComboBox::coTUIComboBox(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_COMBOBOX)
{
	label = "";
	text = "";
	selection = -1;
}

coTUIComboBox::coTUIComboBox(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_COMBOBOX)
{
	label = "";
	text = "";
	selection = -1;
}


coTUIComboBox::~coTUIComboBox()
{
}

void coTUIComboBox::parseMessage(TokenBuffer& tb)
{
	tb >> text;
	int i = 0;
	selection = -1;
	for (const auto& it : elements)
	{
		if (it == text)
		{
			selection = i;
			break;
		}
		i++;
	}
	if (listener)
		listener->tabletEvent(this);
}

void coTUIComboBox::addEntry(const std::string& t)
{
	elements.push_back(t);
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ADD_ENTRY;
	tb << ID;
	tb << t.c_str();
	tui()->send(tb);
}

void coTUIComboBox::delEntry(const std::string& t)
{
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_REMOVE_ENTRY;
	tb << ID;
	tb << t;
	tui()->send(tb);

	for (const auto& it : elements)
	{
		if (it == t)
		{
			elements.remove(it);
			break;
		}
	}
}

int coTUIComboBox::getNumEntries()
{
	return elements.size();
}

void coTUIComboBox::clear()
{
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_REMOVE_ALL;
	tb << ID;
	tui()->send(tb);
	elements.clear();
}

void coTUIComboBox::setSelectedText(const std::string& t)
{
	text = t;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SELECT_ENTRY;
	tb << ID;
	tb << text;
	tui()->send(tb);
	int i = 0;
	selection = -1;
	for (const auto& it : elements)
	{
		if (it == text)
		{
			selection = i;
			break;
		}
		i++;
	}
}

const std::string& coTUIComboBox::getSelectedText() const
{
	return text;
}

int coTUIComboBox::getSelectedEntry() const
{
	return selection;
}

void coTUIComboBox::setSelectedEntry(int e)
{
	selection = e;
	if (e >= elements.size())
		selection = elements.size() - 1;
	if (selection < 0)
		return;
	std::string selectedEntry;
	int i = 0;
	for (const auto& it : elements)
	{
		if (i==selection)
		{
			selectedEntry = it;
			break;
		}
		i++;
	}
	text = selectedEntry;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SELECT_ENTRY;
	tb << ID;
	tb << text;
	tui()->send(tb);
}

void coTUIComboBox::resend(bool create)
{
	coTUIElement::resend(create);
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_REMOVE_ALL;
		tb << ID;
		tui()->send(tb);
	}
	for (const auto& it : elements)
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_ADD_ENTRY;
		tb << ID;
		tb << it;
		tui()->send(tb);
	}
	if (text != "")
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_SELECT_ENTRY;
		tb << ID;
		tb << text;
		tui()->send(tb);
	}
}

//----------------------------------------------------------
//----------------------------------------------------------

coTUIListBox::coTUIListBox(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_LISTBOX)
{
	text = "";
	selection = -1;
}

coTUIListBox::~coTUIListBox()
{
}

void coTUIListBox::parseMessage(TokenBuffer& tb)
{
	char* m;
	tb >> m;
	text = m;
	int i = 0;
	for (const auto& it : elements)
	{
		if (it == text)
		{
			selection = i;
			break;
		}
		i++;
	}
	if (listener)
		listener->tabletEvent(this);
}

void coTUIListBox::addEntry(const std::string& t)
{
	elements.push_back(t);
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ADD_ENTRY;
	tb << ID;
	tb << t.c_str();
	tui()->send(tb);
}

void coTUIListBox::delEntry(const std::string& t)
{
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_REMOVE_ENTRY;
	tb << ID;
	tb << t.c_str();
	tui()->send(tb);
	for (const auto& it : elements)
	{
		if (it == t)
		{
			elements.remove(it);
			break;
		}
	}
}

void coTUIListBox::setSelectedText(const std::string& t)
{
	text = t;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SELECT_ENTRY;
	tb << ID;
	tb << text.c_str();
	tui()->send(tb);
	int i = 0;
	for (const auto& it : elements)
	{
		if (it == text)
		{
			selection = i;
			break;
		}
		i++;
	}
}

const std::string& coTUIListBox::getSelectedText() const
{
	return text;
}

int coTUIListBox::getSelectedEntry() const
{
	return selection;
}

void coTUIListBox::setSelectedEntry(int e)
{
	selection = e;
	if (e >= elements.size())
		selection = elements.size() - 1;
	if (selection < 0)
		return;
	std::string selectedEntry;
	int i = 0;
	for (const auto& it : elements)
	{
		if (i == selection)
		{
			selectedEntry = it;
			break;
		}
		i++;
	}
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SELECT_ENTRY;
	tb << ID;
	tb << text.c_str();
	tui()->send(tb);
}

void coTUIListBox::resend(bool create)
{
	coTUIElement::resend(create);


	for (const auto& it : elements)
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_ADD_ENTRY;
		tb << ID;
		tb << it;
		tui()->send(tb);
	}
	if (text != "")
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_SELECT_ENTRY;
		tb << ID;
		tb << text.c_str();
		tui()->send(tb);
	}
}

//----------------------------------------------------------
//----------------------------------------------------------

MapData::MapData(const char* pname, float pox, float poy, float pxSize, float pySize, float pheight)
{
	name = new char[strlen(pname) + 1];
	strcpy(name, pname);
	ox = pox;
	oy = poy;
	xSize = pxSize;
	ySize = pySize;
	height = pheight;
}

MapData::~MapData()
{
	delete[] name;
}

coTUIMap::coTUIMap(const char* n, int pID)
	: coTUIElement(n, pID, TABLET_MAP)
{
}

coTUIMap::~coTUIMap()
{

}

void coTUIMap::parseMessage(TokenBuffer& tb)
{
	tb >> mapNum;
	tb >> xPos;
	tb >> yPos;
	tb >> height;

	if (listener)
		listener->tabletEvent(this);
}

void coTUIMap::addMap(const char* name, float ox, float oy, float xSize, float ySize, float height)
{
	MapData* md = new MapData(name, ox, oy, xSize, ySize, height);
	maps.push_back(md);
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_ADD_MAP;
	tb << ID;
	tb << md->name;
	tb << md->ox;
	tb << md->oy;
	tb << md->xSize;
	tb << md->ySize;
	tb << md->height;
	tui()->send(tb);
}

void coTUIMap::resend(bool create)
{
	coTUIElement::resend(create);


	for (const auto& it : maps)
	{
		TokenBuffer tb;
		tb << TABLET_SET_VALUE;
		tb << TABLET_ADD_MAP;
		tb << ID;
		tb << it->name;
		tb << it->ox;
		tb << it->oy;
		tb << it->xSize;
		tb << it->ySize;
		tb << it->height;
		tui()->send(tb);
	}
}



coTUIEarthMap::coTUIEarthMap(const char* n, int pID)
	: coTUIElement(n, pID, TABLET_EARTHMAP)
{
}

coTUIEarthMap::~coTUIEarthMap()
{

}

void coTUIEarthMap::parseMessage(TokenBuffer& tb)
{
	tb >> latitude;
	tb >> longitude;
	tb >> altitude;

	if (listener)
		listener->tabletEvent(this);
}

void coTUIEarthMap::setPosition(float lat, float longi, float alt)
{
	latitude = lat;
	longitude = longi;
	altitude = alt;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_FLOAT;
	tb << ID;
	tb << latitude;
	tb << longitude;
	tb << altitude;
	tui()->send(tb);
}

void coTUIEarthMap::addPathNode(float latitude, float longitude)
{
	path.push_back(pair<float, float>(latitude, longitude));
}

void coTUIEarthMap::updatePath()
{
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_GEO_PATH;
	tb << ID;
	tb << (unsigned)path.size();
	for (auto p = path.begin(); p != path.end(); p++)
	{
		tb << p->first;
		tb << p->second;
	}
	tui()->send(tb);
}
void coTUIEarthMap::setMinMax(float minH, float maxH)
{
	minHeight = minH;
	maxHeight = maxH;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_MIN_MAX;
	tb << ID;
	tb << minHeight;
	tb << maxHeight;
	tui()->send(tb);
}

void coTUIEarthMap::resend(bool create)
{
	coTUIElement::resend(create);

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_FLOAT;
	tb << ID;
	tb << latitude;
	tb << longitude;
	tb << altitude;
	tui()->send(tb);
	setMinMax(minHeight, maxHeight);
	updatePath();
}

//----------------------------------------------------------
//----------------------------------------------------------
//##########################################################

coTUIPopUp::coTUIPopUp(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_POPUP)
{
	text = "";
	immediate = false;
}


coTUIPopUp::~coTUIPopUp()
{
}

void coTUIPopUp::setImmediate(bool i)
{
	immediate = i;
	setVal(immediate);
}

void coTUIPopUp::parseMessage(TokenBuffer& tb)
{
	char* m;
	tb >> m;
	text = m;
	if (listener)
		listener->tabletEvent(this);
}

void coTUIPopUp::setText(const std::string& t)
{
	text = t;
	setVal(text);
}

void coTUIPopUp::resend(bool create)
{
	coTUIElement::resend(create);
	setVal(text);
	setVal(immediate);
}

//----------------------------------------------------------
//----------------------------------------------------------

coTabletUI* coTabletUI::tUI = NULL;

coTabletUI* coTabletUI::instance()
{
	if (tUI == NULL)
		tUI = new coTabletUI();
	return tUI;
}
//----------------------------------------------------------
//----------------------------------------------------------

coTUIElement::coTUIElement(coTabletUI* tabletUI, const std::string& n, int pID, int ty)
{
	type = ty;
	m_tui = tabletUI;
	xs = -1;
	ys = -1;
	xp = 0;
	yp = 0;
	parentID = pID;
	name = n;
	label = n;
	ID = tui()->getID();
	listener = NULL;
	tui()->addElement(this);
	createSimple(type);
	/*if (tui()->debugTUI())
	{
		coVRMSController::instance()->syncStringStop(name);
		coVRMSController::instance()->syncInt(ID);
	}*/
}

coTUIElement::coTUIElement(const std::string& n, int pID, int ty)
{
	type = ty;
	m_tui = NULL;
	xs = -1;
	ys = -1;
	xp = 0;
	yp = 0;
	parentID = pID;
	name = n;
	label = n;
	ID = tui()->getID();
	listener = NULL;
	tui()->addElement(this);
	createSimple(type);
	/*if (tui()->debugTUI())
	{
		coVRMSController::instance()->syncStringStop(name);
		coVRMSController::instance()->syncInt(ID);
	}*/
}

coTUIElement::~coTUIElement()
{
	TokenBuffer tb;
	tb << TABLET_REMOVE;
	tb << ID;
	tui()->send(tb);
	tui()->removeElement(this);
}

void coTUIElement::createSimple(int type)
{
	TokenBuffer tb;
	tb << TABLET_CREATE;
	tb << ID;
	tb << type;
	tb << parentID;
	tb << name.c_str();
	tui()->send(tb);
}

coTabletUI* coTUIElement::tui() const
{
	if (m_tui)
		return m_tui;
	else
		return coTabletUI::instance();
}

void coTUIElement::setLabel(const char* l)
{
	if (l)
		setLabel(std::string(l));
	else
		setLabel(std::string(""));
}

void coTUIElement::setLabel(const std::string& l)
{
	label = l;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_LABEL;
	tb << ID;
	tb << label.c_str();
	tui()->send(tb);
}

int coTUIElement::getID() const
{
	return ID;
}

void coTUIElement::setEventListener(coTUIListener* l)
{
	listener = l;
}

void coTUIElement::parseMessage(TokenBuffer&)
{
}

coTUIListener* coTUIElement::getMenuListener()
{
	return listener;
}

void coTUIElement::setVal(float value)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_FLOAT;
	tb << ID;
	tb << value;
	tui()->send(tb);
}

void coTUIElement::setVal(bool value)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_BOOL;
	tb << ID;
	tb << (char)value;
	tui()->send(tb);
}

void coTUIElement::setVal(int value)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_INT;
	tb << ID;
	tb << value;
	tui()->send(tb);
}

void coTUIElement::setVal(const std::string& value)
{
	if (!tui()->isConnected())
		return;

	//cerr << "coTUIElement::setVal info: " << (value ? value : "*NULL*") << endl;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_STRING;
	tb << ID;
	tb << value.c_str();
	tui()->send(tb);
}

void coTUIElement::setVal(int type, int value)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << type;
	tb << ID;
	tb << value;
	tui()->send(tb);
}

void coTUIElement::setVal(int type, float value)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << type;
	tb << ID;
	tb << value;
	tui()->send(tb);
}
void coTUIElement::setVal(int type, int value, const std::string& nodePath)
{
	if (!tui()->isConnected())
		return;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << type;
	tb << ID;
	tb << value;
	tb << nodePath.c_str();
	tui()->send(tb);
}
void coTUIElement::setVal(int type, const std::string& nodePath, const std::string& simPath, const std::string& simName)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << type;
	tb << ID;
	tb << nodePath.c_str();
	tb << simPath.c_str();
	tb << simName.c_str();
	tui()->send(tb);
}

void coTUIElement::setVal(int type, int value, const std::string& nodePath, const std::string& parentPath)
{
	if (!tui()->isConnected())
		return;

	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << type;
	tb << ID;
	tb << value;
	tb << nodePath.c_str();
	tb << parentPath.c_str();
	tui()->send(tb);
}
void coTUIElement::resend(bool create)
{
	if (create)
		createSimple(type);

	TokenBuffer tb;

	if (xs > 0)
	{
		tb.reset();
		tb << TABLET_SET_VALUE;
		tb << TABLET_SIZE;
		tb << ID;
		tb << xs;
		tb << ys;
		tui()->send(tb);
	}

	tb.reset();
	tb << TABLET_SET_VALUE;
	tb << TABLET_POS;
	tb << ID;
	tb << xp;
	tb << yp;
	tui()->send(tb);

	tb.reset();
	tb << TABLET_SET_VALUE;
	tb << TABLET_LABEL;
	tb << ID;
	tb << label.c_str();
	tui()->send(tb);

	if (!enabled)
	{
		tb.reset();
		tb << TABLET_SET_VALUE;
		tb << TABLET_SET_ENABLED;
		tb << ID;
		tb << enabled;
		tui()->send(tb);
	}

	if (hidden)
	{
		tb.reset();
		tb << TABLET_SET_VALUE;
		tb << TABLET_SET_HIDDEN;
		tb << ID;
		tb << hidden;
		tui()->send(tb);
	}
}

void coTUIElement::setPos(int x, int y)
{
	xp = x;
	yp = y;
	if ((x > 10000 || x < -10000) || (y > 10000 || y < -10000))
	{
		fprintf(stderr, "coordinates out of range!, x=%d, y=%d\n", x, y);
#ifdef _WIN32
		DebugBreak();
#else
		abort();
#endif
	}
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_POS;
	tb << ID;
	tb << xp;
	tb << yp;
	tui()->send(tb);
}

void coTUIElement::setHidden(bool newState)
{
	//std::cerr << "coTUIElement::setHidden(hide=" << hidden << " -> " << newState << "): ID=" << ID << std::endl;
	if (hidden == newState)
		return;

	hidden = newState;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SET_HIDDEN;
	tb << ID;
	tb << hidden;
	tui()->send(tb);
}

void coTUIElement::setEnabled(bool newState)
{
	if (enabled == newState)
		return;

	enabled = newState;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SET_ENABLED;
	tb << ID;
	tb << enabled;
	tui()->send(tb);
}
/*
void coTUIElement::setColor(Qt::GlobalColor color)
{

	this->color = color;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_COLOR;
	tb << ID;
	tb << this->color;
	tui()->send(tb);
}*/

void coTUIElement::setSize(int x, int y)
{
	xs = x;
	ys = y;
	TokenBuffer tb;
	tb << TABLET_SET_VALUE;
	tb << TABLET_SIZE;
	tb << ID;
	tb << xs;
	tb << ys;
	tui()->send(tb);
}

coTabletUI::coTabletUI()
{
	//assert(!tUI);
	tUI = this;

	init();

	std::string line;
	if (getenv("COVER_TABLETUI"))
	{
		std::string env(getenv("COVER_TABLETUI"));
		std::string::size_type p = env.find(':');
		if (p != std::string::npos)
		{
			port = atoi(env.substr(p + 1).c_str());
			env = env.substr(0, p);
		}
		line = env;
		std::cerr << "getting TabletPC configuration from $COVER_TABLETUI: " << line << ":" << port << std::endl;
		serverMode = false;
	}
	else
	{
		//port = coCoviseConfig::getInt("port", "COVER.TabletUI", port);
		line = "127.0.0.1"; //coCoviseConfig::getEntry("host", "COVER.TabletUI");
		serverMode = false; //coCoviseConfig::isOn("COVER.TabletUI.ServerMode", false);
	}

	if (!line.empty())
	{
		if (stricmp(line.c_str(), "NONE") != 0)
		{
			serverHost = new Host(line.c_str());
			localHost = new Host("localhost");
		}
	}
	else
	{
		localHost = new Host("localhost");
	}

	tryConnect();
}

coTabletUI::coTabletUI(const std::string& host, int port)
	: port(port)
{
	init();

	serverMode = false;
	serverHost = new Host(host.c_str());

	tryConnect();
}

void coTabletUI::init()
{
	//debugTUIState = coCoviseConfig::isOn("COVER.DebugTUI", debugTUIState);

	mainFolder = new coTUITabFolder("MainFolder");
	mainFolder->setPos(0, 0);

	//timeout = coCoviseConfig::getFloat("COVER.TabletPC.Timeout", timeout);
}

// resend all ui Elements to the TabletPC
void coTabletUI::resendAll()
{
	for (auto el : elements)
	{
		el->resend(true);
	}
}

bool coTabletUI::isConnected() const
{
	return connectedHost != nullptr;
}

void coTabletUI::close()
{
	connectedHost = NULL;
	delete conn;
	conn = NULL;

	delete sgConn;
	sgConn = NULL;

	delete serverConn;
	serverConn = NULL;

	tryConnect();
}

bool coTabletUI::debugTUI()
{
	return debugTUIState;
}

void coTabletUI::tryConnect()
{
	delete serverConn;
	serverConn = nullptr;
	delete conn;
	conn = nullptr;
	connectedHost = nullptr;
}

coTabletUI::~coTabletUI()
{
	if (tUI == this)
		tUI = nullptr;

	delete serverConn;
	delete conn;
	delete serverHost;
	delete localHost;
}

int coTabletUI::getID()
{
	return ID++;
}

void coTabletUI::send(TokenBuffer& tb)
{
	if (!connectedHost)
	{
		return;
	}
	assert(conn);
	Message m(tb);
	m.type = COVISE_MESSAGE_TABLET_UI;
	conn->send_msg(&m);
}

VOID CALLBACK coTabletUI::timerCallback(HWND hwnd,
	UINT uMsg,
	UINT_PTR idEvent,
	DWORD dwTime)
{
		coTabletUI::instance()->update();
}


bool coTabletUI::update()
{
	//if (coVRMSController::instance() == NULL)
	//	return false;

	if (connectedHost)
	{
	}
	else if ( serverMode)
	{
		if (serverConn == NULL)
		{
			serverConn = new ServerConnection(port, 0, (sender_type)0);
			serverConn->listen();
		}
	}
	else if ( (serverHost != NULL || localHost != NULL))
	{
		if (time(NULL) - oldTime > 2.)
		{
			oldTime = time(NULL);

			{
				//Message msg(Message::UI, "WANT_TABLETUI");
				//coVRPluginList::instance()->sendVisMessage(&msg);
			}

			lock();
			if (!connFuture.valid())
			{
				connectedHost = NULL;
				connFuture = std::async(std::launch::async, [this]() -> Host*
					{
						ClientConnection* nconn = nullptr;
						Host* host = nullptr;
						if (serverHost)
						{
							if (firstConnection)
								std::cerr << "Trying tablet UI connection to " << serverHost->get_name() << ":" << port << "... " << std::flush;
							nconn = new ClientConnection(serverHost, port, 0, (sender_type)0, 0);
							if (firstConnection)
								std::cerr << (nconn->is_connected() ? "success" : "failed") << "." << std::endl;
							firstConnection = false;
						}
						if (nconn && nconn->is_connected())
						{
							conn = nconn;
							host = serverHost;
						}
						else if (nconn) // could not open server port
						{
							delete nconn;
							nconn = NULL;
						}

						if (!conn && localHost)
						{
							if (firstConnection)
								std::cerr << "Trying tablet UI connection to " << localHost->get_name() << ":" << port << "... " << std::flush;
							nconn = new ClientConnection(localHost, port, 0, (sender_type)0, 0);
							if (firstConnection)
								std::cerr << (nconn->is_connected() ? "success" : "failed") << "." << std::endl;
							firstConnection = false;

							if (nconn->is_connected())
							{
								conn = nconn;
								host = localHost;
							}
							else
							{
								// could not open server port
								delete nconn;
								nconn = NULL;

							}
						}


						if (!conn || !host)
						{
							return nullptr;
						}

						// create Texture and SGBrowser Connections
						Message* msg = new Message();
						conn->recv_msg(msg);
						if (msg->type == COVISE_MESSAGE_SOCKET_CLOSED)
						{
							delete msg;

							delete conn;
							conn = NULL;

							delete sgConn;
							sgConn = NULL;

							return nullptr;
						}

						{
							TokenBuffer stb(msg);
							int sgPort = 0;
							stb >> sgPort;
							delete msg;

							ClientConnection* cconn = new ClientConnection(host, sgPort, 0, (sender_type)0);
							//ClientConnection* cconn = new ClientConnection(host, sgPort, 0, (sender_type)0, 2, 1);
							if (!cconn->is_connected()) // could not open server port
							{
#ifndef _WIN32
								if (errno != ECONNREFUSED)
								{
									fprintf(stderr, "Could not connect to TabletPC SGBrowser %s; port %d: %s\n",
										host->getName(), sgPort, strerror(errno));
								}
#else
								//fprintf(stderr, "Could not connect to TabletPC %s; port %d\n", connectedHost->get_name(), sgPort);
#endif
								delete conn;
								conn = NULL;

								delete cconn;
								cconn = NULL;

								return nullptr;
							}
							sgConn = cconn;
						}

						return host;
					});
			}
			unlock();
		}

		if (connFuture.valid())
		{
			lock();
			auto status = connFuture.wait_for(std::chrono::seconds(0));
			if (status == std::future_status::ready)
			{
				connectedHost = connFuture.get();

				if (!connectedHost)
				{
					delete conn;
					conn = NULL;

					delete sgConn;
					sgConn = NULL;
				}
				else
				{
					assert(conn);
					assert(sgConn);

					// resend all ui Elements to the TabletPC
					resendAll();
				}
			}
			unlock();
		}
	}

	if (serverConn && serverConn->check_for_input())
	{
		if (conn)
		{
			connectedHost = nullptr;
			delete conn;
			conn = NULL;

			delete sgConn;
			sgConn = NULL;
		}
		conn = serverConn->spawn_connection();
		if (conn && conn->is_connected())
		{
			Message m;
			conn->recv_msg(&m);
			TokenBuffer tb(&m);
			char* hostName;
			tb >> hostName;
			serverHost = new Host(hostName);

			resendAll();
		}
	}

	for (auto el : newElements)
	{
		el->resend(false);
	}
	newElements.clear();

	bool changed = false;
	bool gotMessage = false;
	do
	{
		gotMessage = false;
		Message m;
			if (conn)
			{
				if (conn->check_for_input())
				{
					conn->recv_msg(&m);
					gotMessage = true;
				}
			}
		//gotMessage = coVRMSController::instance()->syncBool(gotMessage);
		if (gotMessage)
		{
			

			changed = true;

			TokenBuffer tb(&m);
			switch (m.type)
			{
			case COVISE_MESSAGE_SOCKET_CLOSED:
			case COVISE_MESSAGE_CLOSE_SOCKET:
			{
				connectedHost = nullptr;
				delete conn;
				conn = NULL;

				delete sgConn;
				sgConn = NULL;
			}
			break;
			case COVISE_MESSAGE_TABLET_UI:
			{

				int ID;
				tb >> ID;
				if (ID >= 0)
				{
					for (auto el : elements)
					{
						if (el && el->getID() == ID)
						{
							el->parseMessage(tb);
							break;
						}
					}
				}
			}
			break;
			default:
			{
				cerr << "unknown Message type" << endl;
			}
			break;
			}
		}
	} while (gotMessage);

	return changed;
}

void coTabletUI::addElement(coTUIElement* e)
{
	elements.push_back(e);
	newElements.push_back(e);
}

void coTabletUI::removeElement(coTUIElement* e)
{
	{
		auto it = std::find(newElements.begin(), newElements.end(), e);
		if (it != newElements.end())
			newElements.erase(it);
	}
	{
		auto it = std::find(elements.begin(), elements.end(), e);
		if (it != elements.end())
			elements.erase(it);
	}
}


coTUIGroupBox::coTUIGroupBox(const std::string& n, int pID)
	: coTUIElement(n, pID, TABLET_GROUPBOX)
{

}

coTUIGroupBox::coTUIGroupBox(coTabletUI* tui, const std::string& n, int pID)
	: coTUIElement(tui, n, pID, TABLET_GROUPBOX)
{

}

coTUIGroupBox::~coTUIGroupBox()
{

}

void coTUIGroupBox::parseMessage(TokenBuffer& tb)
{
	int i;
	tb >> i;
	if (i == TABLET_ACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletPressEvent(this);
		}
	}
	else if (i == TABLET_DISACTIVATED)
	{
		if (listener)
		{
			listener->tabletEvent(this);
			listener->tabletReleaseEvent(this);
		}
	}
	else
	{
		cerr << "unknown event " << i << endl;
	}
}
