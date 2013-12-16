#ifndef DIALOG_BOX_H
#define DIALOG_BOX_H

/*
 *  dialog.h
 *  PALBenchmark
 *
 *  Created by Brock Woolf on 16/01/10.
 *  Copyright 2010 Blastcube. All rights reserved.
 *
 */

#ifdef __APPLE__
	#include <Carbon/Carbon.h>
#elif defined _WIN32
	#include <windows.h>
#endif

#include <cstdio>

class GUIFeedback
{
public:
	GUIFeedback() { ; }
	
	void DisplayErrorDialogue(const char *cmsg)
	{
	#ifdef __APPLE__
                printf("ERROR: %s\n", cmsg);
          // It would be nice to show a dialog box, but I can't figure out how from C++ for Cocoa.
          /*
                CFStringRef msg = CFStringCreateWithCString(NULL, cmsg, kCFStringEncodingASCII);
		
		StandardAlert(kAlertStopAlert, "Error", (ConstStr255Param)msg, NULL, NULL);
          */
	#elif defined _WIN32
		MessageBox(NULL, cmsg, "Error", MB_ICONERROR | MB_OK);
	#else
		printf("ERROR: %s\n", cmsg);
	#endif
	}
};
 
#endif
