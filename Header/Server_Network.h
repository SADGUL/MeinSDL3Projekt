#ifndef SERVER_NETWORK_H
#define SERVER_NETWORK_H

#include <winsock2.h>
#include <string>

#include "Brett.h"
#include "Figuren.h"
#include "Bauer.h"
#include "Springer.h"
#include "Laeufer.h"
#include "Turm.h"
#include "Dame.h"
#include "Koenig.h"
#include "main_quanten.h"

using namespace std;

SOCKET InitializeServer();
void CloseServer();
bool sendToClient(const string& message);
string receiveFromClient();
Figuren* CreateFigureServer(char name, bool farbe, int s, int z, float p, bool gezogen, bool geschlagen, const string& path);
string sendBoardServer(const Brett& Spielfeld);
void clearBoardServer(Brett& Spielfeld);
void getBoardServer(const string& message, Brett& Spielfeld);

#endif
