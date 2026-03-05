#ifndef CLIENT_NETWORK_H
#define CLIENT_NETWORK_H

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

SOCKET InitializeClient();
void CloseClient();
bool sendtoServer(const string& message);
string receiveFromServer();
Figuren* CreateFigureClient(char name, bool farbe, int s, int z, float p, bool gezogen, bool geschlagen, const string& path);
string sendBoardClient(const Brett& Spielfeld);
void clearBoardClient(Brett& Spielfeld);
void getBoardClient(const string& message, Brett& Spielfeld);
#endif
