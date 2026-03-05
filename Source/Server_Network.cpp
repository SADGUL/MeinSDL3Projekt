#include "../Header/Server_Network.h"
#include <iostream>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <fstream>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")

#define FILENAME "Protokoll_Server.txt"
#define SERVER_PORT 26000
#define DEFAULT_BUFLEN 512

using namespace std;

static SOCKET ClientSocket = INVALID_SOCKET;
static SOCKET ListenSocket = INVALID_SOCKET;
static ofstream Server_Protokoll;

SOCKET InitializeServer()
{
    // 1) Protokoll öffnen
    Server_Protokoll.open(FILENAME);
    if (!Server_Protokoll.is_open()) {
        cout << "Protokoll konnte nicht geöffnet werden\n";
        return INVALID_SOCKET;
    }

    cout << "Protokoll wurde geöffnet\n";
    Server_Protokoll << "Protokoll Server:\n";

    // 2) Winsock starten
    WSADATA wsaData;
    int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (err != 0) {
        Server_Protokoll << "WSAStartup failed with error: " << err << "\n";
        return INVALID_SOCKET;
    }
    Server_Protokoll << "WSAStartup successful!\n";

    // 3) Socket erstellen
    ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ListenSocket == INVALID_SOCKET) {
        Server_Protokoll << "socket failed with error: " << WSAGetLastError() << "\n";
        WSACleanup();
        return INVALID_SOCKET;
    }
    Server_Protokoll << "Socket created successfully!\n";

    // 4) Bind
    sockaddr_in serverService{};
    serverService.sin_family = AF_INET;
    serverService.sin_addr.s_addr = htonl(INADDR_ANY);
    serverService.sin_port = htons(SERVER_PORT);

    if (bind(ListenSocket, (sockaddr*)&serverService, sizeof(serverService)) == SOCKET_ERROR) {
        Server_Protokoll << "bind failed with error: " << WSAGetLastError() << "\n";
        closesocket(ListenSocket);
        WSACleanup();
        return INVALID_SOCKET;
    }
    Server_Protokoll << "bind -done-\n";

    // 5) Listen
    if (listen(ListenSocket, SOMAXCONN) == SOCKET_ERROR) {
        Server_Protokoll << "listen failed with error: " << WSAGetLastError() << "\n";
        closesocket(ListenSocket);
        WSACleanup();
        return INVALID_SOCKET;
    }
    Server_Protokoll << "listening -done-\n";

    // 6) Accept
    ClientSocket = accept(ListenSocket, nullptr, nullptr);
    if (ClientSocket == INVALID_SOCKET) {
        Server_Protokoll << "accept failed with error: " << WSAGetLastError() << "\n";
        closesocket(ListenSocket);
        WSACleanup();
        return INVALID_SOCKET;
    }
    Server_Protokoll << "accept -done-\n";

    return ClientSocket;
}

void CloseServer()
{
    if (ClientSocket != INVALID_SOCKET)
    {
        closesocket(ClientSocket);
    }

    if (ListenSocket != INVALID_SOCKET)
    {
        closesocket(ListenSocket);
    }
    Server_Protokoll << "Sockets closed successfully\n";
    cout << "Sockets closed successfully\n";

    WSACleanup();
}

string receiveFromClient()
{
    char buffer[DEFAULT_BUFLEN];
    int result = recv(ClientSocket, buffer, DEFAULT_BUFLEN - 1, 0);

    if (result <= 0) {
        Server_Protokoll << "receiveFromClient failed\n";
        return "";
    }

    buffer[result] = '\0';
    Server_Protokoll << "Received from client: " << buffer << "\n";
    return string(buffer);
}

bool sendToClient(const string& message)
{
    int iResult = send(ClientSocket, message.c_str(), (int)message.size(), 0);

    if (iResult == SOCKET_ERROR) {
        Server_Protokoll << "send failed with error: " << WSAGetLastError() << "\n";
        return false;
    }

    Server_Protokoll << "Message sent successfully: " << message << "\n";
    return true;
}


Figuren* CreateFigureServer(char name, bool farbe, int s, int z, float p, bool gezogen, bool geschlagen, const std::string& path)
{
    Figuren* newFigure = nullptr;

    switch (name) { //neue figur erstellen
    case 'b': newFigure = new Bauer(); break;
    case 'S': newFigure = new Springer(); break;
    case 'L': newFigure = new Laeufer(); break;
    case 'T': newFigure = new Turm(); break;
    case 'D': newFigure = new Dame(); break;
    case 'K': newFigure = new Koenig(); break;
    default: return nullptr;
    }

    newFigure->Set_Name(name);
    newFigure->Set_Farbe(farbe);
    newFigure->Set_Spalte(s - 1);
    newFigure->Set_Zeile(z - 1);
    newFigure->Set_Wahrscheinlichkeit(p);
    newFigure->Set_Gezogen(gezogen);
    newFigure->Set_Geschlagen(geschlagen);
    newFigure->Set_Dateipfad(path);

    return newFigure;
}

string sendBoardServer(const Brett& Spielfeld)
{
    ostringstream outputstring;

    outputstring << "BOARD;\n";

    // Brettstatus
    outputstring << Spielfeld.whites_turn << ","
        << Spielfeld.schachmatt << ","
        << Spielfeld.en_passant << ","
        << Spielfeld.en_passant_spalte << ","
        << Spielfeld.en_passant_zeile << ";\n";


    for (int zeile = 1; zeile <= 8; zeile++) { //jede spielfeldzeile durchgehen
        for (int spalte = 1; spalte <= 8; spalte++) { //jede Spielfeldspalte durchgehen

            Figuren* figur = Spielfeld.Felder[spalte - 1][zeile - 1];
            if (!figur) continue;

            outputstring << figur->Get_Name() << ","
                << figur->Get_Farbe() << ","
                << spalte << ","
                << zeile << ","
                << figur->Get_Wahrscheinlichkeit() << ","
                << figur->Get_Gezogen() << ","
                << figur->Get_Geschlagen() << ","
                << figur->Get_Dateipfad()
                << ";\n";
        }
    }

    outputstring << "END;\n";
    return outputstring.str();
}

void clearBoardServer(Brett& Spielfeld) //vor dem übertragen das alte feld löschen
{
    for (int zeile = 0; zeile < 8; zeile++)
        for (int spalte = 0; spalte < 8; spalte++)
            Spielfeld.Felder[spalte][zeile] = nullptr;
}

void getBoardServer(const string& message, Brett& Spielfeld)
{
    clearBoardServer(Spielfeld);

    istringstream inputstring(message); //wie bei datei zeile für zeile lesen
    string line;

    getline(inputstring, line); // "BOARD" anfang von der message

    // Brettstatus
    getline(inputstring, line); //liest zweite zeile
    {
        stringstream newstring(line);
        char comma;

        newstring >> Spielfeld.whites_turn >> comma
            >> Spielfeld.schachmatt >> comma
            >> Spielfeld.en_passant >> comma
            >> Spielfeld.en_passant_spalte >> comma
            >> Spielfeld.en_passant_zeile;
    }

    // Figuren
    while (getline(inputstring, line)) {
        if (line == "END;" || line.empty()) //liest so lange bis end kommt oder nichts mehr
            break;

        stringstream newstring(line);

        char name;
        int farbe;
        int spalte;
        int zeile;
        int gezogen;
        int geschlagen;
        float p;
        string path;
        char comma;

        newstring >> name >> comma
            >> farbe >> comma
            >> spalte >> comma
            >> zeile >> comma
            >> p >> comma
            >> gezogen >> comma
            >> geschlagen >> comma;

        getline(newstring, path, ';');

        // Sicherheitscheck gegen Out-of-Bounds
        if (spalte < 1 || spalte > 8 || zeile < 1 || zeile > 8)
            continue;

        Figuren* figures = CreateFigureServer(name, farbe, spalte, zeile, p, gezogen, geschlagen, path);
        Spielfeld.Felder[spalte - 1][zeile - 1] = figures; //setzten der neuen figuren
    }
}
