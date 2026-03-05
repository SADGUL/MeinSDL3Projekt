#include <winsock2.h>
#include <ws2tcpip.h>
#include "../Header/Client_Network.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

// Brett + Figuren


#pragma comment(lib, "Ws2_32.lib") //Windows Socket Bibliothek

#define FILENAME "Protokoll_Client.txt"
#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 26000
#define DEFAULT_BUFLEN 4096   

using namespace std;

static SOCKET ConnectSocketClient = INVALID_SOCKET;
static ofstream Client_Protokoll;

SOCKET InitializeClient()
{
    Client_Protokoll.open(FILENAME);
    if (!Client_Protokoll.is_open()) {
        cout << "Protokoll konnte nicht geöffnet werden\n";
        return INVALID_SOCKET;
    }

    cout << "Protokoll wurde geöffnet\n";
    Client_Protokoll << "Protokoll Client:\n";

    WSADATA wsaData;
    int err = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if (err != 0) {
        Client_Protokoll << "WSAStartup failed with error: " << err << "\n";
        cout << "WSAStartup failed with error: " << err << "\n";
        return INVALID_SOCKET;
    }

    ConnectSocketClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ConnectSocketClient == INVALID_SOCKET) {
        Client_Protokoll << "socket failed with error: " << WSAGetLastError() << "\n";
        cout << "socket failed with error: " << WSAGetLastError() << "\n";
        WSACleanup();
        return INVALID_SOCKET;
    }

    sockaddr_in clientService{};
    clientService.sin_family = AF_INET;
    clientService.sin_port = htons(SERVER_PORT);

    if (inet_pton(AF_INET, SERVER_IP, &clientService.sin_addr) <= 0) {
        Client_Protokoll << "Invalid address or address not supported\n";
        cout << "Invalid address or address not supported\n";
        closesocket(ConnectSocketClient);
        WSACleanup();
        return INVALID_SOCKET;
    }

    int iResult = connect(ConnectSocketClient, (SOCKADDR*)&clientService, sizeof(clientService));
    if (iResult == SOCKET_ERROR) {
        Client_Protokoll << "connect failed with error: " << WSAGetLastError() << "\n";
        cout << "connect failed with error: " << WSAGetLastError() << "\n";
        closesocket(ConnectSocketClient);
        WSACleanup();
        return INVALID_SOCKET;
    }

    Client_Protokoll << "Connected to server successfully!\n";
    Client_Protokoll << "\nSpielprotokoll\n";

    return ConnectSocketClient;
}

void CloseClient() {

    int iResult = closesocket(ConnectSocketClient);
    if (iResult == SOCKET_ERROR) {
        Client_Protokoll << "closesocket failed with error: " << WSAGetLastError() << "\n";
        cout << "closesocket failed with error: " << WSAGetLastError() << "\n";
    }
    else {
        Client_Protokoll << "Socket closed successfully\n";
        cout << "Socket closed successfully\n";
    }

    WSACleanup();
}

string receiveFromServer()
{
    char buffer[DEFAULT_BUFLEN];
    int result = recv(ConnectSocketClient, buffer, DEFAULT_BUFLEN - 1, 0);
    if (result <= 0)
    {
        Client_Protokoll << "receiveFromServer failed\n";
        return "";
    }

    buffer[result] = '\0';
    Client_Protokoll << "Received from server: " << buffer << "\n";
    return string(buffer);
}

bool sendtoServer(const string& message)
{
    int iResult = send(ConnectSocketClient, message.c_str(), (int)message.size(), 0);
    if (iResult == SOCKET_ERROR) {
        Client_Protokoll << "send failed with error: " << WSAGetLastError() << "\n";
        return false;
    }

    Client_Protokoll << "Message sent successfully: " << message << "\n";
    return true;
}

Figuren* CreateFigureClient(char name, bool farbe, int s, int z, float p, bool gezogen, bool geschlagen, const string& path)
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

string sendBoardClient(const Brett& Spielfeld)
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

void clearBoardClient(Brett& Spielfeld) //vor dem übertragen das alte feld löschen
{
    for (int zeile = 0; zeile < 8; zeile++)
        for (int spalte = 0; spalte < 8; spalte++)
            Spielfeld.Felder[spalte][zeile] = nullptr;
}

void getBoardClient(const string& message, Brett& Spielfeld)
{
    clearBoardClient(Spielfeld);

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

        Figuren* figures = CreateFigureClient(name, farbe, spalte, zeile, p, gezogen, geschlagen, path);
        Spielfeld.Felder[spalte - 1][zeile - 1] = figures; //setzten der neuen figuren
    }
}
