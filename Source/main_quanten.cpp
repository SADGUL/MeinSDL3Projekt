#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "../Header/Brett.h"
#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include "../Header/Spiel_Logik.h"
#include <string>
#include <vector>

using namespace std;

struct AppState {
    const char* title;
    SDL_Window* window;
    SDL_Renderer* renderer;
    bool redraw = true;
};

/**
 * Initializes an SDL3 Window
 */
SDL_AppResult AppInit(void** appstate, Window_Configuration WindowConfigs) {
    if (!(*appstate = SDL_malloc(sizeof(AppState)))) {
        return SDL_APP_FAILURE;
    }
    auto* state = static_cast<AppState*>(*appstate);
    state->title = "Quantum Chess";

    if (!SDL_Init(SDL_INIT_VIDEO))
        return SDL_APP_FAILURE;

    if (!(state->window = SDL_CreateWindow(state->title, WindowConfigs.WindowWidth, WindowConfigs.WindowHeight, SDL_WINDOW_RESIZABLE)))
        return SDL_APP_FAILURE;
    SDL_SetWindowMinimumSize(state->window, WindowConfigs.minWidth, WindowConfigs.minHeight);

    if (!(state->renderer = SDL_CreateRenderer(state->window, NULL)))
        return SDL_APP_FAILURE;

    std::cout << "Program loaded!" << std::endl;
    return SDL_APP_CONTINUE;
}

/**
 * Logic that needs to be done every frame
 */
SDL_AppResult AppIterate(void* appstate, Window_Configuration* WindowConfigs, Layout* Game_Layout, MoveButton* Button_Texture, Brett& Spielfeld, vector <Moegliches_Feld> Vector_Moegliche_felder) {
    auto* state = static_cast<AppState*>(appstate);

    if (state->redraw) {
        state->redraw = false;

        SDL_SetRenderDrawColor(state->renderer, 0, 0, 0, 255);
        SDL_RenderClear(state->renderer);

        SDL_GetWindowSize(state->window, &WindowConfigs->WindowWidth, &WindowConfigs->WindowHeight);

        Game_Layout->sidebarWidth = WindowConfigs->WindowWidth * Game_Layout->sidebarRatio;
        Game_Layout->WidthMinusSidebar = static_cast<float>(WindowConfigs->WindowWidth) - Game_Layout->sidebarWidth;
        Game_Layout->boardSize = min(static_cast<float>(WindowConfigs->WindowHeight), Game_Layout->WidthMinusSidebar);

        if (Game_Layout->WidthMinusSidebar < static_cast<float>(WindowConfigs->WindowHeight)) {
            WindowConfigs->WindowHeight = WindowConfigs->WindowHeight - (WindowConfigs->WindowHeight - Game_Layout->boardSize);
        }
        if ((Game_Layout->boardSize + Game_Layout->sidebarWidth) < WindowConfigs->WindowWidth) {
            WindowConfigs->WindowWidth = Game_Layout->boardSize + Game_Layout->sidebarWidth;
        }
        SDL_SetWindowSize(state->window, WindowConfigs->WindowWidth, WindowConfigs->WindowHeight);

        drawChessboard(state->window, state->renderer, Game_Layout->boardSize);
        RenderTextures(state->window, state->renderer, Spielfeld, Vector_Moegliche_felder, Game_Layout, WindowConfigs, Button_Texture);

        SDL_RenderPresent(state->renderer);
    }

    return SDL_APP_CONTINUE;
}

int drawChessboard(SDL_Window* window, SDL_Renderer* renderer, float width) {
    constexpr int boardFields = 8;
    const float squareSize = (width * 1.0f) / (boardFields * 1.0f);

    SDL_RenderClear(renderer);

    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if ((row + col) % 2 == 0) {
                SDL_SetRenderDrawColor(renderer, 240, 217, 181, 255);
            }
            else {
                SDL_SetRenderDrawColor(renderer, 181, 136, 99, 255);
            }
            SDL_FRect rect = { col * squareSize, row * squareSize, squareSize, squareSize };
            SDL_RenderFillRect(renderer, &rect);
        }
    }
    return 0;
}

void RenderTextures(SDL_Window* window, SDL_Renderer* renderer, Brett& Spielfeld, vector <Moegliches_Feld> Vector_Moegliche_felder, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button_Texture) {
    constexpr int boardFields = 8;
    const float squareSize = (Game_Layout->boardSize * 1.0f) / (boardFields * 1.0f);
    int row;
    int col;
    float probability = 1.0f;
    float height = 7;
    float width = squareSize;

    //Figuren:
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (Spielfeld.Felder[i][j] == NULL) continue;
            else {
                row = Spielfeld.Felder[i][j]->Get_Zeile();
                col = Spielfeld.Felder[i][j]->Get_Spalte();
                row = 8 - row;
                col = col - 1;
                SDL_FRect rect = { col * squareSize, row * squareSize, squareSize, squareSize - 5 };
                SDL_RenderTexture(renderer, Spielfeld.Felder[i][j]->Get_Texture(), NULL, &rect);

                probability = Spielfeld.Felder[i][j]->Get_Wahrscheinlichkeit();
                width = squareSize * probability;
                SDL_FRect probability_bar = { col * squareSize, (row * squareSize) + (squareSize - height + 2) , width, height };
                SDL_SetRenderDrawColor(renderer, 0, 0, 255.0f, 0);
                SDL_RenderFillRect(renderer, &probability_bar);
            }
        }
    }

    //moegliche Felder
    for (int i = 0; i < Vector_Moegliche_felder.size(); i++) {
        col = Vector_Moegliche_felder[i].spalte - 1;
        row = 8 - Vector_Moegliche_felder[i].zeile;
        SDL_FRect rect = { col * squareSize, row * squareSize, squareSize, squareSize };
        GreenCircle(renderer, rect);
    }

    //sidebar button aktiviert:
    if (Button_Texture->normal_move) {
        SDL_SetTextureColorMod(Button_Texture->normal, 0, 255, 0);
    }
    else SDL_SetTextureColorMod(Button_Texture->normal, 200, 200, 255);
    if (Button_Texture->split_move) {
        SDL_SetTextureColorMod(Button_Texture->split, 0, 255, 0);
    }
    else SDL_SetTextureColorMod(Button_Texture->split, 200, 200, 255);
    if (Button_Texture->merge_move) {
        SDL_SetTextureColorMod(Button_Texture->merge, 0, 255, 0);
    }
    else SDL_SetTextureColorMod(Button_Texture->merge, 200, 200, 255);

    //sidebar:
    float height_button = WindowConfigs->WindowHeight / 3.0f;
    SDL_FRect button_normal = { Game_Layout->boardSize , 0, Game_Layout->sidebarWidth , height_button };
    SDL_RenderTexture(renderer, Button_Texture->normal, NULL, &button_normal);

    SDL_FRect button_split = { Game_Layout->boardSize , height_button, Game_Layout->sidebarWidth , height_button };
    SDL_RenderTexture(renderer, Button_Texture->split, NULL, &button_split);

    SDL_FRect button_merge = { Game_Layout->boardSize , 2 * height_button, Game_Layout->sidebarWidth , height_button };
    SDL_RenderTexture(renderer, Button_Texture->merge, NULL, &button_merge);
}

void GreenCircle(SDL_Renderer* renderer, SDL_FRect rect) {
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    float width_rect = rect.w;
    float height_rect = rect.h;
    float y_rect = rect.y;
    float x_rect = rect.x;

    float center_x = x_rect + (width_rect / 2.0f);
    float center_y = y_rect + (height_rect / 2.0f);

    float radius = width_rect / 4.0f;

    const int segments = 32;
    SDL_Vertex vertices[segments + 1];
    int PointsOfCircle[segments * 3];

    SDL_FColor Color = { 57.0f / 255.0f, 1.0f, 20.0f / 255.0f, 0.5f };

    vertices[0] = { {center_x, center_y},Color,{0.0f, 0.0f} };

    for (int i = 0; i < segments; i++) {
        float angle = (float)i / segments * 2.0f * SDL_PI_F;
        vertices[i + 1] = { {center_x + radius * SDL_sinf(angle),center_y + radius * SDL_cosf(angle)},Color, {0.0f, 0.0f} };
    }

    for (int i = 0; i < segments; i++) {
        PointsOfCircle[i * 3] = 0;
        PointsOfCircle[i * 3 + 1] = i + 1;
        PointsOfCircle[i * 3 + 2] = (i + 1) % segments + 1;
    }

    SDL_RenderGeometry(renderer, nullptr, vertices, segments + 1, PointsOfCircle, segments * 3);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
}

void setPossibleFields(Brett& Spielfeld) {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (Spielfeld.Felder[i][j] == NULL) continue;
            Spielfeld.Felder[i][j]->Set_Moegliche_Felder(Spielfeld);
        }
    }
}

void createTexture(void* appstate, Brett& Spielfeld) {
    const auto* state = static_cast<AppState*>(appstate);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (Spielfeld.Felder[i][j] == NULL) continue;
            else {
                string path = Spielfeld.Felder[i][j]->Get_Dateipfad();
                const char* c_path = path.c_str();

                SDL_Texture* texture = CreateTexture_from_Image(state->renderer, c_path);

                if (!texture) {
                    cout << "Fehler beim Laden von :" << Spielfeld.Felder[i][j]->Get_Name()
                        << " Position: " << i << " " << j << endl;
                }
                else {
                    Spielfeld.Felder[i][j]->Set_Texture(texture);
                }
            }
        }
    }
}

void createButtonTextures(void* appstate, MoveButton* Button_Texture) {
    const auto* state = static_cast<AppState*>(appstate);
    Button_Texture->normal = CreateTexture_from_Image(state->renderer, "Png/arrow.png");
    Button_Texture->split = CreateTexture_from_Image(state->renderer, "Png/split.png");
    Button_Texture->merge = CreateTexture_from_Image(state->renderer, "Png/merge.png");
}

SDL_Texture* CreateTexture_from_Image(SDL_Renderer* renderer, const char* c_path) {
    int width, height, original_channels;

    unsigned char* pixels = stbi_load(c_path, &width, &height, &original_channels, 4);

    if (!pixels) {
        std::cout << "Fehler beim Laden von " << c_path << ": " << stbi_failure_reason() << std::endl;
        return nullptr;
    }

    SDL_Surface* surface = SDL_CreateSurfaceFrom(width, height, SDL_PIXELFORMAT_RGBA32, pixels, width * 4);

    if (!surface) {
        std::cout << "Fehler bei SDL_Surface Erstellung: " << SDL_GetError() << std::endl;
        stbi_image_free(pixels);
        return nullptr;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

    SDL_DestroySurface(surface);
    stbi_image_free(pixels);

    return texture;
}

void calculateFieldFromCoordinates(void* appstate, const float& mouseX, const float& mouseY, int* selectedRow, int* selectedCol, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button) {
    const auto* state = static_cast<AppState*>(appstate);

    constexpr int boardFields = 8;
    float squareSize = Game_Layout->boardSize / static_cast<float>(boardFields);

    if (mouseX <= Game_Layout->boardSize) {    // mausklick im Spielfeld
        *selectedRow = 8 - (static_cast<int>(mouseY / squareSize));
        *selectedCol = (static_cast<int>(mouseX / squareSize)) + 1;

        // ==========================================
        // --- SAFETY CLAMPS ---
        // Verhindert, dass mathematische Rundungsfehler das Array crashen
        if (*selectedRow < 1) *selectedRow = 1;
        if (*selectedRow > 8) *selectedRow = 8;
        if (*selectedCol < 1) *selectedCol = 1;
        if (*selectedCol > 8) *selectedCol = 8;
        // ==========================================
    }
    else {
        // Klick in der Sidebar -> Radio-Button Logik!
        if ((mouseY <= WindowConfigs->WindowHeight / 3.0f)) {
            Button->normal_move = true;
            Button->split_move = false;
            Button->merge_move = false;
            cout << "Modus: Normal Move" << endl;
        }
        else if (mouseY < WindowConfigs->WindowHeight / 3.0f * 2) {
            Button->normal_move = false;
            Button->split_move = true;
            Button->merge_move = false;
            cout << "Modus: Split Move" << endl;
        }
        else {
            Button->normal_move = false;
            Button->split_move = false;
            Button->merge_move = true;
            cout << "Modus: Merge Move" << endl;
        }
    }
}

SDL_AppResult AppEvent(void* appstate, SDL_Event* event, float* mouseX, float* mouseY, int* selectedRow, int* selectedCol, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button) {
    auto* state = static_cast<AppState*>(appstate);

    switch (event->type) {
    case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED: {
        state->redraw = true;
        break;
    }
    case SDL_EVENT_QUIT:
        return SDL_APP_SUCCESS;
    case SDL_EVENT_MOUSE_BUTTON_DOWN: {
        if (event->button.button == SDL_BUTTON_LEFT) {
            *mouseX = event->button.x;
            *mouseY = event->button.y;
            calculateFieldFromCoordinates(appstate, *mouseX, *mouseY, selectedRow, selectedCol, Game_Layout, WindowConfigs, Button);
            state->redraw = true;
        }
    }
    default:
        break;
    }

    return SDL_APP_CONTINUE;
}

void AppQuit(void* appstate, SDL_AppResult result) {
    const auto* state = static_cast<AppState*>(appstate);
    SDL_DestroyRenderer(state->renderer);
    SDL_DestroyWindow(state->window);
    if (result == SDL_APP_FAILURE)
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error",
            SDL_GetError(), appstate ? state->window : NULL);
    SDL_free(appstate);
    SDL_Quit();
}











// ==========================================
// HILFSFUNKTIONEN FÜR DAS SETUP
// ==========================================
void Setup_Test_Board(Brett& Spielfeld, vector<Bauer>& bauern, vector<Springer>& springer, vector<Laeufer>& laeufer, vector<Turm>& tuerme, vector<Dame>& damen, vector<Koenig>& koenige) {
    bauern.clear(); springer.clear(); laeufer.clear();
    tuerme.clear(); damen.clear(); koenige.clear();

    bauern.reserve(100); springer.reserve(100); laeufer.reserve(100);
    tuerme.reserve(100); damen.reserve(100); koenige.reserve(100);

    Spielfeld_Reset(Spielfeld);
    Startaufstellung_Bauern(bauern, Spielfeld);
    Startaufstellung_Springer(springer, Spielfeld);
    Startaufstellung_Laeufer(laeufer, Spielfeld);
    Startaufstellung_Tuerme(tuerme, Spielfeld);
    Startaufstellung_Damen(damen, Spielfeld);
    Startaufstellung_Koenige(koenige, Spielfeld);
    Spielfeld.whites_turn = true;
}

void PrintResult(bool condition, string errorMessage) {
    if (condition) {
        cout << "[ERFOLG]" << endl << endl;
    }
    else {
        cout << "[FEHLER] " << errorMessage << endl << endl;
    }
}

// ==========================================
// KATEGORIE 1: KLASSISCHES SCHACH
// ==========================================

void Test_1_1_NormalerZug() {
    cout << "Test 1.1: Standard-Bewegung (Bauer e2 nach e4)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    Logik_normal(5, 2, Spielfeld, b, s, l, t, d, k); // e2 auswählen
    Logik_normal(5, 4, Spielfeld, b, s, l, t, d, k); // e4 Ziel

    PrintResult(Spielfeld.Felder[4][3] != nullptr && Spielfeld.Felder[4][1] == nullptr, "Bauer hat sich nicht korrekt bewegt.");
}

void Test_1_2_Blockade() {
    cout << "Test 1.2: Blockaden (Turm a1 durch Bauer a2 ziehen)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    Logik_normal(1, 1, Spielfeld, b, s, l, t, d, k); // Turm a1 auswählen
    Logik_normal(1, 3, Spielfeld, b, s, l, t, d, k); // Versuch nach a3

    PrintResult(Spielfeld.Felder[0][2] == nullptr && Spielfeld.Felder[0][0] != nullptr, "Turm ist illegal durch den Bauern gezogen.");
}

void Test_1_3_Schlagen() {
    cout << "Test 1.3: Normales Schlagen (e4 -> d5)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Weiß: e2 -> e4
    Logik_normal(5, 2, Spielfeld, b, s, l, t, d, k); Logik_normal(5, 4, Spielfeld, b, s, l, t, d, k);
    Spielfeld.whites_turn = false; // Manueller Wechsel (falls deine Logik das nicht tut)

    // Schwarz: d7 -> d5
    Logik_normal(4, 7, Spielfeld, b, s, l, t, d, k); Logik_normal(4, 5, Spielfeld, b, s, l, t, d, k);
    Spielfeld.whites_turn = true;

    // Weiß: e4 -> d5 (Schlagen)
    Logik_normal(5, 4, Spielfeld, b, s, l, t, d, k); Logik_normal(4, 5, Spielfeld, b, s, l, t, d, k);

    PrintResult(Spielfeld.Felder[3][4] != nullptr && Spielfeld.Felder[3][4]->Get_Farbe() == true, "Weißer Bauer hat schwarzen Bauern nicht korrekt geschlagen.");
}


// ==========================================
// KATEGORIE 2: SPEZIELLE SCHACHREGELN
// ==========================================

void Test_2_1_EnPassant() {
    cout << "Test 2.1: En Passant... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Setup: Weiß Bauer e2->e5, Schwarz Bauer d7->d5 (Doppelschritt daneben)
    Logik_normal(5, 2, Spielfeld, b, s, l, t, d, k); Logik_normal(5, 4, Spielfeld, b, s, l, t, d, k);
    Spielfeld.whites_turn = true;
    Logik_normal(5, 4, Spielfeld, b, s, l, t, d, k); Logik_normal(5, 5, Spielfeld, b, s, l, t, d, k);
    Spielfeld.whites_turn = false;
    Logik_normal(4, 7, Spielfeld, b, s, l, t, d, k); Logik_normal(4, 5, Spielfeld, b, s, l, t, d, k);

    Spielfeld.whites_turn = true;
    // En Passant Schlag (e5 -> d6)
    Logik_normal(5, 5, Spielfeld, b, s, l, t, d, k); Logik_normal(4, 6, Spielfeld, b, s, l, t, d, k);

    // Bauer d5 muss weg sein, weißer Bauer auf d6
    PrintResult(Spielfeld.Felder[3][4] == nullptr && Spielfeld.Felder[3][5] != nullptr, "En Passant hat nicht funktioniert (oder Gegner nicht gelöscht).");
}

// ==========================================
// KATEGORIE 3: QUANTEN-MECHANIKEN
// ==========================================

void Test_3_1_SplitMove() {
    cout << "Test 3.1: Quanten Split Move... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // 1. Klick: Springer auf g1 (Spalte 7, Zeile 1) auswählen
    Logik_Split(7, 1, Spielfeld, b, s, l, t, d, k);

    // 2. Klick: Erstes Zielfeld f3 (Spalte 6, Zeile 3)
    Logik_Split(6, 3, Spielfeld, b, s, l, t, d, k);

    // 3. Klick: Zweites Zielfeld h3 (Spalte 8, Zeile 3) -> Löst den Zug aus!
    Logik_Split(8, 3, Spielfeld, b, s, l, t, d, k);

    // Prüfen, ob die Figuren nun auf den beiden ZIELFELDERN liegen
    Figuren* feld_1 = Spielfeld.Felder[5][2]; // f3 (Spalte 6, Zeile 3)
    Figuren* feld_2 = Spielfeld.Felder[7][2]; // h3 (Spalte 8, Zeile 3)

    if (feld_1 != nullptr && feld_2 != nullptr) {
        PrintResult(feld_1->Get_Wahrscheinlichkeit() == 0.5f && feld_2->Get_Wahrscheinlichkeit() == 0.5f, "Wahrscheinlichkeiten nicht 50/50 geteilt.");
    }
    else {
        cout << "[FEHLER] Gesplittete Figur fehlt auf f3 oder h3!" << endl;
    }
}

void Test_3_3_MergeMove() {
    cout << "Test 3.3: Quanten Merge Move... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // 1. Erst Splitten (3 Klicks)
    Logik_Split(7, 1, Spielfeld, b, s, l, t, d, k); // Springer auf g1 wählen
    Logik_Split(6, 3, Spielfeld, b, s, l, t, d, k); // Split-Ziel 1: f3
    Logik_Split(8, 3, Spielfeld, b, s, l, t, d, k); // Split-Ziel 2: h3

    // Wieder Weiß an den Zug lassen, da der Split den Zug beendet hat
    Spielfeld.whites_turn = true;

    // 2. Jetzt Mergen (ebenfalls 3 Klicks!)
    Logik_Merge(6, 3, Spielfeld); // Klick 1: Teilfigur auf f3 anwählen
    Logik_Merge(8, 3, Spielfeld); // Klick 2: Teilfigur auf h3 anwählen
    Logik_Merge(7, 5, Spielfeld); // Klick 3: ZIELFELD g5 anwählen -> Löst den Merge aus!

    // 3. Ergebnis prüfen
    Figuren* feld_ziel = Spielfeld.Felder[6][4]; // Array-Index für g5 (Spalte 7, Zeile 5)

    if (feld_ziel != nullptr) {
        PrintResult(feld_ziel->Get_Wahrscheinlichkeit() == 1.0f, "Figur steht auf g5, hat aber keine 100% (1.0) Wahrscheinlichkeit.");
    }
    else {
        cout << "[FEHLER] Keine Figur auf dem Ziel-Feld g5 angekommen!" << endl;
    }
}
// ==========================================
// KATEGORIE 4: FORTGESCHRITTENE REGELN & QUANTEN-PHYSIK
// ==========================================

void Test_4_1_Bauernumwandlung() {
    cout << "Test 4.1: Bauernumwandlung (Promotion)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Setup: Wir machen den Weg für den weißen a-Bauern komplett frei
    Spielfeld.Felder[0][6] = nullptr; // Schwarzer Bauer auf a7 wird entfernt
    Spielfeld.Felder[0][7] = nullptr; // NEU: Schwarzer Turm auf a8 wird entfernt!

    // Weißer Bauer marschiert Schritt für Schritt durch
    Logik_normal(1, 2, Spielfeld, b, s, l, t, d, k); Logik_normal(1, 4, Spielfeld, b, s, l, t, d, k); // a2 -> a4
    Spielfeld.whites_turn = true;
    Logik_normal(1, 4, Spielfeld, b, s, l, t, d, k); Logik_normal(1, 5, Spielfeld, b, s, l, t, d, k); // a4 -> a5
    Spielfeld.whites_turn = true;
    Logik_normal(1, 5, Spielfeld, b, s, l, t, d, k); Logik_normal(1, 6, Spielfeld, b, s, l, t, d, k); // a5 -> a6
    Spielfeld.whites_turn = true;
    Logik_normal(1, 6, Spielfeld, b, s, l, t, d, k); Logik_normal(1, 7, Spielfeld, b, s, l, t, d, k); // a6 -> a7
    Spielfeld.whites_turn = true;
    Logik_normal(1, 7, Spielfeld, b, s, l, t, d, k); Logik_normal(1, 8, Spielfeld, b, s, l, t, d, k); // a7 -> a8

    // Die Umwandlungsfunktion aufrufen
    Check_For_Promotion(Spielfeld, d);

    Figuren* feld_a8 = Spielfeld.Felder[0][7];
    PrintResult(feld_a8 != nullptr && feld_a8->Get_Name() == 'D', "Bauer wurde auf a8 nicht in eine Dame ('D') umgewandelt.");
}

void Test_4_2_Rochade() {
    cout << "Test 4.2: Kurze Rochade (Castling)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Setup: Läufer (f1) und Springer (g1) entfernen, damit der Weg zwischen König (e1) und Turm (h1) frei ist
    Spielfeld.Felder[5][0] = nullptr;
    Spielfeld.Felder[6][0] = nullptr;

    // König von e1(5,1) nach g1(7,1) bewegen
    Logik_normal(5, 1, Spielfeld, b, s, l, t, d, k);
    Logik_normal(7, 1, Spielfeld, b, s, l, t, d, k);

    Figuren* koenig = Spielfeld.Felder[6][0]; // g1
    Figuren* turm = Spielfeld.Felder[5][0];   // f1

    PrintResult(koenig != nullptr && koenig->Get_Name() == 'K' && turm != nullptr && turm->Get_Name() == 'T', "König oder Turm stehen nach der Rochade auf dem falschen Feld.");
}

void Test_4_3_Quanten_Kollaps() {
    cout << "Test 4.3: Quanten-Schlagen & Kollaps (Schrödingers Katze)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Setup: Schwarzer Springer von b8(2,8) splittet auf a6(1,6) und c6(3,6)
    Spielfeld.whites_turn = false;
    Logik_Split(2, 8, Spielfeld, b, s, l, t, d, k);
    Logik_Split(1, 6, Spielfeld, b, s, l, t, d, k);
    Logik_Split(3, 6, Spielfeld, b, s, l, t, d, k);

    // Den Weg für den weißen a1-Turm freimachen (weißen a2-Bauern löschen)
    Spielfeld.Felder[0][1] = nullptr;

    // Weißer Turm von a1(1,1) greift die 50%-Figur auf a6(1,6) an
    Spielfeld.whites_turn = true;
    Logik_normal(1, 1, Spielfeld, b, s, l, t, d, k);
    Logik_normal(1, 6, Spielfeld, b, s, l, t, d, k);

    // Auswertung: Da Zufall im Spiel ist, wissen wir nicht, welches Feld gewinnt.
    // Aber wir wissen: EINE der beiden schwarzen Springer-Positionen muss nun 100% (oder tot) sein.
    Figuren* feld_c6 = Spielfeld.Felder[2][5]; // Der alternative Springer auf c6
    Figuren* feld_a6 = Spielfeld.Felder[0][5]; // Das Angriffsfeld

    bool kollaps_korrekt = false;
    if (feld_a6 != nullptr && feld_a6->Get_Name() == 'T') {
        // Weißer Turm steht auf a6. Was ist mit dem schwarzen Springer auf c6 passiert?
        if (feld_c6 == nullptr) {
            // Fall 1: Springer war auf a6 (wurde geschlagen), c6 ist verpufft.
            kollaps_korrekt = true;
        }
        else if (feld_c6 != nullptr && feld_c6->Get_Wahrscheinlichkeit() == 1.0f) {
            // Fall 2: Springer war NICHT auf a6 (Turm griff ins Leere), c6 wurde dafür 100% real.
            kollaps_korrekt = true;
        }
    }

    PrintResult(kollaps_korrekt, "Der Quantenkollaps hat kein logisch valides Ergebnis (0% oder 100%) erzeugt.");
}

void Test_4_4_Geister_Blockade() {
    cout << "Test 4.4: Wegfindung durch Quanten-Figuren (Geister-Blockade)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // Setup: Wir setzen gezielt Figuren, um einen komplexen Weg zu testen
    Spielfeld_Reset(Spielfeld); // Komplett leeres Brett

    // Weißer Turm auf a1
    Turm wT; wT.Set_Name('T'); wT.Set_Farbe(true); wT.Set_Wahrscheinlichkeit(1.0f);
    Spielfeld.Felder[0][0] = &wT; wT.Set_Spalte(1); wT.Set_Zeile(1);

    // Schwarzer Bauer auf a3 (50%)
    Bauer sB; sB.Set_Name('b'); sB.Set_Farbe(false); sB.Set_Wahrscheinlichkeit(0.5f); sB.Add_Same_Piece_S(2); sB.Add_Same_Piece_Z(3); // (Fake-Verlinkung zu b3)
    Spielfeld.Felder[0][2] = &sB; sB.Set_Spalte(1); sB.Set_Zeile(3);

    // Weißer Turm berechnet seine möglichen Felder
    wT.Set_Moegliche_Felder(Spielfeld);
    vector<Moegliches_Feld> felder = wT.Get_Moegliche_Felder();

    // Suche das Feld a4 (direkt HINTER der 50%-Blockade)
    bool a4_gefunden = false;
    float a4_wahrscheinlichkeit = 0.0f;
    for (int i = 0; i < felder.size(); i++) {
        if (felder[i].spalte == 1 && felder[i].zeile == 4) {
            a4_gefunden = true;
            a4_wahrscheinlichkeit = felder[i].wahrscheinlichkeit;
            break;
        }
    }

    // Wenn der Turm auf a4 zieht, sollte er dort nur noch mit 50% Wahrscheinlichkeit ankommen (da er zu 50% auf a3 geblockt wird)
    PrintResult(a4_gefunden && a4_wahrscheinlichkeit == 0.5f, "Der Turm durfte entweder nicht durch den 50%-Geist ziehen, oder seine Wahrscheinlichkeit dahinter wurde nicht auf 50% reduziert.");
}
void Test_4_5_Kollaps_Nullpointer_Crash() {
    cout << "Test 4.5: Kollaps Nullpointer Crash (Geloeschte Geister)... ";
    Brett Spielfeld; vector<Bauer> b; vector<Springer> s; vector<Laeufer> l; vector<Turm> t; vector<Dame> d; vector<Koenig> k;
    Setup_Test_Board(Spielfeld, b, s, l, t, d, k);

    // 1. Weißen Springer (g1) splitten nach f3 und h3
    Logik_Split(7, 1, Spielfeld, b, s, l, t, d, k); // Springer wählen
    Logik_Split(6, 3, Spielfeld, b, s, l, t, d, k); // Ziel 1: f3
    Logik_Split(8, 3, Spielfeld, b, s, l, t, d, k); // Ziel 2: h3

    Figuren* springer_f3 = Spielfeld.Felder[5][2];
    Figuren* springer_h3 = Spielfeld.Felder[7][2];

    // 2. Wir simulieren den Fehlerzustand: 
    // Der Springer auf h3 wurde durch irgendetwas (z.B. ein vorheriges Schlagen) vom Brett gelöscht.
    // Aber die `same_piece` Liste vom Springer auf f3 weiß davon nichts!
    springer_h3->Set_Geschlagen(true);
    Spielfeld.Felder[7][2] = nullptr;

    // 3. Wir zwingen die Messung() auf f3, fehlzuschlagen (0% Wahrscheinlichkeit).
    // Dadurch MUSS das Spiel in der same_piece Liste nach dem Zwilling auf h3 suchen und Kollaps() aufrufen.
    springer_f3->Set_Wahrscheinlichkeit(0.0f);

    // 4. Der Moment der Wahrheit: Aufruf der Messung
    // Ohne den Bugfix würde das Spiel in exakt dieser Zeile komplett abstürzen (schließen)!
    Messung(6, 3, Spielfeld);

    // 5. Überprüfung
    // Wenn das Programm diese Zeile erreicht, ohne abzustürzen, hat dein Sicherheits-Check funktioniert!
    PrintResult(true, "Das Spiel ist abgestuerzt!");
}

// ==========================================
// TEST RUNNER HAUPTFUNKTION
// ==========================================
void RunAllTests() {
    cout << "========================================" << endl;
    cout << "   STARTE QUANTUM CHESS TEST SUITE      " << endl;
    cout << "========================================" << endl;

    Test_1_1_NormalerZug();
    Test_1_2_Blockade();
    Test_1_3_Schlagen();

    Test_2_1_EnPassant();

    Test_3_1_SplitMove();
    Test_3_3_MergeMove();

    Test_4_1_Bauernumwandlung();
    Test_4_2_Rochade();
    Test_4_3_Quanten_Kollaps();
    Test_4_4_Geister_Blockade();
	Test_4_5_Kollaps_Nullpointer_Crash();

    cout << "========================================" << endl;
    cout << "             TESTS BEENDET              " << endl;
    cout << "========================================" << endl;
}















int main() {
    // init SDL
    RunAllTests();


    void* appstate = nullptr;
    Window_Configuration WindowConfigs;

    SDL_AppResult result = AppInit(&appstate, WindowConfigs);

    if (result == SDL_APP_CONTINUE) {

        Brett Spielfeld;

        vector <Bauer> bauern;
        vector <Springer> springer;
        vector <Laeufer> laeufer;
        vector <Turm> tuerme;
        vector <Dame> damen;
        vector <Koenig> koenige;

        // RESERVE MEMORY TO PREVENT VECTOR REALLOCATION (Verschränkung fix)
        bauern.reserve(100);
        springer.reserve(100);
        laeufer.reserve(100);
        tuerme.reserve(100);
        damen.reserve(100);
        koenige.reserve(100);

        Spielfeld_Reset(Spielfeld);
        Startaufstellung_Bauern(bauern, Spielfeld);
        Startaufstellung_Springer(springer, Spielfeld);
        Startaufstellung_Laeufer(laeufer, Spielfeld);
        Startaufstellung_Tuerme(tuerme, Spielfeld);
        Startaufstellung_Damen(damen, Spielfeld);
        Startaufstellung_Koenige(koenige, Spielfeld);

        Layout Game_Layout;
        MoveButton Button_Texture;

        createTexture(appstate, Spielfeld);
        createButtonTextures(appstate, &Button_Texture);

        SDL_Event event;

        float mouseX = 0;
        float mouseY = 0;
        int selectedRow = 0;
        int selectedCol = 0;

        // NEW EVENT FLAG
        bool newClick = false;

        vector <Moegliches_Feld> Vector_Moegliche_felder;
        bool piece_moved = false;

        while (true) {
            // handle events
            while (SDL_PollEvent(&event)) {

                // DETECT ACTUAL CLICK
                if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
                    newClick = true;
                }

                result = AppEvent(appstate, &event, &mouseX, &mouseY, &selectedRow, &selectedCol, &Game_Layout, &WindowConfigs, &Button_Texture);
                switch (result) {
                case SDL_APP_SUCCESS:
                    AppQuit(appstate, result);
                    return 0;
                case SDL_APP_FAILURE:
                    AppQuit(appstate, result);
                    return 1;
                case SDL_APP_CONTINUE:
                default:
                    break;
                }
            }

            // ONLY EXECUTE IF A CLICK OCCURRED
            // ONLY EXECUTE IF A CLICK OCCURRED
            if (newClick && !Spielfeld.schachmatt) {

                // 1. FIX: Züge nur ausführen, wenn die Maus auf dem Schachbrett war! (Verhindert Sidebar-Crash)
                if (mouseX <= Game_Layout.boardSize) {

                    if (Button_Texture.normal_move && !Button_Texture.split_move && !Button_Texture.merge_move) {
                        Logik_normal(selectedCol, selectedRow, Spielfeld, bauern, springer, laeufer, tuerme, damen, koenige);
                    }
                    else if (!Button_Texture.normal_move && Button_Texture.split_move && !Button_Texture.merge_move) {
                        Logik_Split(selectedCol, selectedRow, Spielfeld, bauern, springer, laeufer, tuerme, damen, koenige);
                    }
                    else if (!Button_Texture.normal_move && !Button_Texture.split_move && Button_Texture.merge_move) {
                        Logik_Merge(selectedCol, selectedRow, Spielfeld);
                    }
                    else {
                        No_Move(Spielfeld);
                    }

                    if (Check_For_Promotion(Spielfeld, damen)) {
                        createTexture(appstate, Spielfeld);
                    }
                    Check_For_Kollaps_Verschraenkung(Spielfeld);

                    // CHECK FOR MATE IMMEDIATELY AFTER COLLAPSE
                    Check_for_Mate(Spielfeld);

                    cout << "Ausgewaehlte Reihe: " << selectedRow << endl;
                    cout << "Ausgewaehlte Spalte: " << selectedCol << endl;

                } // <-- ENDE der if-Abfrage für das Schachbrett

                // ==========================================
                // WINNER POPUP ANNOUNCEMENT
                // ==========================================
                if (Spielfeld.schachmatt) {
                    const char* winnerText = Spielfeld.whites_turn ? "Black wins by Checkmate!" : "White wins by Checkmate!";
                    auto* state = static_cast<AppState*>(appstate);
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Game Over!", winnerText, state->window);
                    break;
                }
                // ==========================================

                // 2. FIX: Visuelles Feedback (inkl. Schnittmenge für den Merge-Move)
                if (Spielfeld.piece_selected) {
                    Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(Spielfeld);
                    Vector_Moegliche_felder = Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();

                    if (Button_Texture.merge_move && Spielfeld.first_piece_selected && Spielfeld.second_piece_selected) {

                        // Felder der zuerst angeklickten Figur laden
                        Spielfeld.Felder[Spielfeld.first_piece_s - 1][Spielfeld.first_piece_z - 1]->Set_Moegliche_Felder(Spielfeld);
                        vector<Moegliches_Feld> mf1 = Spielfeld.Felder[Spielfeld.first_piece_s - 1][Spielfeld.first_piece_z - 1]->Get_Moegliche_Felder();
                        vector<Moegliches_Feld> schnittmenge;

                        // Vergleichen und nur ueberlappende 100%-Felder uebernehmen
                        for (int i = 0; i < Vector_Moegliche_felder.size(); i++) {
                            for (int j = 0; j < mf1.size(); j++) {
                                if (Vector_Moegliche_felder[i].spalte == mf1[j].spalte &&
                                    Vector_Moegliche_felder[i].zeile == mf1[j].zeile &&
                                    Vector_Moegliche_felder[i].wahrscheinlichkeit == 1.0f &&
                                    mf1[j].wahrscheinlichkeit == 1.0f) {

                                    schnittmenge.push_back(Vector_Moegliche_felder[i]);
                                    break;
                                }
                            }
                        }
                        Vector_Moegliche_felder = schnittmenge;
                    }
                }
                else {
                    Vector_Moegliche_felder.clear();
                }

                // CONSUME CLICK
                newClick = false;
            }

            if (Spielfeld.piece_selected) {
                AppIterate(appstate, &WindowConfigs, &Game_Layout, &Button_Texture, Spielfeld, Vector_Moegliche_felder);
            }
            else {
                AppIterate(appstate, &WindowConfigs, &Game_Layout, &Button_Texture, Spielfeld, {});
            }
        }
    }

    AppQuit(appstate, result);
    return 0;
}