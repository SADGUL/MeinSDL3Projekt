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
 * @param appstate AppState struct to store information of the apps state in
 * @return SDL_APP_FAILURE to terminate with an error, SDL_APP_SUCCESS to
 *         terminate with success, SDL_APP_CONTINUE to continue.
 */
SDL_AppResult AppInit(void** appstate, Window_Configuration WindowConfigs) {
    // allocate memory for AppState struct
    if (!(*appstate = SDL_malloc(sizeof(AppState)))) {     //SDL_malloc gibt Zeiger auf reservierten Speicherbreich zurück
        return SDL_APP_FAILURE;                        //Wenn SDL_malloc nullpointer zurückgibt
    }
    auto* state = static_cast<AppState*>(*appstate);   //lokaler Zeiger , static_cast, da malloc void* zurückgibt
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
 * @param appstate AppState struct to store information of the apps state in
 * @return SDL_APP_FAILURE to terminate with an error, SDL_APP_SUCCESS to
 *         terminate with success, SDL_APP_CONTINUE to continue.
 */
SDL_AppResult AppIterate(void* appstate, Window_Configuration* WindowConfigs, Layout* Game_Layout, MoveButton* Button_Texture, Brett& Spielfeld, vector <Moegliches_Feld> Vector_Moegliche_felder) {
    auto* state = static_cast<AppState*>(appstate);

    if (state->redraw) {
        state->redraw = false;

        // TODO: redraw Frame
        // clear frame buffer
        SDL_SetRenderDrawColor(state->renderer, 0, 0, 0, 255);
        SDL_RenderClear(state->renderer);

        SDL_GetWindowSize(state->window, &WindowConfigs->WindowWidth, &WindowConfigs->WindowHeight);   //Fenstergröße holen

        Game_Layout->sidebarWidth = WindowConfigs->WindowWidth * Game_Layout->sidebarRatio;         //Breite Sidebar berechnen
        Game_Layout->WidthMinusSidebar = static_cast<float>(WindowConfigs->WindowWidth) - Game_Layout->sidebarWidth;
        Game_Layout->boardSize = min(static_cast<float>(WindowConfigs->WindowHeight), Game_Layout->WidthMinusSidebar);  //Größe des Spielfelds berechnen, ja nachdem, was kleiner ist

        //damit unter dem Spieldfeld kein schwarzer Bereich enstehen kann:
        if (Game_Layout->WidthMinusSidebar < static_cast<float>(WindowConfigs->WindowHeight)) {
            WindowConfigs->WindowHeight = WindowConfigs->WindowHeight - (WindowConfigs->WindowHeight - Game_Layout->boardSize);
        }
        if ((Game_Layout->boardSize + Game_Layout->sidebarWidth) < WindowConfigs->WindowWidth) {
            WindowConfigs->WindowWidth = Game_Layout->boardSize + Game_Layout->sidebarWidth;
        }
        SDL_SetWindowSize(state->window, WindowConfigs->WindowWidth, WindowConfigs->WindowHeight);

        //float maxBoardFromWidth = WindowConfigs->WindowWidth / (1.0f + WindowConfigs->sidebarRatio);
        //float boardSize = min(static_cast<float>(WindowConfigs->WindowHeight),maxBoardFromWidth);
        //float sidebarWidth = boardSize * WindowConfigs->sidebarRatio;

        // draw chessboard
        drawChessboard(state->window, state->renderer, Game_Layout->boardSize);
        // Render textures/ figures
        RenderTextures(state->window, state->renderer, Spielfeld, Vector_Moegliche_felder, Game_Layout, WindowConfigs, Button_Texture);

        // swap frame buffer
        SDL_RenderPresent(state->renderer);

        //determine the possible fields to which a piece could be moved
      //   setPossibleFields(Spielfeld);
    }

    return SDL_APP_CONTINUE;
}

// TODO: add javadoc
int drawChessboard(SDL_Window* window, SDL_Renderer* renderer, float width) {
    constexpr int boardFields = 8;

    // get size of squares
    const float squareSize = (width * 1.0f) / (boardFields * 1.0f);    //*1.f -> float/float und nicht mehr int/int

    // render Background
    SDL_RenderClear(renderer);

    // render squares
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {

            // Alternate colors
            if ((row + col) % 2 == 0) {
                // SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
                SDL_SetRenderDrawColor(renderer, 240, 217, 181, 255);      //Farbe linchess
            }
            else {
                //SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
                SDL_SetRenderDrawColor(renderer, 181, 136, 99, 255);       //Farbe linchess
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
                SDL_FRect rect = { col * squareSize, row * squareSize, squareSize, squareSize - 5 }; //Figur im Feld nach oben verschoben
                SDL_RenderTexture(renderer, Spielfeld.Felder[i][j]->Get_Texture(), NULL, &rect);

                //Wahrscheinlichkeitsanzeige
                probability = Spielfeld.Felder[i][j]->Get_Wahrscheinlichkeit();
                width = squareSize * probability;  //Länge des Balken ist abhängig von der Wahrscheinlichkeit
                SDL_FRect probability_bar = { col * squareSize, (row * squareSize) + (squareSize - height + 2) , width, height }; //Balken unterhalb der Figur
                SDL_SetRenderDrawColor(renderer, 0, 0, 255.0f, 0); //blau
                SDL_RenderFillRect(renderer, &probability_bar);
            }
        }
    }

    //mögliche Felder
    for (int i = 0; i < Vector_Moegliche_felder.size(); i++) {
        col = Vector_Moegliche_felder[i].spalte - 1;
        row = 8 - Vector_Moegliche_felder[i].zeile;
        //cout <<"Spalte: "<<  col << "  Zeile:" << row << endl;    Nur zum Testen
        SDL_FRect rect = { col * squareSize, row * squareSize, squareSize, squareSize };
        GreenCircle(renderer, rect);
        //SDL_SetRenderDrawColor(renderer, 57, 255, 20, 255);
        //SDL_RenderFillRect(renderer, &rect);
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

    float center_x = x_rect + (width_rect / 2.0f);   //Float Division
    float center_y = y_rect + (height_rect / 2.0f);

    float radius = width_rect / 4.0f;

    const int segments = 32;    //Kreis bzw. Polygon mit 32 Segmenten
    SDL_Vertex vertices[segments + 1];   // Randpunkte + Mittelpunkt
    int PointsOfCircle[segments * 3];  // Alle Punkte die den Kreis definieren. Immer 3 Punkte pro dreieck -> 32 *3

    SDL_FColor Color = { 57.0f / 255.0f, 1.0f, 20.0f / 255.0f, 0.5f };

    //Mittelpunkt:
    vertices[0] = { {center_x, center_y},Color,{0.0f, 0.0f} };

    // Kreisumfang
    for (int i = 0; i < segments; i++) {
        float angle = (float)i / segments * 2.0f * SDL_PI_F;   // i durch 32 mal 2pi -> gleichmäßige Verteilung
        //Mit diesen Winkel nun die Randpunkte (Einheitskreis)
        vertices[i + 1] = { {center_x + radius * SDL_sinf(angle),center_y + radius * SDL_cosf(angle)},Color, {0.0f, 0.0f} };
    }

    //Alle Punkte:
    //Immer nacheinander drei Punkte gehören zu einem Dreieck
    //Ich muss in PointOfCirle nur sagen welcher Platz im Array verticles dem Punkt des Dreiecks entspricht
    // Der Rest macht dann SDL_RenderGeometry

    for (int i = 0; i < segments; i++) {
        PointsOfCircle[i * 3] = 0;           //Mittelpunkt (immer jeder 3. Punkt)
        PointsOfCircle[i * 3 + 1] = i + 1;    // Erster Randpunkt: vertices[i + 1]
        PointsOfCircle[i * 3 + 2] = (i + 1) % segments + 1;  // Zweiter Randpunkt: Modulo wichtig, damit letzter punkt wieder 0 + 1,also erste Randpunkt wieder
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

                // Use our new stb_image helper function instead of SDL_LoadPNG!
                SDL_Texture* texture = CreateTexture_from_Image(state->renderer, c_path);

                if (!texture) {
                    cout << "Fehler beim Laden von :" << Spielfeld.Felder[i][j]->Get_Name()
                        << " Position: " << i << " " << j << endl;
                }
                else {
                    // Only set the texture if it successfully loaded
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

    // 1. Load the image data using stbi_load. 
    // We pass '4' at the end to force it to load Red, Green, Blue, and Alpha (transparency).
    unsigned char* pixels = stbi_load(c_path, &width, &height, &original_channels, 4);

    if (!pixels) {
        std::cout << "Fehler beim Laden von " << c_path << ": " << stbi_failure_reason() << std::endl;
        return nullptr;
    }

    // 2. Create an SDL_Surface from those raw STB pixels
    // Pitch (the last number) is width * 4 because there are 4 color channels per pixel.
    SDL_Surface* surface = SDL_CreateSurfaceFrom(width, height, SDL_PIXELFORMAT_RGBA32, pixels, width * 4);

    if (!surface) {
        std::cout << "Fehler bei SDL_Surface Erstellung: " << SDL_GetError() << std::endl;
        stbi_image_free(pixels); // Prevent memory leaks!
        return nullptr;
    }

    // 3. Convert that surface into an SDL_Texture
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

    // 4. Clean up the temporary surface and the raw pixel data
    SDL_DestroySurface(surface);
    stbi_image_free(pixels);

    return texture;
}

/*
void loadTextureWithSDL3(vector<figures>& InfoFigures, void* appstate) {
    const auto* state = static_cast<AppState*>(appstate);

    for (int i = 0; i < InfoFigures.size(); i++) {

        SDL_Surface* surface = SDL_LoadPNG(InfoFigures[i].path);
        if (!surface) {
            cout << "Fehler beim Laden von Figur:" << InfoFigures[i].name << SDL_GetError() << endl;
        }

        // Textur erstellen
        InfoFigures[i].texture = SDL_CreateTextureFromSurface(state->renderer, surface);
        SDL_DestroySurface(surface);
    }


}

*/
void calculateFieldFromCoordinates(void* appstate, const float& mouseX, const float& mouseY, int* selectedRow, int* selectedCol, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button) {
    const auto* state = static_cast<AppState*>(appstate);

    constexpr int boardFields = 8;

    float squareSize = Game_Layout->boardSize / static_cast<float>(boardFields);     //da Feld quadratisch egal ob Breite oder Höhe
    /*
    *selectedRow = 8 - (static_cast<int>(mouseY / squareSize));
    *selectedCol = (static_cast<int>(mouseX / squareSize)) + 1;
    */

    if (mouseX <= Game_Layout->boardSize) {    //mausklick im Spielfeld
        *selectedRow = 8 - (static_cast<int>(mouseY / squareSize));
        *selectedCol = (static_cast<int>(mouseX / squareSize)) + 1;
    }
    else {
        if ((mouseY <= WindowConfigs->WindowHeight / 3.0f)) {
            Button->normal_move = !Button->normal_move;
            cout << "Normal Move: " << Button->normal_move << endl;
        }
        else if (mouseY < WindowConfigs->WindowHeight / 3.0f * 2) {
            Button->split_move = !Button->split_move;
            cout << "Split Move: " << Button->split_move << endl;
        }
        else {
            Button->merge_move = !Button->merge_move;
            cout << "Merge Move: " << Button->merge_move << endl;
        }
    }
}


/**
 * Handles SDL events
 * @param appstate AppState struct to store information of the apps state in
 * @param event Pointer of the event to handle
* @return SDL_APP_FAILURE to terminate with an error, SDL_APP_SUCCESS to
 *        terminate with success, SDL_APP_CONTINUE to continue.
 */
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
            *mouseX = event->button.x;          //X Werte von der Maus
            *mouseY = event->button.y;          //Y Werte von der Maus
            calculateFieldFromCoordinates(appstate, *mouseX, *mouseY, selectedRow, selectedCol, Game_Layout, WindowConfigs, Button);
            state->redraw = true;
        }
    }
    default:    // ignore all events not listed above
        break;
    }

    return SDL_APP_CONTINUE;
}

/**
 * Gets called when quitting SDL
 * @param appstate AppState struct to store information of the apps state in
* @param result SDL_APP_FAILURE to terminate with an error, SDL_APP_SUCCESS to
 *              terminate with success, SDL_APP_CONTINUE to continue.
 */
void AppQuit(void* appstate, SDL_AppResult result) {
    //Cleanup here
    const auto* state = static_cast<AppState*>(appstate);
    SDL_DestroyRenderer(state->renderer);
    SDL_DestroyWindow(state->window);
    if (result == SDL_APP_FAILURE)
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error",
            SDL_GetError(), appstate ? state->window : NULL);
    SDL_free(appstate); // deallocate AppState
    SDL_Quit();
}

/**
 * Pseudo main function to run an SDL Window
 * @return error code
 */
int main() {
    // init SDL
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

        Spielfeld_Reset(Spielfeld);
        Startaufstellung_Bauern(bauern, Spielfeld);
        Startaufstellung_Springer(springer, Spielfeld);
        Startaufstellung_Laeufer(laeufer, Spielfeld);
        Startaufstellung_Tuerme(tuerme, Spielfeld);
        Startaufstellung_Damen(damen, Spielfeld);
        Startaufstellung_Koenige(koenige, Spielfeld);

        Layout Game_Layout;
        MoveButton Button_Texture;


        createTexture(appstate, Spielfeld);                //Datenpfad in Textur
        createButtonTextures(appstate, &Button_Texture);

        // main loop
        //bool running = true;
        SDL_Event event;

        float mouseX = 0;  //mouseclick x coordinates
        float mouseY = 0;  //mouseclick y coordinates
        int selectedRow = 0;    //current selectedRow
        int selectedCol = 0;    //current selectedCol

        // 1. DELETE 'a' and 'b'. Add a newClick flag instead:
        bool newClick = false;

        vector <Moegliches_Feld> Vector_Moegliche_felder;
        bool piece_moved = false;

        while (true) {
            // handle events
            while (SDL_PollEvent(&event)) {

                // 2. DETECT the physical click here and set the flag
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

            // 3. ONLY execute the game logic if a fresh click happened
            if (newClick && !Spielfeld.schachmatt) {
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

                if (Ceck_For_Promotion(Spielfeld, damen)) {
                    createTexture(appstate, Spielfeld);
                }
                Check_For_Kollaps_Verschraenkung(Spielfeld);
                cout << "Ausgewaehlte Reihe: " << selectedRow << endl;
                cout << "Ausgewaehlte Spalte: " << selectedCol << endl;

                if (Spielfeld.piece_selected) {
                    Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(Spielfeld);
                    Vector_Moegliche_felder = Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();
                }
                else {
                    Vector_Moegliche_felder.clear();
                }

                // 4. RESET the flag so we don't process the same click again!
                newClick = false;
            }

            // (Render frame as normal below)
            if (Spielfeld.piece_selected) {
                AppIterate(appstate, &WindowConfigs, &Game_Layout, &Button_Texture, Spielfeld, Vector_Moegliche_felder);
            }
            else {
                AppIterate(appstate, &WindowConfigs, &Game_Layout, &Button_Texture, Spielfeld, {});
            }
        }
    }

    // Deconstruct
    AppQuit(appstate, result);
    return 0;
}
