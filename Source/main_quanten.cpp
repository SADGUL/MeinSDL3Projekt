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

                if (Check_For_Promotion(Spielfeld, damen)) {
                    createTexture(appstate, Spielfeld);
                }
                Check_For_Kollaps_Verschraenkung(Spielfeld);

                // ==========================================
                // CHECK FOR MATE IMMEDIATELY AFTER COLLAPSE
                // ==========================================
                Check_for_Mate(Spielfeld);

                cout << "Ausgewaehlte Reihe: " << selectedRow << endl;
                cout << "Ausgewaehlte Spalte: " << selectedCol << endl;

                // ==========================================
                // WINNER POPUP ANNOUNCEMENT
                // ==========================================
                if (Spielfeld.schachmatt) {
                    const char* winnerText = Spielfeld.whites_turn ? "Black wins by Checkmate!" : "White wins by Checkmate!";
                    auto* state = static_cast<AppState*>(appstate);

                    // The game pauses right here until the player closes the pop-up
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Game Over!", winnerText, state->window);

                    // Break out of the while(true) loop to trigger AppQuit and close the game!
                    break;
                }
                // ==========================================

                if (Spielfeld.piece_selected) {
                    Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(Spielfeld);
                    Vector_Moegliche_felder = Spielfeld.Felder[Spielfeld.selected_piece_s - 1][Spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();
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