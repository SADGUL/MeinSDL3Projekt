
#ifndef main_quanten
#define main_quanten


#include <SDL3/SDL.h>

#include <vector>
using namespace std; //notwendig vor vector

struct Brett;
struct Moegliches_Feld;
struct Window_Configuration;
struct Layout;
struct MoveButton;

SDL_AppResult AppInit(void** appstate, Window_Configuration WindowConfigs);
SDL_AppResult AppIterate(void* appstate, Window_Configuration* WindowConfigs, Layout* Game_Layout, MoveButton* Button_Texture, Brett& Spielfeld, vector <Moegliches_Feld> Vector_Moegliche_felder);
void AppQuit(void* appstate, SDL_AppResult result);
SDL_AppResult AppEvent(void* appstate, SDL_Event* event, float* mouseX, float* mouseY, int* selectedRow, int* selectedCol, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button);
void createTexture(void* appstate, Brett& Spielfeld);
int drawChessboard(SDL_Window* window, SDL_Renderer* renderer, float width);

void GreenCircle(SDL_Renderer* renderer, SDL_FRect rect);

void RenderTextures(SDL_Window* window, SDL_Renderer* renderer, Brett& Spielfeld, vector <Moegliches_Feld> Vector_Moegliche_felder, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button_Texture);
void createButtonTextures(void* appstate, MoveButton* Button_Texture);
SDL_Texture* CreateTexture_from_Image(SDL_Renderer* renderer, const char* c_path);
void calculateFieldFromCoordinates(void* appstate, const float& mouseX, const float& mouseY, int* selectedRow, int* selectedCol, Layout* Game_Layout, Window_Configuration* WindowConfigs, MoveButton* Button);
void setPossibleFields(Brett& Spielfeld);



#endif //QUANTUMCHESS_SDL_TEST_H

