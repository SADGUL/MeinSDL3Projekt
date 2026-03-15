#include <SDL3/SDL.h>
#include <vector>
#include <iostream>

using namespace std;

int runSDL() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {            //SDL_INIT_VIDEO initialisiert den Grafikteil von SDL
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
    }

    //Hier kann man die Groe?e des Fensters einstellen
    int width = 1200;
    int height = 1000;
    SDL_Window* window = SDL_CreateWindow("Karte", width, height, SDL_WINDOW_RESIZABLE); ///Fenster wird erzeugt (Gr??e ver?nderbar)
    if (!window) {
        SDL_Log("Fehler beim Fenster erstellen: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr); //Renderer bzw. Grafikausgabe erzeugen
    if (!renderer) {
        SDL_Log("Fehler beim Renderer erstellen: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_Surface* surface = SDL_LoadBMP("./assets/chessboard.bmp");
    if (surface == nullptr) {
        cout << "ERROR:could not open BMP file\n";
    }
    //5)Texture
    SDL_Texture* texture = SDL_CreateTextureFromSurface(
        renderer, surface);
    if (texture == nullptr) {
        cout << "ERROR:could not generate texture from surface\n";
        SDL_DestroySurface(surface);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    bool running = true;
    SDL_Event event;

    while (running) {
        // Events verarbeiten
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }


            // Renderer l?schen (Hintergrundfarbe)
            SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); // Dunkelgrauer Hintergrund
            SDL_RenderClear(renderer);

            // Schachbrett-Bild zeichnen
            if (texture) {
                // Zielrechteck (kann angepasst werden)
                SDL_FRect dest;
                dest.x = 0;    // Linker Rand
                dest.y = 0;    // Oberer Rand
                dest.w = width;   // Volle Fensterbreite
                dest.h = height;  // Volle Fensterh?he

                // ODER: Zentriert mit fester Gr??e:
                // dest.x = (width - 600) / 2;   // Zentriert bei 600x600
                // dest.y = (height - 600) / 2;
                // dest.w = 600;
                // dest.h = 600;

                SDL_RenderTexture(renderer, texture, nullptr, &dest);
            }

            // Renderer aktualisieren
            SDL_RenderPresent(renderer);

            // Kurze Pause (nicht n?tig, aber gut f?r CPU)
            SDL_Delay(16); // ~60 FPS
        }
    }

    // 4)Cleanup&close
    SDL_DestroySurface(surface);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
