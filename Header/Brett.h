#pragma once
#include "Figuren.h"
struct Brett
{
	Figuren* Felder[8][8];
	
	// Zug Logik
	bool piece_selected = false;
	int selected_piece_s = -1;
	int selected_piece_z = -1;

	// Spiel logik
	bool whites_turn = true;
	bool schachmatt = false;


	// En Passant
	bool en_passant = false;
	int en_passant_spalte = -2;
	int en_passant_zeile = -2;

	// Split Move
	bool split_move_selected = false;
	
	bool first_field_selected = false;
	int first_field_s = -1;
	int first_field_z = -1;

	bool second_field_selected = false;
	int second_field_s = -1;
	int second_field_z = -1;


	

	// Merge Move

	bool merge_move_selected = false;

	bool first_piece_selected = false;
	int first_piece_s = -1;
	int first_piece_z = -1;

	bool second_piece_selected = false;
	int second_piece_s = -1;
	int second_piece_z = -1;

	// Verschraenkung

	vector <Figuren*> F_Im_Weg;
	vector <int> F_Im_Weg_s;
	vector <int> F_Im_Weg_z;



};
struct Moegliches_Feld
{
	int spalte;
	int zeile;
	float wahrscheinlichkeit = 1.0;
};

struct Window_Configuration {
	int WindowWidth = 960;
	int WindowHeight = 720;
	int minWidth = 640;
	int minHeight = 480;
};

struct Layout {
	float sidebarRatio = 0.2f;   // 20 % UI
	float sidebarWidth;
	float WidthMinusSidebar;
	float boardSize;
};

struct MoveButton {
	SDL_Texture* normal;
	SDL_Texture* split;
	SDL_Texture* merge;
	bool normal_move = true;
	bool split_move = false;
	bool merge_move = false;

};