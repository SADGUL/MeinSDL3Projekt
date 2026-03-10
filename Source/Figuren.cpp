
#include "../Header/Figuren.h"
#include "../Header/Brett.h"
#include <vector>
#include <string>
using namespace std;


SDL_Texture* Figuren::Get_Texture() {
	return texture_ptr;
}
string Figuren::Get_Dateipfad() {
	return dateipfad;
}
char Figuren::Get_Name() {
	return name;
}
bool Figuren::Get_Geschlagen() {
	return geschlagen;
}
bool Figuren::Get_Farbe() {
	return weiss;
}
int Figuren::Get_Spalte() {
	return spalte;
}
int Figuren::Get_Zeile() {
	return zeile;
}
bool Figuren::Get_Gezogen() {
	return gezogen;
}
vector <Moegliches_Feld> Figuren::Get_Moegliche_Felder() {
	return moegliche_felder;
}
float Figuren::Get_Wahrscheinlichkeit() {
	return wahrscheinlichkeit;
}
vector <Figuren*> Figuren::Get_Same_Piece() {
	return same_piece;
}
vector <int> Figuren::Get_Same_Piece_S() {
	return same_piece_s;
}
vector <int> Figuren::Get_Same_Piece_Z() {
	return same_piece_z;
}

vector <Figuren*> Figuren::Get_Connected_Piece() {
	return connected_piece;
}
vector <int> Figuren::Get_Connected_Piece_S() {
	return connected_piece_s;
}
vector <int> Figuren::Get_Connected_Piece_Z() {
	return connected_piece_z;
}

void Figuren::Set_Name(char neuer_name) {
	name = neuer_name;
}
void Figuren::Set_Geschlagen(bool neuer_zustand) {
	geschlagen = neuer_zustand;
}
void Figuren::Set_Farbe(bool neue_farbe) {
	weiss = neue_farbe;
}
void Figuren::Set_Spalte(int neue_spalte) {
	spalte = neue_spalte;
}
void Figuren::Set_Zeile(int neue_zeile) {
	zeile = neue_zeile;
}
void Figuren::Set_Gezogen(bool neuer_zustand) {
	gezogen = neuer_zustand;
}
void Figuren::Set_Moegliche_Felder(Brett& spielfeld) {
	return;
}

void Figuren::Set_Dateipfad(string neuer_dateipfad) {
	dateipfad = neuer_dateipfad;
}

void Figuren::Set_Texture(SDL_Texture* neue_texture_ptr) {
	texture_ptr = neue_texture_ptr;
}
void Figuren::Set_Wahrscheinlichkeit(const float wahrscheinlichkeit) {
	this->wahrscheinlichkeit = wahrscheinlichkeit;
}

void Figuren::Add_Same_Pieces(vector <Figuren*> new_same_piece) {
	
	same_piece = new_same_piece;
}
void Figuren::Add_Same_Piece(Figuren* F) {
	if (F != nullptr) {
		same_piece.push_back(F);
	}
}
void Figuren::Clear_Same_Piece() {
	same_piece.clear();
}

void Figuren::Add_Same_Piece_S(int s) {
	same_piece_s.push_back(s);
}

void Figuren::Add_Same_Piece_Z(int z) {
	same_piece_z.push_back(z);
}

void Figuren::Clear_Same_Piece_S() {
	same_piece_s.clear();
}
void Figuren::Clear_Same_Piece_Z() {
	same_piece_z.clear();
}

void Figuren::Add_Connected_Pieces(vector <Figuren*> new_connected_piece) {

	same_piece = new_connected_piece;
}
void Figuren::Add_Connected_Piece(Figuren* F) {
	if (F != nullptr) {
		connected_piece.push_back(F);
	}
}
void Figuren::Replace_Connected_Piece(int i, Figuren* F) {
	if (F != nullptr) {
		connected_piece[i] = F;
	}
}
void Figuren::Clear_Connected_Piece() {
	connected_piece.clear();
}

void Figuren::Add_Connected_Piece_S(int s) {
	connected_piece_s.push_back(s);
}

void Figuren::Add_Connected_Piece_Z(int z) {
	connected_piece_z.push_back(z);
}

void Figuren::Clear_Connected_Piece_S() {
	connected_piece_s.clear();
}
void Figuren::Clear_Connected_Piece_Z() {
	connected_piece_z.clear();
}
