  #pragma once
#include <SDL3/SDL.h>

#include <vector>
#include <array>
#include <string>


using namespace std;

struct Brett;
struct Moegliches_Feld;

class Figuren
{

protected:

	bool geschlagen = false; // koenig
	bool weiss = true;
	bool gezogen = false; // Turm Bauer Koenig
	char name = 'F';
	
	// bool gewaehlt = false;

	float wahrscheinlichkeit = 1.0f;
	vector <Figuren*> same_piece; // hier sind pointer auf figuren, die dieselebe Figur sind, jede "selbe Figur" hat pointer auf alle anderen selben Figuren
	vector <int> same_piece_s;
	vector <int> same_piece_z;



	vector <Figuren*> connected_piece; // hier sind pointer auf Figuren, mit denen dei Figur verschraenkt ist
	vector <int> connected_piece_s;
	vector <int> connected_piece_z;


	vector <Moegliches_Feld> moegliche_felder ;
	int spalte = 0;
	int  zeile = 0;
	
	

	// Ausgabe:
	string dateipfad = "leer";
	SDL_Texture* texture_ptr = nullptr;


public:
	
	
	virtual char Get_Name();
	virtual bool Get_Geschlagen();
	virtual bool Get_Farbe();
	virtual int Get_Spalte();
	virtual int Get_Zeile();
	virtual bool Get_Gezogen();
	virtual vector <Moegliches_Feld> Get_Moegliche_Felder();
	virtual string Get_Dateipfad();
	virtual SDL_Texture* Get_Texture();
	virtual float Get_Wahrscheinlichkeit();

	virtual vector <Figuren*> Get_Same_Piece();
	virtual vector <int> Get_Same_Piece_S();
	virtual vector <int> Get_Same_Piece_Z();

	virtual vector <Figuren*> Get_Connected_Piece();
	virtual vector <int> Get_Connected_Piece_S();
	virtual vector <int> Get_Connected_Piece_Z();


	virtual void Set_Name(char neuer_name);
	virtual void Set_Geschlagen(bool neuer_zustand);
	virtual void Set_Farbe(bool neue_farbe);
	virtual void Set_Spalte(int neue_Spalte);
	virtual void Set_Zeile(int neue_Zeile);
	virtual void Set_Gezogen(bool neuer_zustand);
	virtual void Set_Moegliche_Felder(Brett& spielfeld);
	virtual void Set_Wahrscheinlichkeit(float wahrscheinlichkeit);

	virtual void Add_Same_Pieces(vector <Figuren*> new_same_piece);
	virtual void Add_Same_Piece(Figuren* F);
	virtual void Clear_Same_Piece();

	virtual void Add_Same_Piece_S(int s);
	virtual void Add_Same_Piece_Z(int z);

	virtual void Clear_Same_Piece_S();
	virtual void Clear_Same_Piece_Z();

	virtual void Add_Connected_Pieces(vector <Figuren*> new_connected_piece);
	virtual void Add_Connected_Piece(Figuren* F);
	virtual void Replace_Connected_Piece(int i, Figuren);
	virtual void Clear_Connected_Piece();

	virtual void Add_Connected_Piece_S(int s);
	virtual void Add_Connected_Piece_Z(int z);

	virtual void Clear_Connected_Piece_S();
	virtual void Clear_Connected_Piece_Z();

	// 
	virtual void Set_Dateipfad(string neuer_dateipfad);
	virtual void Set_Texture(SDL_Texture* neuer_texture_ptr);
	



};

