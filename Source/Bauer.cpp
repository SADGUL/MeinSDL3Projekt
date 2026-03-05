#include <iostream>
#include "../Header/Bauer.h"
#include "../Header/Brett.h"
#include <vector>
#include <array>
using namespace std;



/*void Bauer::Set_Moegliche_Felder(Brett spielfeld) {

	moegliche_felder.clear();
	int z = -1;
	
	if (weiss) {
		z = 1;
		
	}
	
	if (zeile + z <= 8 && zeile + z >= 1) { // Zeile im Feld (Bauer nicht am Spielfeldrand
		for (int s = -1; s < 2; s++) {
			Moegliches_Feld F;
			if (spielfeld.Felder[spalte-1][zeile + z-1] == nullptr && s == 0) { // grade aus ziehen
				F.spalte = spalte;
				F.zeile = zeile + z;
				moegliche_felder.push_back(F);
			}
			else if(spalte + s >= 1 && spalte + s <= 8 && s!=0){
				
				if (spielfeld.Felder[spalte + s - 1][zeile + z - 1] != nullptr) {
					
					if (spielfeld.Felder[spalte + s - 1][zeile + z - 1]->Get_Farbe() != weiss) {
						
						F.spalte = spalte + s;
						F.zeile = zeile + z;
						moegliche_felder.push_back(F);
					}
				}
			}
		}
	}

	if (gezogen == false && spielfeld.Felder[spalte-1][zeile + z-1] == nullptr && spielfeld.Felder[spalte-1][zeile + (2 * z) -1] == nullptr) {
		Moegliches_Feld F;
		F.spalte = spalte;
		F.zeile = zeile + 2 * z;
		moegliche_felder.push_back(F);
	}


	if (spielfeld.en_passant) {
		if (spalte + 1 == spielfeld.en_passant_spalte || spalte - 1 == spielfeld.en_passant_spalte) {
			if (zeile == spielfeld.en_passant_zeile) {
				Moegliches_Feld F;
				F.spalte = spielfeld.en_passant_spalte;
				F.zeile = zeile + z;
				moegliche_felder.push_back(F);
			}
		}
	}
	
}*/
void Bauer::Set_Moegliche_Felder(Brett& spielfeld) {

	moegliche_felder.clear();

	spielfeld.F_Im_Weg.clear();
	spielfeld.F_Im_Weg_s.clear();
	spielfeld.F_Im_Weg_z.clear();

	int z = -1;
	if (weiss) {
		z = 1;
	}
	float p = 1.0;
	vector <int> sv;
	vector <int> zv;
	bool moving_over_self = false;

	// Hole allen alternativen Positione der Figur
	for (int i = 0; i < spielfeld.Felder[spalte - 1][zeile - 1]->Get_Same_Piece().size(); i++) {
		int s = spielfeld.Felder[spalte - 1][zeile - 1]->Get_Same_Piece()[i]->Get_Spalte();
		int z = spielfeld.Felder[spalte - 1][zeile - 1]->Get_Same_Piece()[i]->Get_Zeile();
		sv.push_back(s);
		zv.push_back(z);
	}
	

	if (zeile + z <= 8 && zeile + z >= 1) { // Zeile im Feld (Bauer nicht am Spielfeldrand
		for (int s = -1; s < 2; s++) {
			Moegliches_Feld F;
			for (int j = 0; j < sv.size(); j++) {
				if (sv[j] == spalte + s && zv[j] == zeile + z) {
					moving_over_self = true;
					break;
				}
			}
			if (  s == 0) { // grade aus ziehen
				if (spielfeld.Felder[spalte - 1][zeile + z - 1] == nullptr) {
					F.spalte = spalte;
					F.zeile = zeile + z;
					F.wahrscheinlichkeit = p;
					moegliche_felder.push_back(F);
				}
				else {
					if (spielfeld.Felder[spalte - 1][zeile + z - 1]->Get_Wahrscheinlichkeit() != 1.0 && !moving_over_self) {
						F.spalte = spalte;
						F.zeile = zeile + z;
						F.wahrscheinlichkeit = p;
						moegliche_felder.push_back(F);
					}
				}
			}
			
			else if (spalte + s >= 1 && spalte + s <= 8 && s != 0) { // Diagonal schlagen

				if (spielfeld.Felder[spalte + s - 1][zeile + z - 1] != nullptr) {

					if (spielfeld.Felder[spalte + s - 1][zeile + z - 1]->Get_Farbe() != weiss || spielfeld.Felder[spalte + s - 1][zeile + z - 1]->Get_Wahrscheinlichkeit() != 1.0) {

						F.spalte = spalte + s;
						F.zeile = zeile + z;
						F.wahrscheinlichkeit = 1.0;
						moegliche_felder.push_back(F);
					}
				}
			}
		}
	}

	if (gezogen == false && spielfeld.Felder[spalte - 1][zeile + z - 1] == nullptr && spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1] == nullptr) {
		Moegliches_Feld F;
		F.spalte = spalte;
		F.zeile = zeile + 2 * z;
		F.wahrscheinlichkeit = 1.0;
		moegliche_felder.push_back(F);
	}
	else if (gezogen == false && (spielfeld.Felder[spalte - 1][zeile + z - 1] != nullptr || spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1] != nullptr)) {

		if (spielfeld.Felder[spalte - 1][zeile + z - 1] != nullptr && spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1] == nullptr) {
			Moegliches_Feld F;
			F.spalte = spalte;
			F.zeile = zeile + 2 * z;
			F.wahrscheinlichkeit = 1.0 - spielfeld.Felder[spalte - 1][zeile + z - 1]->Get_Wahrscheinlichkeit();
			moegliche_felder.push_back(F);
			spielfeld.F_Im_Weg.push_back(spielfeld.Felder[spalte - 1][zeile + z - 1]);
			spielfeld.F_Im_Weg_s.push_back(spalte);
			spielfeld.F_Im_Weg_z.push_back(zeile + z);
		}
		else if (spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1] != nullptr && spielfeld.Felder[spalte - 1][zeile + z - 1] == nullptr) {
			if (spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1]->Get_Wahrscheinlichkeit() != 1.0) {
				Moegliches_Feld F;
				F.spalte = spalte;
				F.zeile = zeile + 2 * z;
				F.wahrscheinlichkeit = 1.0;
				moegliche_felder.push_back(F);
			}
		}
		else if (spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1] != nullptr && spielfeld.Felder[spalte - 1][zeile + z - 1] != nullptr) {
			if (spielfeld.Felder[spalte - 1][zeile + (2 * z) - 1]->Get_Wahrscheinlichkeit() != 1.0) {
				Moegliches_Feld F;
				F.spalte = spalte;
				F.zeile = zeile + 2 * z;
				F.wahrscheinlichkeit = 1.0 - spielfeld.Felder[spalte - 1][zeile + z - 1]->Get_Wahrscheinlichkeit();
				moegliche_felder.push_back(F);
				spielfeld.F_Im_Weg.push_back(spielfeld.Felder[spalte - 1][zeile + z - 1]);
				spielfeld.F_Im_Weg_s.push_back(spalte);
				spielfeld.F_Im_Weg_z.push_back(zeile + z);
			}
		}





	}


	if (spielfeld.en_passant) {
		if (spalte + 1 == spielfeld.en_passant_spalte || spalte - 1 == spielfeld.en_passant_spalte) {
			if (zeile == spielfeld.en_passant_zeile) {
				Moegliches_Feld F;
				F.spalte = spielfeld.en_passant_spalte;
				F.zeile = zeile + z;
				moegliche_felder.push_back(F);
			}
		}
	}

}


