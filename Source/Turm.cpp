#include "../Header/Turm.h"

#include <vector>
using namespace std;


vector <Moegliches_Feld> Turm::Get_Moegliche_Felder() {

	return moegliche_felder;
}

void Turm::Set_Moegliche_Felder(Brett& spielfeld) {
	moegliche_felder.clear();
	spielfeld.F_Im_Weg.clear();
	spielfeld.F_Im_Weg_s.clear();
	spielfeld.F_Im_Weg_z.clear();
	float p = 1.0;
	Moegliches_Feld F;
	
	int s = spalte;

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

	


	for (s -1; s > 0; s--) { //links Bewegung
		
		F.spalte = s;
		F.zeile = zeile;
		F.wahrscheinlichkeit = p;
		for (int j = 0; j < sv.size(); j++) {
			if (sv[j] == s  && zv[j] == zeile ) {
				moving_over_self = true;
				break;
			}
		}
		if (spielfeld.Felder[s - 1][zeile - 1] == nullptr) {
			moegliche_felder.push_back(F);
		}
		if (s != spalte && spielfeld.Felder[s - 1][zeile - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][zeile - 1]->Get_Wahrscheinlichkeit() == 1.0 || moving_over_self) {
				if (spielfeld.Felder[s - 1][zeile - 1]->Get_Farbe() != weiss) {
					moegliche_felder.push_back(F);
					break;
				}
				else {
					break;
				}
			}
			else  {
				moegliche_felder.push_back(F);
				p = p - p * spielfeld.Felder[s - 1][zeile - 1]->Get_Wahrscheinlichkeit();
				spielfeld.F_Im_Weg.push_back(spielfeld.Felder[s - 1][zeile - 1]);
				spielfeld.F_Im_Weg_s.push_back(s);
				spielfeld.F_Im_Weg_z.push_back(zeile);
			}
		}
	}
	s = spalte;
	p = 1.0;
	moving_over_self = false;
	for (s +1; s < 9; s++) { //rechts Bewegung
		
		F.wahrscheinlichkeit = p;
		F.spalte = s;
		F.zeile = zeile;
		for (int j = 0; j < sv.size(); j++) {
			if (sv[j] == s && zv[j] == zeile) {
				moving_over_self = true;
				break;
			}
		}
		if (spielfeld.Felder[s - 1][zeile - 1] == nullptr) {
			moegliche_felder.push_back(F);
		}
		if (s != spalte && spielfeld.Felder[s - 1][zeile - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][zeile - 1]->Get_Wahrscheinlichkeit() == 1.0 || moving_over_self) {
				if (spielfeld.Felder[s - 1][zeile - 1]->Get_Farbe() != weiss) {
					moegliche_felder.push_back(F);
					break;
				}
				else {
					break;
				}
			}
			else {
				moegliche_felder.push_back(F);
				p = p - p * spielfeld.Felder[s - 1][zeile - 1]->Get_Wahrscheinlichkeit();
				spielfeld.F_Im_Weg.push_back(spielfeld.Felder[s - 1][zeile - 1]);
				spielfeld.F_Im_Weg_s.push_back(s);
				spielfeld.F_Im_Weg_z.push_back(zeile);
			}
		}
	}
	int z = zeile;
	p = 1.0;
	moving_over_self = false;
	for (z +1; z < 9; z++) { //aufwärts
		F.wahrscheinlichkeit = p;
		F.spalte = spalte;
		F.zeile = z;

		for (int j = 0; j < sv.size(); j++) {
			if (sv[j] == spalte && zv[j] == z) {
				moving_over_self = true;
				break;
			}
		}

		if (spielfeld.Felder[spalte - 1][z - 1] == nullptr) {
			moegliche_felder.push_back(F);
		}
		if (z != zeile && spielfeld.Felder[spalte - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[spalte - 1][z - 1]->Get_Wahrscheinlichkeit() == 1.0 || moving_over_self) {
				if (spielfeld.Felder[spalte - 1][z - 1]->Get_Farbe() != weiss) {
					moegliche_felder.push_back(F);
					break;
				}
				else {
					break;
				}
			}
			else {
				moegliche_felder.push_back(F);
				p = p - p* spielfeld.Felder[spalte - 1][z - 1]->Get_Wahrscheinlichkeit();
				spielfeld.F_Im_Weg.push_back(spielfeld.Felder[spalte - 1][z - 1]);
				spielfeld.F_Im_Weg_s.push_back(spalte);
				spielfeld.F_Im_Weg_z.push_back(z);
			}
		}
	}
	z = zeile;
	p = 1.0;
	moving_over_self = false;
	for (z -1 ; z > 0; z--) { //abwärts
		
		F.spalte = spalte;
		F.zeile = z;
		F.wahrscheinlichkeit = p;

		for (int j = 0; j < sv.size(); j++) {
			if (sv[j] == spalte && zv[j] == z) {
				moving_over_self = true;
				break;
			}
		}
		if (spielfeld.Felder[spalte - 1][z - 1] == nullptr) {
			moegliche_felder.push_back(F);
		}
		if (z != zeile && spielfeld.Felder[spalte - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[spalte - 1][z - 1]->Get_Wahrscheinlichkeit() == 1.0 || moving_over_self) {
				if (spielfeld.Felder[spalte - 1][z - 1]->Get_Farbe() != weiss) {
					moegliche_felder.push_back(F);
					break;
				}
				else {
					break;
				}
			}
			else {
				moegliche_felder.push_back(F);
				p = p - p * spielfeld.Felder[spalte - 1][z - 1]->Get_Wahrscheinlichkeit();
				spielfeld.F_Im_Weg.push_back(spielfeld.Felder[spalte - 1][z - 1]);
				spielfeld.F_Im_Weg_s.push_back(spalte);
				spielfeld.F_Im_Weg_z.push_back(z);
			}
		}
	}
}
