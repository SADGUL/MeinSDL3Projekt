#include "../Header/Koenig.h"

#include <vector>
using namespace std;

 vector <Moegliches_Feld> Koenig::Get_Moegliche_Felder() {

	return moegliche_felder;
}


void Koenig::Set_Moegliche_Felder(Brett& spielfeld) {
	moegliche_felder.clear();
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



	for (int s = -1; s < 2; s++) {
		for (int z = -1; z < 2; z++) {
			Moegliches_Feld F;
			if (s == 0 && z == 0) {
				continue;
			}
			moving_over_self = false;
			for (int j = 0; j < sv.size(); j++) { // Schauen ob der koenig da eine alternative position von sich selbst hat
				if (sv[j] == spalte + s && zv[j] == zeile + z) {
					moving_over_self = true;
					break;
				}
			}

			if (spalte + s >= 1 && spalte + s <= 8) {
				if (  zeile + z >= 1 &&  zeile + z <= 8 ) { // liegt das Feld auf dem Brett
					if (spielfeld.Felder[spalte + s - 1][zeile + z - 1] == nullptr) {
						F.spalte = spalte + s;
						F.zeile = zeile + z;
						moegliche_felder.push_back(F);
					}
					else if (spielfeld.Felder[spalte + s - 1][zeile + z - 1]->Get_Farbe() != weiss) { // darf der König da hinziehn
						F.spalte = spalte + s;
						F.zeile = zeile + z;
						moegliche_felder.push_back(F);
					}
					else if (spielfeld.Felder[spalte + s - 1][zeile + z - 1]->Get_Wahrscheinlichkeit() != 1.0 && !moving_over_self) {
						F.spalte = spalte + s;
						F.zeile = zeile + z;
						moegliche_felder.push_back(F);
					}
				}
			}
		}
	}
	bool scw = Check_For_scw(spielfeld);
	bool lcw = Check_For_lcw(spielfeld);
	bool scb = Check_For_scb(spielfeld);
	bool lcb = Check_For_lcb(spielfeld);
	
	Moegliches_Feld F;

	// Rochade (jetzt kann der Koenig zwei Felder gehen, allerding beqwegt sich der Turm noch nicht
	if (scw && weiss) {
		F.spalte = spalte + 2;
		F.zeile = zeile;
		moegliche_felder.push_back(F);
	}
	if (lcw && weiss) {
		F.spalte = spalte - 2;
		F.zeile = zeile;
		moegliche_felder.push_back(F);
	}
	if (scb && !weiss) {
		F.spalte = spalte + 2;
		F.zeile = zeile;
		moegliche_felder.push_back(F);
	}
	if (lcb && !weiss) {
		F.spalte = spalte - 2;
		F.zeile = zeile;
		moegliche_felder.push_back(F);
	}
	
}

bool Koenig::Check_For_scw(Brett spielfeld) {
	bool scw = false;
	if (spielfeld.Felder[5 - 1][1 - 1] != nullptr && spielfeld.Felder[8 - 1][1 - 1] != nullptr) {
		if (!spielfeld.Felder[5 - 1][1 - 1]->Get_Gezogen() && !spielfeld.Felder[8 - 1][1 - 1]->Get_Gezogen()) {
			if (spielfeld.Felder[6 - 1][0] == nullptr && spielfeld.Felder[7 - 1][0] == nullptr) {
				scw = true;
			}
		}
	}
	return scw;
}
bool Koenig::Check_For_lcw(Brett spielfeld) {
	bool lcw = false;
	if (spielfeld.Felder[5 - 1][1 - 1] != nullptr && spielfeld.Felder[1 - 1][1 - 1] != nullptr) {
		if (!spielfeld.Felder[5 - 1][1 - 1]->Get_Gezogen() && !spielfeld.Felder[1 - 1][1 - 1]->Get_Gezogen()) {
			if (spielfeld.Felder[2 - 1][1 - 1] == nullptr && spielfeld.Felder[3 - 1][1 - 1] == nullptr && spielfeld.Felder[4 - 1][1 - 1]) {
				lcw = true;
			}
		}
	}
	return lcw;
}
bool Koenig::Check_For_scb(Brett spielfeld) {
	bool scb = false;
	if (spielfeld.Felder[5 - 1][8 - 1] != nullptr && spielfeld.Felder[8 - 1][8 - 1] != nullptr) {
		if (!spielfeld.Felder[5 - 1][8 - 1]->Get_Gezogen() && !spielfeld.Felder[8 - 1][8 - 1]->Get_Gezogen()) {
			if (spielfeld.Felder[6 - 1][8 - 1] == nullptr && spielfeld.Felder[7 - 1][8 - 1] == nullptr) {
				scb = true;
			}
		}
	}
	return scb;
}
bool Koenig::Check_For_lcb(Brett spielfeld) {
	bool lcb = false;
	if (spielfeld.Felder[5 - 1][8 - 1] != nullptr && spielfeld.Felder[1 - 1][8 - 1] != nullptr) {
		if (!spielfeld.Felder[5 - 1][8 - 1]->Get_Gezogen() && !spielfeld.Felder[1 - 1][8 - 1]->Get_Gezogen()) {
			if (spielfeld.Felder[2 - 1][8 - 1] == nullptr && spielfeld.Felder[3 - 1][8 - 1] == nullptr && spielfeld.Felder[4 - 1][8 - 1] == nullptr) {
				lcb = true;
			}
		}
	}
	return lcb;
}
