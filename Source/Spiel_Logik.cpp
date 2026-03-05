#include "../Header/Spiel_Logik.h"
#include <random>

void Logik_normal(int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige) {
	Check_for_Mate(spielfeld);
	if (spielfeld.schachmatt) {
		return;
	}
	
	int spalte_ziel = s;
	int zeiel_ziel = z;
	vector <Moegliches_Feld> moegliche_felder;
	if (!spielfeld.piece_selected) {
		if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn){
				spielfeld.piece_selected = true;
				spielfeld.selected_piece_s = s;
				spielfeld.selected_piece_z = z;
				cout << spielfeld.Felder[s - 1][z - 1]->Get_Name() << " gewaehlt" << endl;
			}
		}
	}
	else {
		if (spielfeld.Felder[s - 1][z - 1] != nullptr &&
			spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn) {

			// Switch the selection to the new piece directly!
			spielfeld.selected_piece_s = s;
			spielfeld.selected_piece_z = z;
			cout << "Auswahl gewechselt zu: " << spielfeld.Felder[s - 1][z - 1]->Get_Name() << endl;
			return; // We switched pieces, so stop executing the rest of the move logic!
		}
		spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(spielfeld);

		moegliche_felder = spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();
		if (moegliche_felder.size() == 0) {
			cout << "Keine moeglichen felder" << endl;

		}
		for (int i = 0; i < moegliche_felder.size(); i++) {

			if (moegliche_felder[i].spalte == s && moegliche_felder[i].zeile == z) {
				if (King_selected(spielfeld.selected_piece_s, spielfeld.selected_piece_z, spielfeld)) {
					Check_Castle_Selected(i, moegliche_felder, spielfeld);
				}
				if (spielfeld.en_passant) {
					Check_For_En_Passant(i, moegliche_felder, spielfeld);
				}
				Check_For_Double_Pawn(i, moegliche_felder, spielfeld);
				if (moegliche_felder[i].wahrscheinlichkeit == 1.0) {
					Ziehen(spielfeld.selected_piece_s, spielfeld.selected_piece_z, s, z, spielfeld);
				}
				else {
					Ziehen_Ins_Ungewisse(moegliche_felder[i].wahrscheinlichkeit, spielfeld.selected_piece_s, spielfeld.selected_piece_z, s, z, spielfeld, bauern, springer, laeufer, tuerme, damen, koenige);
				}
				spielfeld.whites_turn = !spielfeld.whites_turn;
				spielfeld.piece_selected = false;
				break;
			}
			
		}
		spielfeld.piece_selected = false;	
	}
}

void Ziehen (int sa, int za, int s, int z, Brett & spielfeld){
	bool zu_schlagende_figur = true;
	if (spielfeld.Felder[s - 1][z - 1] != nullptr) { 
		
		if (!Messung(s, z, spielfeld)) { // zu schlagende Figur echt ?
			cout << "Die zu schlagende Figur ist nicht echt" << endl;
			zu_schlagende_figur = false;
		}
		
		
		if (!Messung(sa, za, spielfeld)) { 
			cout << "Die schlagende Figur ist nicht real" << endl;
			return;
		}
		if (!zu_schlagende_figur && spielfeld.Felder[sa - 1][za - 1]->Get_Name() == 'b' &&  sa != s) {
			return; // Wenn Figur nicht echt war, kann der Bauer nicht schlagen
		}
		else if (zu_schlagende_figur && spielfeld.Felder[sa - 1][za - 1]->Get_Name() == 'b' && sa == s) {
			return; // wenn die Figur echt war, kann der Bauer da nicht hinziehen
		}
		else if (zu_schlagende_figur && spielfeld.Felder[sa - 1][za - 1]->Get_Farbe() == spielfeld.Felder[s - 1][z - 1]->Get_Farbe()) {
			// Man kann keine eigenen "echten" Figuren schlagen
			return;
		}
	}
	
	
	Feld_Leeren(s, z, spielfeld);
	
	spielfeld.Felder[s - 1][z - 1] = spielfeld.Felder[sa - 1][za - 1];
	spielfeld.Felder[s - 1][z - 1]->Set_Spalte(s);
	spielfeld.Felder[s - 1][z - 1]->Set_Zeile(z);
	spielfeld.Felder[s - 1][z - 1]->Set_Gezogen(true);
	spielfeld.Felder[s - 1][z - 1]->Set_Wahrscheinlichkeit(spielfeld.Felder[sa - 1][za - 1]->Get_Wahrscheinlichkeit());
	spielfeld.Felder[sa - 1][za - 1] = nullptr;
		cout << "Figur gezogen" << endl;
	
	
}

void Ziehen_Ins_Ungewisse(float p,int sa, int za, int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige) {
	
	int si = 0; // si = spalte inkrement
	int sd = 0; // sd = spalten differenz

	int zi = 0; // zeile inkrement
	int zd = 0; // zeilde differenz
	//vector <Figuren*> abhaengig;
	if (s > sa) {
		si = 1;
		sd = s - sa;
	}
	else if (s < sa) {
		si = -1;
		sd = sa - s;
	}


	if (z > za) {
		zi = 1;
		zd = z - za;
	}
	else if (z < za) {
		zi = -1;
		zd = za - z;
	}
	
	if ( zd != 0 ){
		for (int i = 1; i < zd; i++) { // Figur, die im Weg steht finden
			int x = sa + i * si;
			int y = za + i * zi;
			if (x > 8 || x < 1 || y > 8 || y < 1) {
				break;
			}
			if (x == s && y == z) { // End psotion erreicht
				break;
			}
			for (int j = 0; j < spielfeld.F_Im_Weg.size();j++) {
				if (x == spielfeld.F_Im_Weg_s[j] && y == spielfeld.F_Im_Weg_z[j]) {
					//jetzt die nicht ziehende Figur mit der im Weg stehenden Figur
					cout << "Verschraenke nicht ziehende Figur" << endl;
					
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece(spielfeld.Felder[sa - 1][za - 1]);
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece_S(sa);
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece_Z(za);

					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece(spielfeld.Felder[x - 1][y - 1]);
					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece_S(x);
					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece_Z(y);
					//abhaengig.push_back(spielfeld.F_Im_Weg[j]);
				}
			}
		}
	}
	else if(sd != 0){
		for (int i = 1; i < sd; i++) { // Figur, die im Weg steht finden
			int x = sa + i * si;
			int y = za + i * zi;
			if (x > 8 || x < 1 || y > 8 || y < 1) {
				break;
			}
			if (x == s && y == z) { // End psotion erreicht
				break;
			}
			for (int j = 0; j < spielfeld.F_Im_Weg.size();j++) {
				if (x == spielfeld.F_Im_Weg_s[j] && y == spielfeld.F_Im_Weg_z[j]) {

					//jetzt die  ziehende Figur mit den nicht im Weg stehenden Figur
					cout << "Verschraenke nicht ziehende Figur" << endl;
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece(spielfeld.Felder[sa - 1][za - 1]);
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece_S(sa);
					spielfeld.Felder[x - 1][y - 1]->Add_Connected_Piece_Z(za);

					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece(spielfeld.Felder[x - 1][y - 1]);
					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece_S(x);
					spielfeld.Felder[sa - 1][za - 1]->Add_Connected_Piece_Z(y);
					//abhaengig.push_back(spielfeld.F_Im_Weg[j]);
				}
			}
		}
	}
	
	
	float pa = spielfeld.Felder[sa - 1][za - 1]->Get_Wahrscheinlichkeit();
	spielfeld.Felder[sa - 1][za - 1]->Set_Wahrscheinlichkeit( pa - (pa * p)); // wahrscheinlichkeit dass die Figur ziehen kann
	if (spielfeld.Felder[s - 1][z - 1] != nullptr) { // Figur schlaegt was, also Messung
		Ziehen(sa, za, s, z, spielfeld);
		if (spielfeld.Felder[sa - 1][za - 1] != nullptr) { // Wenn die Figur nicht gezogen ist
			spielfeld.Felder[sa - 1][za - 1]->Set_Wahrscheinlichkeit(pa); // wahrscheinlichkeit zuruecksetzen
		}
	}
	else {
		// Figur zieht auf ein leeres Feld (keine Messung)
		vector <Figuren*> same_pieces;
		vector <Figuren*> connected_pieces;
		spielfeld.Felder[sa - 1][za - 1]->Set_Wahrscheinlichkeit(pa - (pa * p)); 
		switch (spielfeld.Felder[sa - 1][za - 1]->Get_Name()) {
		case 'b': Create_Bauer(sa, za, s, z, pa * p, bauern, spielfeld);
			break;
		case 'S': Create_Springer(sa, za, s, z, pa * p, springer, spielfeld);
			break;
		case 'L': Create_Laeufer(sa, za, s, z, pa * p, laeufer, spielfeld);
			break;
		case 'T': Create_Turm(sa, za, s, z, pa * p, tuerme, spielfeld);
			break;
		case 'D': Create_Dame(sa, za, s, z, pa * p, damen, spielfeld);
			break;
		case 'K': Create_Koenig(sa, za, s, z, pa * p, koenige, spielfeld);
			break;
		}
		same_pieces = spielfeld.Felder[sa - 1][za - 1]->Get_Same_Piece();
		connected_pieces = spielfeld.Felder[sa - 1][za - 1]->Get_Connected_Piece();

		// Neue Versionen zu alten
		for (int i = 0; i < same_pieces.size(); i++) {
			same_pieces[i]->Add_Same_Piece(spielfeld.Felder[s - 1][z - 1]);
			//same_pieces[i]->Add_Same_Piece(spielfeld.Felder[s2 - 1][z2 - 1]);

		}


		// Alte versionen zur neuen Figuren
		spielfeld.Felder[s - 1][z - 1]->Add_Same_Pieces(same_pieces);


		// Neue versionen zu neuen versionen
		spielfeld.Felder[sa - 1][za - 1]->Add_Same_Piece(spielfeld.Felder[s - 1][z - 1]);
		spielfeld.Felder[s - 1][z - 1]->Add_Same_Piece(spielfeld.Felder[sa - 1][za - 1]);


		// Alternative Positionen der im weg stehenden Figur mit der gezogenen verschraenken
		for (int j = 0; j < connected_pieces.size(); j++) {
			for (int i = 0; i < connected_pieces[j]->Get_Same_Piece().size(); i++) {
				cout << "Verschraenke  ziehende Figur" << endl;
				Figuren* F = connected_pieces[j]->Get_Same_Piece()[i];
				spielfeld.Felder[s - 1][z - 1]->Add_Connected_Piece(F);
				spielfeld.Felder[s - 1][z - 1]->Add_Connected_Piece_S(F->Get_Spalte());
				spielfeld.Felder[s - 1][z - 1]->Add_Connected_Piece_Z(F->Get_Zeile());
				F->Add_Connected_Piece(spielfeld.Felder[s - 1][z - 1]);
				F->Add_Connected_Piece_S(s);
				F->Add_Connected_Piece_Z(z);
			}
		}
	}
}

void Logik_Split(int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige) {
	Check_for_Mate(spielfeld);
	if (spielfeld.schachmatt) {
		return;
	}

	vector <Moegliches_Feld> moegliche_felder;

	// Phase 1: Gar keine Figur ausgewählt -> Initiale Auswahl
	if (!spielfeld.piece_selected) {
		if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn && spielfeld.Felder[s - 1][z - 1]->Get_Name() != 'b') { // Bauer darf keine Quantenzuege ausfuehren
				if (spielfeld.Felder[s - 1][z - 1]->Get_Gezogen() || spielfeld.Felder[s - 1][z - 1]->Get_Name() != 'K') {
					// unbewegter Koenig kein Quantenzug, da sonst Quanten_Rochade -> nicht moeglich
					spielfeld.piece_selected = true;
					spielfeld.selected_piece_s = s;
					spielfeld.selected_piece_z = z;
					cout << spielfeld.Felder[s - 1][z - 1]->Get_Name() << " gewaehlt" << endl;
				}
			}
		}
	}

	// Phase 2: INTERCEPTOR -> Figur wechseln, wenn auf eine andere EIGENE Figur geklickt wird
	else if (spielfeld.Felder[s - 1][z - 1] != nullptr &&
		spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn &&
		!(s == spielfeld.selected_piece_s && z == spielfeld.selected_piece_z)) {

		if (spielfeld.Felder[s - 1][z - 1]->Get_Name() != 'b') {
			if (spielfeld.Felder[s - 1][z - 1]->Get_Gezogen() || spielfeld.Felder[s - 1][z - 1]->Get_Name() != 'K') {

				// Figur direkt wechseln
				spielfeld.selected_piece_s = s;
				spielfeld.selected_piece_z = z;

				// Split-Vorgang zurücksetzen, falls schon ein Ziel gewählt war
				spielfeld.first_field_selected = false;
				spielfeld.second_field_selected = false;

				cout << "Auswahl gewechselt zu: " << spielfeld.Felder[s - 1][z - 1]->Get_Name() << endl;
				return; // Klick abfangen und Funktion beenden
			}
		}
	}

	// Phase 3: Erstes Zielfeld auswählen
	else if (spielfeld.piece_selected && !spielfeld.first_field_selected) {

		spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(spielfeld);

		moegliche_felder = spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();
		for (int i = 0; i < moegliche_felder.size(); i++) {
			if (moegliche_felder[i].spalte == s && moegliche_felder[i].zeile == z && moegliche_felder[i].wahrscheinlichkeit == 1.0) {
				spielfeld.first_field_s = s;
				spielfeld.first_field_z = z;
				spielfeld.first_field_selected = true;
				spielfeld.piece_selected = true;
				cout << "Erstes Feld gewaehlt" << endl;
				break;
			}
		}

		// Wenn kein gültiges Zielfeld geklickt wurde -> Abwählen
		if (!spielfeld.first_field_selected) {
			spielfeld.piece_selected = false;
		}
	}

	// Phase 4: Zweites Zielfeld auswählen und den Move triggern
	else if (spielfeld.piece_selected && spielfeld.first_field_selected) {

		spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Set_Moegliche_Felder(spielfeld);

		moegliche_felder = spielfeld.Felder[spielfeld.selected_piece_s - 1][spielfeld.selected_piece_z - 1]->Get_Moegliche_Felder();
		for (int i = 0; i < moegliche_felder.size(); i++) {
			if (moegliche_felder[i].spalte == s && moegliche_felder[i].zeile == z && moegliche_felder[i].wahrscheinlichkeit == 1.0) {
				spielfeld.second_field_s = s;
				spielfeld.second_field_z = z;
				spielfeld.second_field_selected = true;
				cout << "Zweites Feld gewaehlt" << endl;

				Split_Move(spielfeld, bauern, springer, laeufer, tuerme, damen, koenige);
				break;
			}
		}

		// Egal ob der Move erfolgreich war oder ins Leere geklickt wurde, die Auswahl wird zurückgesetzt
		spielfeld.piece_selected = false;
		spielfeld.first_field_selected = false;
		spielfeld.second_field_selected = false;
	}
}


void Split_Move( Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige) {


	int sa = spielfeld.selected_piece_s;
	int za = spielfeld.selected_piece_z;

	int s1 = spielfeld.first_field_s;
	int z1 = spielfeld.first_field_z;
	
	int s2 = spielfeld.second_field_s;
	int z2 = spielfeld.second_field_z;

	if (spielfeld.Felder[s1 - 1][z1 - 1] == nullptr && spielfeld.Felder[s2 - 1][z2 - 1] == nullptr) {

		vector <Figuren*> same_pieces;
		float p = spielfeld.Felder[sa - 1][za - 1]->Get_Wahrscheinlichkeit() / 2;
		spielfeld.Felder[s1 - 1][z1 - 1] = spielfeld.Felder[sa - 1][za - 1];
		spielfeld.Felder[s1 - 1][z1 - 1]->Set_Spalte(s1);
		spielfeld.Felder[s1 - 1][z1 - 1]->Set_Zeile(z1);
		spielfeld.Felder[s1 - 1][z1 - 1]->Set_Gezogen(true);
		spielfeld.Felder[s1 - 1][z1 - 1]->Add_Same_Pieces(spielfeld.Felder[sa - 1][za - 1]->Get_Same_Piece());
		spielfeld.Felder[s1 - 1][z1 - 1]->Set_Wahrscheinlichkeit(p);
		

		switch (spielfeld.Felder[sa - 1][za - 1]->Get_Name()) {
		
		case 'S': Create_Springer(sa, za,s2,z2,p, springer, spielfeld);
			break;
		case 'L': Create_Laeufer(sa, za, s2, z2, p, laeufer, spielfeld);
			break;
		case 'T': Create_Turm(sa, za, s2, z2, p, tuerme, spielfeld);
			break;
		case 'D': Create_Dame(sa, za, s2, z2, p, damen, spielfeld);
			break;
		case 'K': Create_Koenig(sa, za, s2, z2, p, koenige, spielfeld);
			break;

		}
		
		
		same_pieces = spielfeld.Felder[s1 - 1][z1 - 1]->Get_Same_Piece();

		// Neue Versionen zu alten
		for (int i = 0; i < same_pieces.size(); i++ ){
			same_pieces[i]->Add_Same_Piece(spielfeld.Felder[s1 - 1][z1 - 1]);
			same_pieces[i]->Add_Same_Piece(spielfeld.Felder[s2 - 1][z2 - 1]);

		}


		// Alte versionen zur neuen Figuren
		spielfeld.Felder[s2 - 1][z2 - 1]->Add_Same_Pieces(same_pieces);
		
		
		// Neue versionen zu neuen versionen
		spielfeld.Felder[s1 - 1][z1 - 1]->Add_Same_Piece(spielfeld.Felder[s2 - 1][z2 - 1]);
		spielfeld.Felder[s2 - 1][z2 - 1]->Add_Same_Piece(spielfeld.Felder[s1 - 1][z1 - 1]);


		
	
		spielfeld.Felder[sa - 1][za - 1] = nullptr;
		spielfeld.whites_turn = !spielfeld.whites_turn;
		cout << "Figur geteilt" << endl;
	}
}



void Logik_Merge(int s, int z, Brett& spielfeld) {

	Check_for_Mate(spielfeld);
	if (spielfeld.schachmatt) {
		return;
	}

	//int spalte_ziel = s;
	//int zeiel_ziel = z;
	vector <Moegliches_Feld> moegliche_felder_1;
	vector <Moegliches_Feld> moegliche_felder_2;
	if (!spielfeld.first_piece_selected && !spielfeld.second_piece_selected) { // erster Click
		if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn && spielfeld.Felder[s - 1][z - 1]->Get_Wahrscheinlichkeit() < 1) { // // Figur muss aufgeteilt sein
				spielfeld.first_piece_selected = true;
				spielfeld.first_piece_s = s;
				spielfeld.first_piece_z = z;
				cout << spielfeld.Felder[s - 1][z - 1]->Get_Name() << " gewaehlt" << endl;
				spielfeld.piece_selected = true;
				spielfeld.selected_piece_s = s;
				spielfeld.selected_piece_z = z;
			}
		}
	}
	else if (spielfeld.first_piece_selected && !spielfeld.second_piece_selected) { // zweiter Click
		if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
			if (spielfeld.Felder[s - 1][z - 1]->Get_Farbe() == spielfeld.whites_turn ) { // Bauer darf keine Quantenzuege ausfuehren
				for (int i = 0; i < spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece().size(); i++) { // geht alle same pieces durch
					if (spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece()[i] == spielfeld.Felder[spielfeld.first_piece_s - 1][spielfeld.first_piece_z - 1]) { // muss "gleiche Figur sein" wie andere ausgewaehlet figur
						spielfeld.second_piece_selected = true;
						spielfeld.second_piece_s = s;
						spielfeld.second_piece_z = z;
						cout << spielfeld.Felder[s - 1][z - 1]->Get_Name() << " gewaehlt" << endl;
						// Damit die moeglichen Felder der zweiten Figur angezeigt werden
						spielfeld.piece_selected = true;
						spielfeld.selected_piece_s = s;
						spielfeld.selected_piece_z = z;
						break;
					}
				}
			}
		}
		if (!spielfeld.second_piece_selected) {
			spielfeld.first_piece_selected = false;
			spielfeld.piece_selected = false;
		}
	}
	else if (spielfeld.first_piece_selected && spielfeld.second_piece_selected) { // Dritter Click
		// Erste Figur
		int sa1 = spielfeld.first_piece_s;
		int za1 = spielfeld.first_piece_z;
		// Zweite Figur
		int sa2 = spielfeld.second_piece_s;
		int za2 = spielfeld.second_piece_z;

		spielfeld.Felder[sa1 - 1][za1 - 1]->Set_Moegliche_Felder(spielfeld);
		spielfeld.Felder[sa2 - 1][za2 - 1]->Set_Moegliche_Felder(spielfeld);
		
		moegliche_felder_1 = spielfeld.Felder[sa1 - 1][za1 - 1]->Get_Moegliche_Felder();
		moegliche_felder_2 = spielfeld.Felder[sa2 - 1][za2 - 1]->Get_Moegliche_Felder();

		for (int i = 0; i < moegliche_felder_1.size(); i++) {
			if (moegliche_felder_1[i].spalte == s && moegliche_felder_1[i].zeile == z, moegliche_felder_1[i].wahrscheinlichkeit == 1.0){
				for (int j = 0; j < moegliche_felder_2.size(); j++) {
					if (moegliche_felder_2[j].spalte == s && moegliche_felder_2[j].zeile == z, moegliche_felder_2[i].wahrscheinlichkeit == 1.0) { // ausgewaehltes Feld muss von beiden Figuren ein moegliches Feld sein
						Merge_Move(s, z, spielfeld);
						break;
					}
				}
				break;
			}
		}
		spielfeld.first_piece_selected = false;
		spielfeld.second_piece_selected = false;
		spielfeld.piece_selected = false;
	}
}

void Merge_Move(int sz, int zz, Brett& spielfeld) {

	int sa = spielfeld.first_piece_s;
	int za = spielfeld.first_piece_z;

	int s2 = spielfeld.second_piece_s;
	int z2 = spielfeld.second_piece_z;

	if (spielfeld.Felder[sz - 1][zz - 1] == nullptr) {

		spielfeld.Felder[sz - 1][zz - 1] = spielfeld.Felder[sa - 1][za - 1];
		spielfeld.Felder[sz - 1][zz - 1]->Set_Spalte(sz);
		spielfeld.Felder[sz - 1][zz - 1]->Set_Zeile(zz);
		spielfeld.Felder[sz - 1][zz - 1]->Set_Gezogen(true);
		spielfeld.Felder[sz - 1][zz - 1]->Set_Wahrscheinlichkeit(spielfeld.Felder[sa - 1][za - 1]->Get_Wahrscheinlichkeit() + spielfeld.Felder[s2 - 1][z2 - 1]->Get_Wahrscheinlichkeit());
		spielfeld.Felder[sa - 1][za - 1] = nullptr;
		cout << "Figuren gemerged" << endl;

		// Zweite ausgewaehlte Figur auf geschlagen setzen und verschwinden lassen
		spielfeld.Felder[s2 - 1][z2 - 1]->Set_Geschlagen(true);
		spielfeld.Felder[s2 - 1][z2 - 1] = nullptr;

		spielfeld.whites_turn = !spielfeld.whites_turn;
	}
	
}

void No_Move(Brett& spielfeld) {
	spielfeld.piece_selected = false;

	spielfeld.first_field_selected = false;
	spielfeld.second_field_selected = false;

	spielfeld.first_piece_selected = false;
	spielfeld.second_piece_selected = false;

}

bool Messung(int s, int z,Brett& spielfeld) {

	if (!Zufall(spielfeld.Felder[s - 1][z - 1]->Get_Wahrscheinlichkeit())) { // ausgewahlte Figur ist nicht die echte
		for (int i = 0; i < spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece().size(); i++) { // alle anderen moeglichen durchegehn
			if (spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece()[i] != nullptr) {
				if (Zufall(spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece()[i]->Get_Wahrscheinlichkeit()) || (i == spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece().size() - 1)) {
					// wahrscheinlichkeit tritt ein, oder letzte figur erreicht
					int s2 = spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece()[i]->Get_Spalte();
					int z2 = spielfeld.Felder[s - 1][z - 1]->Get_Same_Piece()[i]->Get_Zeile();
					Kollpas(s2, z2, spielfeld);
					break;
				}
			}
		}
	
	}
	else {
		Kollpas(s, z, spielfeld);
		
		return true;
	}
	return false;
}

void Kollpas(int s, int z, Brett& spielfeld) {

	Figuren* echteFigur = spielfeld.Felder[s - 1][z - 1];

	for (int i = 0; i < echteFigur->Get_Same_Piece().size(); i++) {
		Figuren* F = echteFigur->Get_Same_Piece()[i];
		if (F != echteFigur && F != nullptr) {
			int sl = F->Get_Spalte();
			int zl = F->Get_Zeile();
			Feld_Leeren(sl, zl, spielfeld);
			F->Get_Same_Piece()[i] = nullptr;
		}
	}
	echteFigur->Set_Wahrscheinlichkeit(1.0);
	echteFigur->Clear_Same_Piece();	
}


void Feld_Leeren(int s, int z, Brett& spielfeld) {
	if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
		spielfeld.Felder[s - 1][z - 1]->Set_Geschlagen(true);
		spielfeld.Felder[s - 1][z - 1] = nullptr;
	}
}
bool Zufall(double p) {
	   

	// Sicherstellen, dass die Wahrscheinlichkeit im gültigen Bereich liegt
	p = max(0.0, min(1.0, p));

	// Zufallszahlengenerator initialisieren
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<> dis(0.0, 1.0);

	// Zufälligen Wert zwischen 0 und 1 generieren
	double randomValue = dis(gen);

	// True zurückgeben, wenn der Zufallswert kleiner als die Wahrscheinlichkeit ist
	return randomValue < p;

}


void Messung_Fuer_Verschraenkung(Figuren* F,  Brett& spielfeld) {
	bool kollpas_erfolgt = false;
	while (!kollpas_erfolgt) {
		for (int i = 0; i < F->Get_Same_Piece().size(); i++) { // alle anderen moeglichen durchegehn
			if (F->Get_Same_Piece()[i] != nullptr) {
				if (Zufall(F->Get_Same_Piece()[i]->Get_Wahrscheinlichkeit())) {
					// wahrscheinlichkeit tritt ein, oder letzte figur erreicht
					int s2 = F->Get_Same_Piece()[i]->Get_Spalte();
					int z2 = F->Get_Same_Piece()[i]->Get_Zeile();
					Kollpas(s2, z2, spielfeld);
					F->Get_Same_Piece()[i]->Set_Wahrscheinlichkeit(1.0);
					kollpas_erfolgt = true;
					break;
				}
			}
		}
		cout << "Dauerschleife" << endl;
	}
}
void Kollpas_Verschraenkung(int s, int z, Brett& spielfeld) {

	Figuren* echteFigur = spielfeld.Felder[s - 1][z - 1];
	for (int i = 0; i < echteFigur->Get_Connected_Piece().size(); i++) {
		Figuren* F = echteFigur->Get_Connected_Piece()[i];
		if (F != echteFigur && F != nullptr) {
			int sl = F->Get_Spalte();
			int zl = F->Get_Zeile();
			Messung_Fuer_Verschraenkung(F, spielfeld);
			Feld_Leeren(sl, zl, spielfeld);
		}
	}	
	Feld_Leeren(s, z, spielfeld);
}

void Check_For_Kollaps_Verschraenkung( Brett& spielfeld) {
	for (int s = 1; s < 9; s++) {
		for (int z = 1; z < 9; z++) {
			if (spielfeld.Felder[s - 1][z - 1] != nullptr) {
				for (int i = 0; i < spielfeld.Felder[s - 1][z - 1]->Get_Connected_Piece().size(); i++) {
					if (spielfeld.Felder[s - 1][z - 1]->Get_Connected_Piece()[i] == nullptr) {
						
						Kollpas_Verschraenkung(s, z, spielfeld);
						break;
					}
					else if (spielfeld.Felder[s - 1][z - 1]->Get_Connected_Piece()[i]->Get_Geschlagen()) {
						Kollpas_Verschraenkung(s, z, spielfeld);
						break;
					}
				}
			}
		}
	}
	cout << "Auf Kollaps verschraenkung geprueft" << endl;
	
}

void Create_Bauer(int su, int zu, int sn, int zn, float p, vector <Bauer>& bauern, Brett& spielfeld) {
	
	Bauer b;
	b.Set_Geschlagen(false);
	b.Set_Gezogen(false);
	b.Set_Name('b');
	b.Set_Spalte(sn);
	b.Set_Zeile(zn);
	b.Set_Farbe(spielfeld.Felder[su - 1][zu - 1]->Get_Farbe());
	b.Set_Wahrscheinlichkeit(p);
	b.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	b.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());

	for (int i = 0; i < bauern.size(); i++) {
		if (!bauern[i].Get_Geschlagen()) {
			for (int j = 0; j < bauern[i].Get_Same_Piece().size(); j++) {
				if (bauern[i].Get_Same_Piece()[j] != nullptr) {
					int s = bauern[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = bauern[i].Get_Same_Piece()[j]->Get_Zeile();

					bauern[i].Add_Same_Piece_S(s);
					bauern[i].Add_Same_Piece_Z(z);
				}
			}
			bauern[i].Clear_Same_Piece();	

			/*bauern[i].Clear_Connected_Piece_S();
			bauern[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < bauern[i].Get_Connected_Piece().size(); j++) {
				if (bauern[i].Get_Connected_Piece()[j] != nullptr) {
					
					int s = bauern[i].Get_Spalte();
					int z = bauern[i].Get_Zeile();

					bauern[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					bauern[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
				}
			}
			bauern[i].Clear_Connected_Piece();*/
		}
	}

	bauern.push_back(b);
	Set_Bauer_Pointer(bauern, spielfeld);
}
void Create_Springer(int su, int zu, int sn, int zn, float p,vector <Springer>& springer, Brett& spielfeld) {

	Springer S;
	S.Set_Geschlagen(false);
	S.Set_Gezogen(true);
	S.Set_Name('S');
	S.Set_Spalte(sn);
	S.Set_Zeile(zn);
	S.Set_Farbe(spielfeld.Felder[su -1][zu - 1]->Get_Farbe());
	S.Set_Wahrscheinlichkeit(p);
	S.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	S.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());
	
	for (int i = 0; i < springer.size(); i++) {
		if (!springer[i].Get_Geschlagen()) {
			for (int j = 0; j < springer[i].Get_Same_Piece().size(); j++) {
				if (springer[i].Get_Same_Piece()[j] != nullptr) {
					int s = springer[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = springer[i].Get_Same_Piece()[j]->Get_Zeile();

					springer[i].Add_Same_Piece_S(s);
					springer[i].Add_Same_Piece_Z(z);
				}
			}
			springer[i].Clear_Same_Piece();

			/*springer[i].Clear_Connected_Piece_S();
			springer[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < springer[i].Get_Connected_Piece().size(); j++) {
				if (springer[i].Get_Connected_Piece()[j] != nullptr) {

					int s = springer[i].Get_Spalte();
					int z = springer[i].Get_Zeile();

					springer[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					springer[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
				}
			}
			springer[i].Clear_Connected_Piece();*/
		}
	}

	springer.push_back(S);
	Set_Springer_Pointer(springer, spielfeld);
}
void Create_Laeufer(int su, int zu, int sn, int zn, float p, vector <Laeufer>& laeufer, Brett& spielfeld) {

	Laeufer L;
	L.Set_Geschlagen(false);
	L.Set_Gezogen(false);
	L.Set_Name('L');
	L.Set_Spalte(sn);
	L.Set_Zeile(zn);
	L.Set_Farbe(spielfeld.Felder[su - 1][zu - 1]->Get_Farbe());
	L.Set_Wahrscheinlichkeit(p);
	L.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	L.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());

	for (int i = 0; i < laeufer.size(); i++) {
		if (!laeufer[i].Get_Geschlagen()) {
			for (int j = 0; j < laeufer[i].Get_Same_Piece().size(); j++) {
				if (laeufer[i].Get_Same_Piece()[j] != nullptr) {
					int s = laeufer[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = laeufer[i].Get_Same_Piece()[j]->Get_Zeile();

					laeufer[i].Add_Same_Piece_S(s);
					laeufer[i].Add_Same_Piece_Z(z);
				}
			}
			laeufer[i].Clear_Same_Piece();

			/*laeufer[i].Clear_Connected_Piece_S();
			laeufer[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < laeufer[i].Get_Connected_Piece().size(); j++) {
				if (laeufer[i].Get_Connected_Piece()[j] != nullptr) {

					int s = laeufer[i].Get_Spalte();
					int z = laeufer[i].Get_Zeile();

					laeufer[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					laeufer[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
				}
			}
			laeufer[i].Clear_Connected_Piece();*/
		}
	}

	laeufer.push_back(L);
	Set_Laeufer_Pointer(laeufer, spielfeld);
}
void Create_Turm(int su, int zu, int sn, int zn, float p, vector <Turm>& tuerme, Brett& spielfeld) {

	Turm T;
	T.Set_Geschlagen(false);
	T.Set_Gezogen(false);
	T.Set_Name('T');
	T.Set_Spalte(sn);
	T.Set_Zeile(zn);
	T.Set_Farbe(spielfeld.Felder[su -1][zu - 1]->Get_Farbe());
	T.Set_Wahrscheinlichkeit(p);
	T.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	T.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());

	for (int i = 0; i < tuerme.size(); i++) {
		if (!tuerme[i].Get_Geschlagen()) {
			for (int j = 0; j < tuerme[i].Get_Same_Piece().size(); j++) {
				if (tuerme[i].Get_Same_Piece()[j] != nullptr) {
					int s = tuerme[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = tuerme[i].Get_Same_Piece()[j]->Get_Zeile();

					tuerme[i].Add_Same_Piece_S(s);
					tuerme[i].Add_Same_Piece_Z(z);
				}
			}
			tuerme[i].Clear_Same_Piece();


			/*tuerme[i].Clear_Connected_Piece_S();
			tuerme[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < tuerme[i].Get_Connected_Piece().size(); j++) {
				if (tuerme[i].Get_Connected_Piece()[j] != nullptr) {

					int s = tuerme[i].Get_Spalte();
					int z = tuerme[i].Get_Zeile();

					tuerme[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					tuerme[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
				}
			}
			tuerme[i].Clear_Connected_Piece();*/
		}
	}

	tuerme.push_back(T);
	Set_Tuerme_Pointer(tuerme, spielfeld);
}
void Create_Dame(int su, int zu, int sn, int zn, float p, vector <Dame>& damen, Brett& spielfeld) {

	Dame D;
	D.Set_Geschlagen(false);
	D.Set_Gezogen(false);
	D.Set_Name('D');
	D.Set_Spalte(sn);
	D.Set_Zeile(zn);
	D.Set_Farbe(spielfeld.Felder[su - 1][zu - 1]->Get_Farbe());
	D.Set_Wahrscheinlichkeit(p);
	D.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	D.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());

	for (int i = 0; i < damen.size(); i++) {
		if (!damen[i].Get_Geschlagen()) {
			for (int j = 0; j < damen[i].Get_Same_Piece().size(); j++) {
				if (damen[i].Get_Same_Piece()[j] != nullptr) {
					int s = damen[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = damen[i].Get_Same_Piece()[j]->Get_Zeile();

					damen[i].Add_Same_Piece_S(s);
					damen[i].Add_Same_Piece_Z(z);
				}
			}
			damen[i].Clear_Same_Piece();

		/*	damen[i].Clear_Connected_Piece_S();
			damen[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < damen[i].Get_Connected_Piece().size(); j++) {
				if (damen[i].Get_Connected_Piece()[j] != nullptr) {

					int s = damen[i].Get_Spalte();
					int z = damen[i].Get_Zeile();

					damen[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					damen[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
				}
			}
			damen[i].Clear_Connected_Piece();*/
		}
	}

	damen.push_back(D);
	Set_Damen_Pointer(damen, spielfeld);
}
void Create_Koenig(int su, int zu, int sn, int zn, float p, vector <Koenig>& koenige, Brett& spielfeld) {

	Koenig K;
	K.Set_Geschlagen(false);
	K.Set_Gezogen(false);
	K.Set_Name('K');
	K.Set_Spalte(sn);
	K.Set_Zeile(zn);
	K.Set_Farbe(spielfeld.Felder[su - 1][zu - 1]->Get_Farbe());
	K.Set_Wahrscheinlichkeit(p);
	K.Set_Dateipfad(spielfeld.Felder[su - 1][zu - 1]->Get_Dateipfad());
	K.Set_Texture(spielfeld.Felder[su - 1][zu - 1]->Get_Texture());

	for (int i = 0; i < koenige.size(); i++) {
		if (!koenige[i].Get_Geschlagen()) {
			for (int j = 0; j < koenige[i].Get_Same_Piece().size(); j++) {
				if (koenige[i].Get_Same_Piece()[j] != nullptr) {
					int s = koenige[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = koenige[i].Get_Same_Piece()[j]->Get_Zeile();

					koenige[i].Add_Same_Piece_S(s);
					koenige[i].Add_Same_Piece_Z(z);
				}
			}
			koenige[i].Clear_Same_Piece();


			/*koenige[i].Clear_Connected_Piece_S();
			koenige[i].Clear_Connected_Piece_Z();
			for (int j = 0; j < koenige[i].Get_Connected_Piece().size(); j++) {
				if (koenige[i].Get_Connected_Piece()[j] != nullptr) {
					if (koenige[i].Get_Connected_Piece()[j] == &koenige[i]) {
						int s = koenige[i].Get_Spalte();
						int z = koenige[i].Get_Zeile();

						koenige[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
						koenige[i].Get_Connected_Piece()[j]->Add_Connected_Piece_S(s);
					}
				}
			}
			koenige[i].Clear_Connected_Piece();*/
		}
	}
	koenige.push_back(K);
	Set_Koenige_Pointer(koenige, spielfeld);
}



void Set_Bauer_Pointer(vector <Bauer>& bauern, Brett& spielfeld) {
	for (int i = 0; i < bauern.size(); i++) {
		if (!bauern[i].Get_Geschlagen()) {
			spielfeld.Felder[bauern[i].Get_Spalte() - 1][bauern[i].Get_Zeile() - 1] = &bauern[i];
		}
	}
	for (int i = 0; i < bauern.size(); i++) {
		if (!bauern[i].Get_Geschlagen()) {
			for (int j = 0; j < bauern[i].Get_Same_Piece_S().size(); j++) {
				int s = bauern[i].Get_Same_Piece_S()[j];
				int z = bauern[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				bauern[i].Add_Same_Piece(F);

			}
			bauern[i].Clear_Same_Piece_S();
			bauern[i].Clear_Same_Piece_Z();
			for (int j = 0; j < bauern[i].Get_Connected_Piece().size(); j++) {
				if (bauern[i].Get_Connected_Piece_S()[j] == bauern[i].Get_Spalte()) {
					if (bauern[i].Get_Connected_Piece_Z()[j] == bauern[i].Get_Zeile()) {
						bauern[i].Replace_Connected_Piece(j, bauern[i]);
					}
				}
			}
		}
		
	}
}
void Set_Springer_Pointer(vector <Springer>& springer, Brett& spielfeld) {

	for (int i = 0; i < springer.size(); i++) {
		if (!springer[i].Get_Geschlagen()) {	
			spielfeld.Felder[springer[i].Get_Spalte() - 1][springer[i].Get_Zeile() - 1] = &springer[i]; // neue Pointer werden erzeugt
		}   
	}
	// same piece wieder herstellen
	for (int i = 0; i < springer.size(); i++) {
		if (!springer[i].Get_Geschlagen()) {
			for (int j = 0; j < springer[i].Get_Same_Piece_S().size(); j++) {
				int s = springer[i].Get_Same_Piece_S()[j];
				int z = springer[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				springer[i].Add_Same_Piece(F);

			}
			springer[i].Clear_Same_Piece_S();
			springer[i].Clear_Same_Piece_Z();

			for (int j = 0; j < springer[i].Get_Connected_Piece().size(); j++) {
				if (springer[i].Get_Connected_Piece_S()[j] == springer[i].Get_Spalte()) {
					if (springer[i].Get_Connected_Piece_Z()[j] == springer[i].Get_Zeile()) {
						springer[i].Replace_Connected_Piece(j, springer[i]);
					}
				}
			}
		}
	}
}
void Set_Laeufer_Pointer(vector <Laeufer>& laeufer, Brett& spielfeld) {
	for (int i = 0; i < laeufer.size(); i++) {
		if (!laeufer[i].Get_Geschlagen()) {
			spielfeld.Felder[laeufer[i].Get_Spalte() - 1][laeufer[i].Get_Zeile() - 1] = &laeufer[i];
		}
	}
	for (int i = 0; i < laeufer.size(); i++) {
		if (!laeufer[i].Get_Geschlagen()) {
			for (int j = 0; j < laeufer[i].Get_Same_Piece_S().size(); j++) {
				int s = laeufer[i].Get_Same_Piece_S()[j];
				int z = laeufer[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				laeufer[i].Add_Same_Piece(F);

			}
			laeufer[i].Clear_Same_Piece_S();
			laeufer[i].Clear_Same_Piece_Z();

			for (int j = 0; j < laeufer[i].Get_Connected_Piece().size(); j++) {
				if (laeufer[i].Get_Connected_Piece_S()[j] == laeufer[i].Get_Spalte()) {
					if (laeufer[i].Get_Connected_Piece_Z()[j] == laeufer[i].Get_Zeile()) {
						laeufer[i].Replace_Connected_Piece(j, laeufer[i]);
					}
				}
			}
		}
	}
}
void Set_Tuerme_Pointer(vector <Turm>& tuerme, Brett& spielfeld) {
	for (int i = 0; i < tuerme.size(); i++) {
		if (!tuerme[i].Get_Geschlagen()) {
			spielfeld.Felder[tuerme[i].Get_Spalte() - 1][tuerme[i].Get_Zeile() - 1] = &tuerme[i];
		}
	}
	for (int i = 0; i < tuerme.size(); i++) {
		if (!tuerme[i].Get_Geschlagen()) {
			for (int j = 0; j < tuerme[i].Get_Same_Piece_S().size(); j++) {
				int s = tuerme[i].Get_Same_Piece_S()[j];
				int z = tuerme[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				tuerme[i].Add_Same_Piece(F);

			}
			tuerme[i].Clear_Same_Piece_S();
			tuerme[i].Clear_Same_Piece_Z();

			for (int j = 0; j < tuerme[i].Get_Connected_Piece().size(); j++) {
				if (tuerme[i].Get_Connected_Piece_S()[j] == tuerme[i].Get_Spalte()) {
					if (tuerme[i].Get_Connected_Piece_Z()[j] == tuerme[i].Get_Zeile()) {
						tuerme[i].Replace_Connected_Piece(j, tuerme[i]);
					}
				}
			}
		}
	}
}
void Set_Damen_Pointer(vector <Dame>& damen, Brett& spielfeld) {
	for (int i = 0; i < damen.size(); i++) {
		if (!damen[i].Get_Geschlagen()) {
			spielfeld.Felder[damen[i].Get_Spalte() - 1][damen[i].Get_Zeile() - 1] = &damen[i];
		}
	}
	for (int i = 0; i < damen.size(); i++) {
		if (!damen[i].Get_Geschlagen()) {
			for (int j = 0; j < damen[i].Get_Same_Piece_S().size(); j++) {
				int s = damen[i].Get_Same_Piece_S()[j];
				int z = damen[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				damen[i].Add_Same_Piece(F);

			}
			damen[i].Clear_Same_Piece_S();
			damen[i].Clear_Same_Piece_Z();

			for (int j = 0; j < damen[i].Get_Connected_Piece().size(); j++) {
					if (damen[i].Get_Connected_Piece_S()[j] == damen[i].Get_Spalte()) {
						if (damen[i].Get_Connected_Piece_Z()[j] == damen[i].Get_Zeile()) {
							damen[i].Replace_Connected_Piece(j, damen[i]);
						}
					}
			}
		}
	}
}
void Set_Koenige_Pointer(vector <Koenig>& koenige, Brett& spielfeld) {
	for (int i = 0; i < koenige.size(); i++) {
		if (!koenige[i].Get_Geschlagen()) {
			spielfeld.Felder[koenige[i].Get_Spalte() - 1][koenige[i].Get_Zeile() - 1] = &koenige[i];
		}
	}
	for (int i = 0; i < koenige.size(); i++) {
		if (!koenige[i].Get_Geschlagen()) {
			for (int j = 0; j < koenige[i].Get_Same_Piece_S().size(); j++) {
				int s = koenige[i].Get_Same_Piece_S()[j];
				int z = koenige[i].Get_Same_Piece_Z()[j];
				Figuren* F = spielfeld.Felder[s - 1][z - 1];
				koenige[i].Add_Same_Piece(F);

			}
			koenige[i].Clear_Same_Piece_S();
			koenige[i].Clear_Same_Piece_Z();

			for (int j = 0; j < koenige[i].Get_Connected_Piece().size(); j++) {
				if (koenige[i].Get_Connected_Piece_S()[j] == koenige[i].Get_Spalte()) {
					if (koenige[i].Get_Connected_Piece_Z()[j] == koenige[i].Get_Zeile()) {
						koenige[i].Replace_Connected_Piece(j, koenige[i]);
					}
				}
			}
		}
	}
}




// Rochade
bool King_selected(int s, int z, Brett& spielfeld) {

	if (s == 5 && (z == 8 || z == 1)) {
		if (!spielfeld.Felder[s - 1][z - 1]->Get_Gezogen()) {
			return true;
		}
	}
	else {
		return false;
	}
}
bool Check_For_scw(Brett& spielfeld) {
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
bool Check_For_lcw(Brett& spielfeld) {
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
bool Check_For_scb(Brett& spielfeld) {
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
bool Check_For_lcb(Brett& spielfeld) {
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
void Check_Castle_Selected(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld) {
	bool scw = Check_For_scw(spielfeld);
	bool lcw = Check_For_lcw(spielfeld);
	bool scb = Check_For_scb(spielfeld);
	bool lcb = Check_For_lcb(spielfeld);
	
	if (!scw && !lcb && !spielfeld.whites_turn) {
		return ;
	}
	if (spielfeld.whites_turn) {
		if (!scw && !lcw ) {
			return;
		}
		if (scw && !lcw ) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(8, 1, 6, 1, spielfeld);
				return;
			}
		}
		if (lcw && !scw ) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(1, 1, 4, 1, spielfeld);
				return;
			}
		}

		if (scw && lcw ) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(1, 1, 4, 1, spielfeld); // lc
				return;
			}
		}
		if (scw && lcw ) {
			if (i == moegliche_felder.size() - 2) {
				Ziehen(8, 1, 6, 1, spielfeld); // sc
				return;
			}
		}
	}
	if (!spielfeld.whites_turn) {
		if (!scb && !lcb) {
			return;
		}
		if (scb && !lcb) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(8, 8, 6, 8, spielfeld);
				return;
			}
		}
		if (lcb && !scb) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(1, 8, 4, 8, spielfeld);
				return;
			}
		}

		if (scb && lcb) {
			if (i == moegliche_felder.size() - 1) {
				Ziehen(1, 8, 4, 8, spielfeld); // lc
				return;
			}
		}
		if (scb && lcb) {
			if (i == moegliche_felder.size() - 2) {
				Ziehen(8, 8, 6, 8, spielfeld); // sc
				return;
			}
		}
	}

}

// Pruefen auf Schachmatt
void Check_for_Mate(Brett& spielfeld) {
	int bk = 0;
	int wk = 0;
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			if (spielfeld.Felder[i][j] != nullptr) {
				if (spielfeld.Felder[i][j]->Get_Name() == 'K') {
					if (spielfeld.Felder[i][j]->Get_Farbe()) {
						wk++;
					}
					else {
						bk++;
					}
				}
			}
		}
	}
	if (wk == 0 || bk == 0) {
		spielfeld.schachmatt = true;
	}
}

// Umwandlung Bauer in Dame
bool Ceck_For_Promotion(Brett& spielfeld, vector <Dame>& damen){

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			if (spielfeld.Felder[i][j] != nullptr) {
				if (spielfeld.Felder[i][j]->Get_Name() == 'b') {
					int z = spielfeld.Felder[i][j]->Get_Zeile();
					if (z == 1 || z == 8) {
						Messung(i + 1, z, spielfeld);
						Make_Queen(i + 1, z, spielfeld, damen);
						return true;
					}
				}
			}
		}
	}
	return false;
}
void Make_Queen(int s, int z, Brett& spielfeld, vector <Dame>& damen) {
	
	Dame D;

	D.Set_Zeile(z);
	D.Set_Spalte(s);
	D.Set_Farbe(spielfeld.Felder[s - 1][z - 1]->Get_Farbe());
	D.Set_Wahrscheinlichkeit(spielfeld.Felder[s - 1][z - 1]->Get_Wahrscheinlichkeit());
	D.Set_Gezogen(true);
	D.Set_Name('D');
	D.Set_Geschlagen(false);

	if (D.Get_Farbe()) {
		D.Set_Dateipfad("Png/w_queen_2x.png");
	}
	else {
		D.Set_Dateipfad("Png/b_queen_2x.png");
	}

	for (int i = 0; i < damen.size(); i++) {
		if (!damen[i].Get_Geschlagen()) {
			for (int j = 0; j < damen[i].Get_Same_Piece().size(); j++) {
				if (damen[i].Get_Same_Piece()[j] != nullptr) {
					int s = damen[i].Get_Same_Piece()[j]->Get_Spalte();
					int z = damen[i].Get_Same_Piece()[j]->Get_Zeile();

					damen[i].Add_Same_Piece_S(s);
					damen[i].Add_Same_Piece_Z(z);
				}
			}
			damen[i].Clear_Same_Piece();
		}
	}
	damen.push_back(D);

	Set_Damen_Pointer(damen, spielfeld);
}


// En Passant
void Check_For_Double_Pawn(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld) {

	int s = spielfeld.selected_piece_s;
	int z = spielfeld.selected_piece_z;

	
	
	if (moegliche_felder[i].wahrscheinlichkeit == 1.0) {//der ziehende Bauer darf mit nix verschreankt sein


		if (spielfeld.Felder[s - 1][z - 1]->Get_Name() == 'b' && !spielfeld.Felder[s - 1][z - 1]->Get_Gezogen()) { // Davor in Grundstellung 
			if (i == moegliche_felder.size() - 1) {
				spielfeld.en_passant = true;
				spielfeld.en_passant_spalte = moegliche_felder[i].spalte;
				spielfeld.en_passant_zeile = moegliche_felder[i].zeile;

			}
			else if (i == moegliche_felder.size() - 2 && spielfeld.en_passant) {
				spielfeld.en_passant = true;
				spielfeld.en_passant_spalte = moegliche_felder[i].spalte;
				spielfeld.en_passant_zeile = moegliche_felder[i].zeile;
			}
			else {
				spielfeld.en_passant = false;
				spielfeld.en_passant_spalte = -2;
			}
		}
	}
	else {
		spielfeld.en_passant = false;
		spielfeld.en_passant_spalte = -2;
	}

}
void Check_For_En_Passant(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld) {
	int s = spielfeld.selected_piece_s;
	int z = spielfeld.selected_piece_z;
	if (spielfeld.Felder[s - 1][z - 1]->Get_Wahrscheinlichkeit() != 1.0) {
		// Nur echte Bauern dürfen En Passant schlagen
		return;
	}
	if (i == moegliche_felder.size() - 1 && spielfeld.en_passant && spielfeld.en_passant_spalte != -2) {
		if (z == spielfeld.en_passant_zeile && (s == spielfeld.en_passant_spalte +1 || s == spielfeld.en_passant_spalte -1))

		spielfeld.Felder[spielfeld.en_passant_spalte -1][spielfeld.en_passant_zeile - 1] = nullptr;
	}
}


// Startaufstellung

void Spielfeld_Reset(Brett& spielfeld) {

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			spielfeld.Felder[i][j] = nullptr;
		}
	}
}
void Startaufstellung_Bauern(vector <Bauer>& bauern, Brett& spielfeld) {

	for (int i = 1; i < 17; i++) {
		Bauer b;
		b.Set_Geschlagen(false);
		b.Set_Gezogen(false);
		b.Set_Name('b');
		if (i <= 8) {
			b.Set_Farbe(true); // true = weiss
			b.Set_Zeile(2);
			b.Set_Spalte(i);
			b.Set_Dateipfad("Png/w_pawn_2x.png");
			//bmp:			"Png/w_pawn_2x.bmp"
		}
		else {
			b.Set_Farbe(false);
			b.Set_Zeile(7);
			b.Set_Spalte(i - 8);
			b.Set_Dateipfad("Png/b_pawn_2x.png");

		}\
			bauern.push_back(b);
	}
	for (int i = 0; i < bauern.size(); i++) {
		spielfeld.Felder[bauern[i].Get_Spalte() - 1][bauern[i].Get_Zeile() - 1] = &bauern[i];
	}
}
void Startaufstellung_Springer(vector <Springer>& springer, Brett& spielfeld) {

	for (int i = 1; i < 5; i++) {
		Springer S;
		S.Set_Geschlagen(false);
		S.Set_Name('S');
		if (i <= 2) {
			S.Set_Farbe(true);
			S.Set_Zeile(1);
			S.Set_Dateipfad("Png/w_knight_2x.png");
			switch (i)
			{
			case 1:
				S.Set_Spalte(2);
				break;

			case 2:
				S.Set_Spalte(7);
				break;
			}
		}
		else {
			S.Set_Farbe(false);
			S.Set_Zeile(8);
			S.Set_Dateipfad("Png/b_knight_2x.png");
			switch (i)
			{
			case 3:
				S.Set_Spalte(2);
				break;

			case 4:
				S.Set_Spalte(7);
				break;
			}
		}
		springer.push_back(S);
	}
	for (int i = 0; i < springer.size(); i++) {
		spielfeld.Felder[springer[i].Get_Spalte() - 1][springer[i].Get_Zeile() - 1] = &springer[i];
	}


}
void Startaufstellung_Laeufer(vector <Laeufer>& laeufer, Brett& spielfeld) {
	for (int i = 1; i < 5; i++) {
		Laeufer L;
		L.Set_Geschlagen(false);
		L.Set_Name('L');
		if (i <= 2) {
			L.Set_Farbe(true);
			L.Set_Zeile(1);
			L.Set_Dateipfad("Png/w_bishop_2x.png");
			switch (i)
			{
			case 1:
				L.Set_Spalte(3);
				break;

			case 2:
				L.Set_Spalte(6);
				break;
			}
		}
		else {
			L.Set_Farbe(false);
			L.Set_Zeile(8);
			L.Set_Dateipfad("Png/b_bishop_2x.png");

			switch (i)
			{
			case 3:
				L.Set_Spalte(3);
				break;

			case 4:
				L.Set_Spalte(6);
				break;
			}
		}
		laeufer.push_back(L);
	}
	for (int i = 0; i < laeufer.size(); i++) {
		spielfeld.Felder[laeufer[i].Get_Spalte() - 1][laeufer[i].Get_Zeile() - 1] = &laeufer[i];
	}

}
void Startaufstellung_Tuerme(vector <Turm>& tuerme, Brett& spielfeld) {
	for (int i = 1; i < 5; i++) {
		Turm T;
		T.Set_Geschlagen(false);
		T.Set_Gezogen(false);
		T.Set_Name('T');
		if (i <= 2) {
			T.Set_Farbe(true);
			T.Set_Zeile(1);
			T.Set_Dateipfad("Png/w_rook_2x.png");
			switch (i)
			{
			case 1:
				T.Set_Spalte(1);
				break;

			case 2:
				T.Set_Spalte(8);
				break;
			}
		}
		else {
			T.Set_Farbe(false);
			T.Set_Zeile(8);
			T.Set_Dateipfad("Png/b_rook_2x.png");
			switch (i)
			{
			case 3:
				T.Set_Spalte(1);
				break;

			case 4:
				T.Set_Spalte(8);
				break;
			}
		}
		tuerme.push_back(T);
	}
	for (int i = 0; i < tuerme.size(); i++) {
		spielfeld.Felder[tuerme[i].Get_Spalte() - 1][tuerme[i].Get_Zeile() - 1] = &tuerme[i];
	}
}
void Startaufstellung_Damen(vector <Dame>& damen, Brett& spielfeld) {
	for (int i = 1; i < 3; i++) {
		Dame D;
		D.Set_Geschlagen(false);
		D.Set_Spalte(4);
		D.Set_Name('D');
		if (i == 1) {
			D.Set_Farbe(true);
			D.Set_Zeile(1);
			D.Set_Dateipfad("Png/w_queen_2x.png");
		}
		else {
			D.Set_Farbe(false);
			D.Set_Zeile(8);
			D.Set_Dateipfad("Png/b_queen_2x.png");
		}
		damen.push_back(D);
	}
	for (int i = 0; i < damen.size(); i++) {
		spielfeld.Felder[damen[i].Get_Spalte() - 1][damen[i].Get_Zeile() - 1] = &damen[i];
	}
}
void Startaufstellung_Koenige(vector <Koenig>& koenige, Brett& spielfeld) {
	for (int i = 1; i < 3; i++) {
		Koenig K;
		K.Set_Geschlagen(false);
		K.Set_Gezogen(false);
		K.Set_Name('K');
		K.Set_Spalte(5);
		if (i == 1) {
			K.Set_Farbe(true);
			K.Set_Zeile(1);
			K.Set_Dateipfad("Png/w_king_2x.png");
		}
		else {
			K.Set_Farbe(false);
			K.Set_Zeile(8);
			K.Set_Dateipfad("Png/b_king_2x.png");
		}
		koenige.push_back(K);
	}
	for (int i = 0; i < koenige.size(); i++) {
		spielfeld.Felder[koenige[i].Get_Spalte() - 1][koenige[i].Get_Zeile() - 1] = &koenige[i];
	}
}




