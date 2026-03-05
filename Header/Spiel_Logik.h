

#include <iostream>
#include "SDL.h"
#include "main_quanten.h"
#include "Brett.h"
#include "Figuren.h"
#include "Bauer.h"
#include "Dame.h"
#include "Koenig.h"
#include "Laeufer.h"
#include "Springer.h"
#include "Turm.h"
#include <vector>

using namespace std; //notwendig vor vector

struct Brett;
struct Moegliches_Feld;

void Logik_normal(int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige);
void Ziehen(int sa, int za, int s, int z, Brett& spielfeld);
void Ziehen_Ins_Ungewisse(float p, int sa, int za, int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige);

void Logik_Split(int s, int z, Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige);
void Split_Move(Brett& spielfeld, vector <Bauer>& bauern, vector <Springer>& springer, vector <Laeufer>& laeufer, vector <Turm>& tuerme, vector <Dame>& damen, vector <Koenig>& koenige);


void Logik_Merge(int s, int z, Brett& spielfeld);
void Merge_Move(int sz, int zz, Brett& spielfeld);


void No_Move(Brett& spielfeld);
bool Messung(int s, int z, Brett& spielfeld);
bool Zufall(double p);
void Kollpas(int s, int z, Brett& spielfeld);
void Feld_Leeren(int s, int z, Brett& spielfeld);

void Messung_Fuer_Verschraenkung(Figuren* F, Brett& spielfeld);
void Kollpas_Verschraenkung(int s, int z, Brett& spielfeld);
void Check_For_Kollaps_Verschraenkung( Brett& spielfeld);

void Create_Bauer(int su, int zu, int sn, int zn, float p, vector <Bauer>& bauern, Brett& spielfeld);
void Create_Springer(int su, int zu, int sn, int zn, float p, vector <Springer>& springer, Brett& spielfeld);
void Create_Laeufer(int su, int zu, int sn, int zn, float p, vector <Laeufer>& laeufer, Brett& spielfeld);
void Create_Turm(int su, int zu, int sn, int zn, float p, vector <Turm>& tuerme, Brett& spielfeld);
void Create_Dame(int su, int zu, int sn, int zn, float p, vector <Dame>& damen, Brett& spielfeld);
void Create_Koenig(int su, int zu, int sn, int zn, float p, vector <Koenig>& koenige, Brett& spielfeld);

void Set_Bauer_Pointer(vector <Bauer>& bauern, Brett& spielfeld);
void Set_Springer_Pointer(vector <Springer>& springer, Brett& spielfeld);
void Set_Laeufer_Pointer(vector <Laeufer>& laeufer, Brett& spielfeld);
void Set_Tuerme_Pointer(vector <Turm>& tuerme, Brett& spielfeld);
void Set_Damen_Pointer(vector <Dame>& damen, Brett& spielfeld);
void Set_Koenige_Pointer(vector <Koenig>& koenige, Brett& spielfeld);

bool King_selected(int s, int z, Brett& spielfeld);
bool Check_For_scw(Brett &spielfeld);
bool Check_For_lcw(Brett& spielfeld);
bool Check_For_scb(Brett& spielfeld);
bool Check_For_lcb(Brett& spielfeld);
void Check_Castle_Selected(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld);

void Check_for_Mate(Brett& spielfeld);

bool Check_For_Promotion(Brett& spielfeld, vector <Dame>& damen);
void Make_Queen(int s, int z, Brett& spielfeld, vector <Dame>& damen);

void Check_For_Double_Pawn(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld);
void Check_For_En_Passant(int i, vector <Moegliches_Feld> moegliche_felder, Brett& spielfeld);

void Spielfeld_Reset(Brett& spielfeld);
void Startaufstellung_Bauern(vector <Bauer>& bauern, Brett& spielfeld);
void Startaufstellung_Springer( vector <Springer>& springer, Brett& spielfeld);
void Startaufstellung_Laeufer(vector <Laeufer>& laeufer, Brett& spielfeld);
void Startaufstellung_Tuerme(vector <Turm>& tuerme, Brett& spielfeld);
void Startaufstellung_Damen(vector <Dame>& damen, Brett& spielfeld);
void Startaufstellung_Koenige(vector <Koenig>& koenige, Brett& spielfeld);