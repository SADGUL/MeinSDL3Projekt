#pragma once

#include "Figuren.h"
#include "Brett.h"
#include <vector>
#include <array>
using namespace std;
class Koenig : public Figuren
{
private:
	vector <Moegliches_Feld>  moegliche_felder;
	bool Check_For_scw(Brett spielfeld);
	bool Check_For_lcw(Brett spielfeld);
	bool Check_For_scb(Brett spielfeld);
	bool Check_For_lcb(Brett spielfeld);
public:

	virtual vector <Moegliches_Feld> Get_Moegliche_Felder();
	virtual void Set_Moegliche_Felder(Brett& spielfeld);
};


