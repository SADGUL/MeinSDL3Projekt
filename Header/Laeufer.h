#pragma once

#include "Brett.h"
#include <vector>
#include <array>
using namespace std;
class Laeufer : public Figuren
{
private:
	vector <Moegliches_Feld>  moegliche_felder;
public:

	virtual vector <Moegliches_Feld> Get_Moegliche_Felder();
	virtual void Set_Moegliche_Felder(Brett& spielfeld);
};
