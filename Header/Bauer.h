#pragma once

#include "Brett.h"
#include "Figuren.h"
#include <vector>
#include <array>
using namespace std;
class Bauer : public Figuren
{
public:	
	virtual void Set_Moegliche_Felder(Brett& spielfeld);
};

