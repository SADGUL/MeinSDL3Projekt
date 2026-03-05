#ifndef TURM_H
#define TURM_H

#include "../Header/Figuren.h"
#include "../Header/Brett.h"
#include <vector>
#include <string>

class Turm : public Figuren {
public:
    // Inline constructor to initialize common fields
    Turm(bool farbe = true, int s = 1, int z = 1, float p = 1.0f, bool gezogen_in = false, bool geschlagen_in = false, const std::string& path = "leer")
    {
        name = 'T';
        weiss = farbe;
        spalte = s;
        zeile = z;
        wahrscheinlichkeit = p;
        gezogen = gezogen_in;
        geschlagen = geschlagen_in;
        dateipfad = path;
        texture_ptr = nullptr;
        moegliche_felder.clear();
        same_piece.clear();
        same_piece_s.clear();
        same_piece_z.clear();
        connected_piece.clear();
        connected_piece_s.clear();
        connected_piece_z.clear();
    }

    virtual ~Turm() = default;

    // Overrides implemented in Turm.cpp
    virtual std::vector<Moegliches_Feld> Get_Moegliche_Felder() override;
    virtual void Set_Moegliche_Felder(Brett& spielfeld) override;

    // Optional convenience
    virtual char Get_Name() override { return 'T'; }
};

#endif // TURM_H