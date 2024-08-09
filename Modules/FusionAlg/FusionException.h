#ifndef FUSIONEXCEPTION_H
#define FUSIONEXCEPTION_H

#include <iostream>
using namespace std;

class FusionException : public exception{
public:
    const char* what() const throw(){
        return "Fusion exception";
    }
};

#endif //CCOMPOSITEALGS_FUSION_H
