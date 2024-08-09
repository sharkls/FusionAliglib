#ifndef IMAKEALG_FUSION_H
#define IMAKEALG_FUSION_H

#include "ICommonAlg.h"

class IMakeAlg{
public:
    IMakeAlg() = default;
    ~IMakeAlg() = default;

    virtual void makePre() = 0;
    virtual void makeInf() = 0;
    virtual void makePost() = 0;
    virtual void makeAlgs() = 0;
    std::vector<ICommonAlgPtr> getAlgs()
    {
        return m_vecAlgs;
    }

public:
    ICommonAlgPtr m_pPreAlg;
    ICommonAlgPtr m_pInfAlg;
    ICommonAlgPtr m_pPostAlg;
    std::vector<ICommonAlgPtr> m_vecAlgs;

};

using IMakeAlgPtr = std::shared_ptr<IMakeAlg>;

#endif //IMAKEALG_FUSION_H