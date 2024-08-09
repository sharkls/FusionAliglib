class CLonLatTrans
{
public:
    inline CLonLatTrans(const double p_fdLongitude, const double p_fdLatitude, const double p_fdRotateAngle)
    {
        m_fdLonggitude0 = p_fdLongitude;   //基准点经度
        m_fdLatitude0 = p_fdLatitude;      //基准点纬度
        m_fdRotateAngle = p_fdRotateAngle; //基准点北向夹角
        m_fdMathPi = 3.141592653589793;    //PI
        m_fdLongRa = 6378137.00;           //地球半径长轴
        m_fdShortRb = 6356752.3142;        //地球半径短轴
        m_fdCosA = cos(p_fdRotateAngle * m_fdMathPi / 180.0);
        m_fdSinA = sin(p_fdRotateAngle * m_fdMathPi / 180.0);
    }

    inline std::pair<double, double> lonlat2xy(const double p_fdLongitude, const double p_fdLatitude, const double p_fdRotateAngle = 0)
    {

        auto l_fdX = (p_fdLongitude - m_fdLonggitude0) * m_fdMathPi / 180.0 * m_fdLongRa * cos(m_fdLatitude0 * m_fdMathPi / 180.0);
        auto l_fdY = (p_fdLatitude - m_fdLatitude0) * m_fdMathPi / 180.0 * m_fdShortRb;
        if (p_fdRotateAngle != 0)
        {
            auto m_fdCosA = cos(p_fdRotateAngle * m_fdMathPi / 180.0);
            auto m_fdSinA = sin(p_fdRotateAngle * m_fdMathPi / 180.0);
        }

//        return std::pair<double, double>(l_fdX * m_fdCosA + l_fdY * m_fdSinA, l_fdY * m_fdCosA - l_fdX * m_fdSinA );
        return std::pair<double, double>(l_fdX * m_fdCosA - l_fdY * m_fdSinA, l_fdY * m_fdCosA + l_fdX * m_fdSinA );
    }
    inline std::pair<double, double> xy2lonlat(const double p_fdX, const double p_fdY, const double p_fdRotateAngle = 0)
    {
        if (p_fdRotateAngle != 0)
        {
            auto m_fdCosA = cos(p_fdRotateAngle * m_fdMathPi / 180.0);
            auto m_fdSinA = sin(p_fdRotateAngle * m_fdMathPi / 180.0);
        }
//        auto l_fdX = p_fdX * m_fdCosA - p_fdY * m_fdSinA;
//        auto l_fdY = p_fdY * m_fdCosA + p_fdX * m_fdSinA;
        auto l_fdX = p_fdX * m_fdCosA + p_fdY * m_fdSinA;
        auto l_fdY = p_fdY * m_fdCosA - p_fdX * m_fdSinA;

        auto p_fdLongitude = l_fdX / (m_fdLongRa * cos(m_fdLatitude0 * m_fdMathPi / 180.0))*180.0/m_fdMathPi + m_fdLonggitude0;
        auto p_fdLatitude = l_fdY / (m_fdMathPi / 180.0 * m_fdShortRb) + m_fdLatitude0;
        return std::pair<double, double>(p_fdLongitude, p_fdLatitude);
    }
    inline double getAngle()
    {
        return m_fdRotateAngle;
    }

private:
    double m_fdLonggitude0;
    double m_fdLatitude0;
    double m_fdRotateAngle;
    double m_fdMathPi;
    double m_fdLongRa;
    double m_fdShortRb;
    double m_fdCosA;
    double m_fdSinA;
};