#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

struct DB_Point
{
    float x, y;  // X, Y, Z position
    int clusterID = UNCLASSIFIED;  // clustered ID
};

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps, vector<DB_Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run();
    vector<int> calculateCluster(DB_Point point);
    int expandCluster(DB_Point &point, int clusterID);
    inline double calculateDistance(const DB_Point& pointCore, const DB_Point& pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    
public:
    vector<DB_Point> m_points;
    
private:    
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H
