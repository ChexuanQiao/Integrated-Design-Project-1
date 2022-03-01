#ifndef LFDetection 
#define LFDetection

class LFDetection
{
public:
    void LFDataRead(void); // Read data from line sensors.
    void DSDataRead(void); // Read data from distance sensors.
    // void TurnDetection(void);
    void EdgeDetection(void);
    void IntersectionDetection(void); // Count the number of intersections encountered.

    void BlockDetection(void);

};

#endif
