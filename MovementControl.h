#ifndef MovementControl_H
#define MovementControl_H

class MovementControl: public LFDetection
{
  public:
      void FindTask(void);
      void TURN(void);
      void LineFollow(void);
      void STOP(void);
      void PID(void);
      void DummyMove(void);
      void SEARCH(void);

  private:
      int speedL;
      int speedR;
};

#endif
