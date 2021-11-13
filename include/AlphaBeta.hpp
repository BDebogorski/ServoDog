#pragma once

class alpha_beta
{
  private:
  
    float ypri = 0;
    float ypost = 0;
    float vpri = 0;
    float vpost = 0;
    float alpha = 0;
    float beta = 0;

  public:
  
    bool setParam(float alpha, float beta);    // set coefficients of filter
    void zero();                               // reset memory of filter
    float addSample(float sample, float dt);   // add sample
    float getValue();                          // get output value
};