#include <AlphaBeta.hpp>

bool alpha_beta::setParam(float alpha, float beta)
{
    if(alpha > 1 || alpha < 0 || beta > 1 || beta < 0)
        return false;
        
    this -> alpha = alpha;
    this -> beta = beta;

    return true;
}

void alpha_beta::zero()
{
    ypri = 0;
    ypost = 0;
    vpri = 0;
    vpost = 0;
}
  
float alpha_beta::addSample(float sample, float dt)
{
    ypri = ypost + dt*vpost;
    vpri = vpost;
    ypost = ypri + alpha*(sample - ypri);
    vpost = vpri + beta*(sample - ypri)/dt;

    return ypost;
}

float alpha_beta::getValue()
{
    return ypost;
}