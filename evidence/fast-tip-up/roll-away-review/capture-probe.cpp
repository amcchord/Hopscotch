#include "balance_tip_up.h"
#include <iostream>
int main(){balance_math::FastTipUp tip;balance_math::TipInput in;in.healthy=true;tip.begin(0,in);for(unsigned t=20;t<4000;t+=20){tip.step(t,.02f,in);in.left=tip.left();in.right=tip.right();in.tilt=83.f;in.left_velocity=in.right_velocity=0;if(t>2400)in.wheel_left=in.wheel_right=5.f;if(tip.ready()){std::cout << "capture_ms=" << t << " measured_wheel_rad_s=" << in.wheel_left << "\n";return 0;}}return 1;}
