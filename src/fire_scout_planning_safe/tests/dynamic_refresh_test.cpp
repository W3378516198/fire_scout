#include "fire_scout/turn_geometry.hpp"
#include "fire_scout/bspline.hpp"
#include <cassert>
#include <iostream>
using namespace fire_scout;
int main(){
  GridConfig gc;gc.resolution=.05;gc.inflation_xy=.1;gc.inflation_z=.1;Grid free(gc);
  std::vector<Vec3> detour{{0,0,1.5},{1.5,0,1.5},{3,.5,1.5},{4,.5,1.5},{5,0,1.5},{8,0,1.5}};
  auto shortcut=refreshLocalShortcut(free,detour,detour.front(),0,1,.25,true);
  assert(!shortcut.empty()&&pathLength(shortcut)<pathLength(detour)-.12);
  assert(project(shortcut,arcLengths(shortcut),{.8,0,1.5}).error<1e-9);
  assert(distance(shortcut.back(),detour.back())<1e-9&&routeGeometryValid(free,shortcut,true));
  Grid blocked(gc);KeySet wall;
  // Block every available chord, while keeping the existing y=.5 detour free.
  for(double x=2.0;x<5.0;x+=.04)wall.insert(blocked.toKey({x,0,1.5}));
  blocked.update(wall,{});
  assert(refreshLocalShortcut(blocked,detour,detour.front(),0,1,.25,true).empty());
  assert(refreshLocalShortcut(free,detour,detour.front(),0,1,.25,true,[]{return true;}).empty());
  assert(refreshLocalShortcut(free,detour,{0,2,1.5},0,1,.25,true).empty());
  // Verify the physical finite-difference penalties' analytic gradient.
  std::vector<Vec3> q;for(int i=0;i<12;++i)q.push_back({.3*i,(i%2?.2:-.2),1.5+.02*(i%3)});
  SplineConfig c;c.obstacle_weight=0;c.max_jerk=.8;c.max_acceleration=.4;c.max_velocity=.6;
  std::vector<Vec3> grad;controlObjective(q,q,c,&grad);const double eps=1e-6;
  for(size_t i=0;i<q.size();++i)for(int axis=0;axis<3;++axis){
    auto plus=q,minus=q;
    double* a=axis==0?&plus[i].x:axis==1?&plus[i].y:&plus[i].z;
    double* b=axis==0?&minus[i].x:axis==1?&minus[i].y:&minus[i].z;
    *a+=eps;*b-=eps;
    double numeric=(controlObjective(plus,q,c)-controlObjective(minus,q,c))/(2*eps);
    double actual=axis==0?grad[i].x:axis==1?grad[i].y:grad[i].z;
    assert(std::abs(numeric-actual)<1e-4*std::max(1.,std::abs(actual)));
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
  auto optimized=optimizeSplineControls(q,c,deadline);
  assert(optimized.time_scale>1&&optimized.cost_after<=optimized.cost_before);
  assert(controlFeasibilityScale(optimized.controls,optimized.interval,c)<=1+1e-8);
  for(size_t i=0;i<q.size();++i){
    assert(distance(q[i],optimized.controls[i])<=std::min(c.max_control_offset,c.max_deviation)+1e-8);
    if(i<3||i+3>=q.size())assert(distance(q[i],optimized.controls[i])<1e-9);
  }
  auto cancelled=optimizeSplineControls(q,c,deadline,[]{return true;});
  assert(cancelled.iterations==0&&cancelled.controls.size()==q.size());
  std::cout<<"dynamic_refresh_test: PASS obstacle-gated shortcut, protected prefix, dynamic gradient, bounded refit and nominal time allocation\n";
}
