#pragma once
#include "grid.hpp"
#include "gap_policy.hpp"
#include <chrono>
#include <string>

namespace fire_scout {
using BezierSpan = std::array<Vec3,4>;

// Open/clamped cubic B-spline. Interior knots are simple: position is C2.
// We keep the geometric curve separate from the follower's speed profile.
class CubicBSpline {
public:
  explicit CubicBSpline(std::vector<Vec3> controls) : p_(std::move(controls)) {
    if (p_.size()<4) throw std::runtime_error("Cubic B-spline needs >=4 controls");
    const int n=int(p_.size());
    knots_.assign(n+4,0.);
    for(int i=4;i<n;++i)knots_[i]=i-3;
    for(int i=n;i<n+4;++i)knots_[i]=n-3;
    for(int i=0;i<n-1;++i)
      dp_.push_back((p_[i+1]-p_[i])*(3./(knots_[i+4]-knots_[i+1])));
    dk_=std::vector<double>(knots_.begin()+1,knots_.end()-1);
    for(int i=0;i<int(dp_.size())-1;++i)
      ddp_.push_back((dp_[i+1]-dp_[i])*(2./(dk_[i+3]-dk_[i+1])));
    ddk_=std::vector<double>(dk_.begin()+1,dk_.end()-1);
  }
  Vec3 at(double t) const { return deBoor(p_,knots_,3,t); }
  Vec3 derivative(double t) const { return deBoor(dp_,dk_,2,t); }
  Vec3 secondDerivative(double t) const { return deBoor(ddp_,ddk_,1,t); }
  double end() const { return knots_.back(); }
  std::vector<BezierSpan> spans() const {
    std::vector<BezierSpan> out;
    for(int j=0;j<int(end());++j){
      Vec3 a=at(j), b=at(j+1);
      out.push_back({a,a+derivative(j)*(1./3.),b-derivative(j+1)*(1./3.),b});
    }
    return out;
  }
private:
  std::vector<Vec3> p_,dp_,ddp_;
  std::vector<double> knots_,dk_,ddk_;
  static Vec3 deBoor(const std::vector<Vec3>&p,const std::vector<double>&k,int degree,double t){
    if(t<=k[degree])return p.front();
    if(t>=k[p.size()])return p.back();
    int span=int(std::upper_bound(k.begin(),k.end(),t)-k.begin())-1;
    span=std::clamp(span,degree,int(p.size())-1);
    std::array<Vec3,4>d{};
    for(int j=0;j<=degree;++j)d[j]=p[span-degree+j];
    for(int r=1;r<=degree;++r)for(int j=degree;j>=r;--j){
      int i=span-degree+j;
      double den=k[i+degree-r+1]-k[i];
      double a=den>0?(t-k[i])/den:0;
      d[j]=d[j-1]*(1-a)+d[j]*a;
    }
    return d[degree];
  }
};
inline double splinePeakCurvature(const CubicBSpline&s,
                                 const std::function<bool()>&cancelled={}){
  double peak=0;
  const int count=std::max(12,int(s.end())*12);
  for(int i=0;i<=count;++i){
    if(cancelled && cancelled())return std::numeric_limits<double>::infinity();
    const double t=s.end()*double(i)/count;
    Vec3 v=s.derivative(t),a=s.secondDerivative(t);
    const double vv=dot(v,v);
    if(vv<1e-12)continue;
    peak=std::max(peak,std::sqrt(std::max(0.,vv*dot(a,a)-dot(v,a)*dot(v,a)))/
                       (vv*std::sqrt(vv)));
  }
  return peak;
}
inline Vec3 bezierPoint(const BezierSpan &p,double t){
  double s=1-t;
  return p[0]*(s*s*s)+p[1]*(3*s*s*t)+p[2]*(3*s*t*t)+p[3]*(t*t*t);
}
inline bool certifySpan(const Grid&g,const BezierSpan&p,bool unknown,int depth=0,
                        const std::function<bool()>&cancelled={}){
  if(cancelled && cancelled())return false;
  Vec3 lo=p[0],hi=p[0];
  for(Vec3 v:p){
    lo={std::min(lo.x,v.x),std::min(lo.y,v.y),std::min(lo.z,v.z)};
    hi={std::max(hi.x,v.x),std::max(hi.y,v.y),std::max(hi.z,v.z)};
  }
  if(g.boxClear(lo,hi,unknown))return true;
  if(depth>=12 || !g.segment(p[0],p[0],unknown) || !g.segment(p[3],p[3],unknown))return false;
  Vec3 a=(p[0]+p[1])*.5,b=(p[1]+p[2])*.5,c=(p[2]+p[3])*.5;
  Vec3 d=(a+b)*.5,e=(b+c)*.5,m=(d+e)*.5;
  return certifySpan(g,{p[0],a,d,m},unknown,depth+1,cancelled) &&
         certifySpan(g,{m,e,c,p[3]},unknown,depth+1,cancelled);
}
struct SplineConfig {
  bool enabled{true};
  double control_spacing{.40}, sample_spacing{.06};
  int attempts{4};
  // Try a broader turn where the map permits it; retain all original polyline
  // vertices as controls and bound deviation to preserve the selected corridor.
  double max_control_spacing{1.2}, max_deviation{.30};
  // EGO-inspired geometric smoothness objective. This is not a time-indexed
  // EGO trajectory: physical speed/acceleration limits remain in the follower.
  bool optimize{true};
  double jerk_weight{3.0}, bending_weight{.5}, fitting_weight{1.0};
  double max_control_offset{.20}, optimization_budget_ms{8.0};
  int optimization_iterations{96};
  double obstacle_weight{0}; // ROS enables gap-adaptive obstacle repulsion.
  GapPolicyConfig gap;
};
struct SplineResult {
  std::vector<Vec3> points;
  std::string mode;
  size_t spans{0};
  double control_spacing{0}, deviation{0};
  bool optimized{false};
  int optimization_iterations{0};
  double optimization_cost_before{0}, optimization_cost_after{0};
};
struct ControlOptimization {
  std::vector<Vec3> controls;
  int iterations{0};
  double cost_before{0}, cost_after{0};
};
// Finite differences are a geometric jerk proxy, as in EGO's smoothness cost:
// ZJU-FAST-Lab/ego-planner, src/planner/bspline_opt/src/bspline_optimizer.cpp,
// calcSmoothnessCost. This dependency-free projected solver is local to this
// package; it is not EGO's rebound/L-BFGS optimizer or time parameterization.
// Keeping a fitting term and a hard trust region avoids changing the corridor
// selected by the global planner. No obstacle cost is substituted for collision
// certification: every candidate curve still passes certifySpan below.
inline double controlObjective(const std::vector<Vec3>&q,const std::vector<Vec3>&seed,
                               const SplineConfig&cfg,std::vector<Vec3>*gradient=nullptr){
  if(gradient)gradient->assign(q.size(),{});
  double cost=0;
  for(size_t i=0;i<q.size();++i){
    Vec3 d=q[i]-seed[i];
    cost+=cfg.fitting_weight*dot(d,d);
    if(gradient)(*gradient)[i]=d*(2*cfg.fitting_weight);
  }
  for(size_t i=0;i+2<q.size();++i){
    Vec3 d=q[i+2]-q[i+1]*2+q[i];
    cost+=cfg.bending_weight*dot(d,d);
    if(gradient){
      Vec3 v=d*(2*cfg.bending_weight);
      (*gradient)[i]=(*gradient)[i]+v;
      (*gradient)[i+1]=(*gradient)[i+1]-v*2;
      (*gradient)[i+2]=(*gradient)[i+2]+v;
    }
  }
  for(size_t i=0;i+3<q.size();++i){
    Vec3 d=q[i+3]-q[i+2]*3+q[i+1]*3-q[i];
    cost+=cfg.jerk_weight*dot(d,d);
    if(gradient){
      Vec3 v=d*(2*cfg.jerk_weight);
      (*gradient)[i]=(*gradient)[i]-v;
      (*gradient)[i+1]=(*gradient)[i+1]+v*3;
      (*gradient)[i+2]=(*gradient)[i+2]-v*3;
      (*gradient)[i+3]=(*gradient)[i+3]+v;
    }
  }
  return cost;
}
inline ControlOptimization optimizeSplineControls(
    const std::vector<Vec3>&seed,const SplineConfig&cfg,
    const std::chrono::steady_clock::time_point&deadline,
    const std::function<bool()>&cancelled={},const Grid*grid=nullptr){
  ControlOptimization out{seed};
  std::vector<double> margin(seed.size(),0);
  if(grid && cfg.obstacle_weight>0 && cfg.gap.enabled)
    for(size_t i=1;i+1<seed.size();++i){
      if((cancelled&&cancelled())||std::chrono::steady_clock::now()>=deadline)break;
      margin[i]=localGap(*grid,seed[i],seed[i+1]-seed[i-1],1.,cfg.gap).extra_margin;
    }
  auto objective=[&](const std::vector<Vec3>&q,std::vector<Vec3>*gradient=nullptr){
    double value=controlObjective(q,seed,cfg,gradient);
    if(!grid)return value;
    const double eps=std::max(.01,.2*grid->cfg.resolution);
    for(size_t i=3;i+3<q.size();++i){
      if(margin[i]<=0)continue;
      double deficit=std::max(0.,margin[i]-grid->clearanceAt(q[i],margin[i]+eps));
      value+=cfg.obstacle_weight*deficit*deficit;
      if(gradient && deficit>0){
        const double dx=(grid->clearanceAt(q[i]+Vec3{eps,0,0},margin[i]+eps)-
                         grid->clearanceAt(q[i]-Vec3{eps,0,0},margin[i]+eps))/(2*eps);
        const double dy=(grid->clearanceAt(q[i]+Vec3{0,eps,0},margin[i]+eps)-
                         grid->clearanceAt(q[i]-Vec3{0,eps,0},margin[i]+eps))/(2*eps);
        (*gradient)[i]=(*gradient)[i]-Vec3{dx,dy,0}*(2*cfg.obstacle_weight*deficit);
      }
    }
    return value;
  };
  out.cost_before=out.cost_after=objective(seed);
  if(!cfg.optimize || seed.size()<7 || cfg.optimization_iterations<=0)return out;
  const double radius=std::max(0.,std::min(cfg.max_control_offset,cfg.max_deviation));
  // Spectral upper bound for the Hessian of squared first/second/third
  // differences: this fixed step makes projected gradient descent monotone.
  const double L=2*cfg.fitting_weight+32*cfg.bending_weight+128*cfg.jerk_weight+8*cfg.obstacle_weight;
  if(radius<=0 || !std::isfinite(L) || L<=0)return out;
  const double step=.95/L;
  std::vector<Vec3> gradient;
  for(int iteration=0;iteration<cfg.optimization_iterations;++iteration){
    if((cancelled && cancelled()) || std::chrono::steady_clock::now()>=deadline)break;
    objective(out.controls,&gradient);
    double change=0;
    // A clamped cubic's endpoint position, tangent and second derivative are
    // set by its first/last three controls. Preserve all of them when refining.
    for(size_t i=3;i+3<seed.size();++i){
      Vec3 next=seed[i]+limitNorm(out.controls[i]-gradient[i]*step-seed[i],radius);
      change=std::max(change,distance(next,out.controls[i]));
      out.controls[i]=next;
    }
    ++out.iterations;
    if(change<1e-6)break;
  }
  out.cost_after=objective(out.controls);
  // Retain the seed if bad numeric input or roundoff prevented an improvement.
  if(!std::isfinite(out.cost_after) || out.cost_after>=out.cost_before-1e-9){
    out.controls=seed;out.cost_after=out.cost_before;
  }
  return out;
}
inline SplineResult smoothPath(const Grid&g,const std::vector<Vec3>&route,
                               const SplineConfig&cfg={},bool unknown=true,
                               const std::function<bool()>&cancelled={}){
  if(cancelled && cancelled())return {{},"CANCELLED",0};
  if(route.empty())return {{},"EMPTY",0};
  if(!std::isfinite(cfg.control_spacing) || cfg.control_spacing<=0 ||
     !std::isfinite(cfg.sample_spacing) || cfg.sample_spacing<=0 ||
     !std::isfinite(cfg.max_control_spacing) || cfg.max_control_spacing<=0 ||
     !std::isfinite(cfg.max_deviation) || cfg.max_deviation<0 ||
     !std::isfinite(cfg.max_control_offset) || cfg.max_control_offset<0 ||
     !std::isfinite(cfg.optimization_budget_ms) || cfg.optimization_budget_ms<0 ||
     !std::isfinite(cfg.jerk_weight) || cfg.jerk_weight<0 ||
     !std::isfinite(cfg.bending_weight) || cfg.bending_weight<0 ||
     !std::isfinite(cfg.fitting_weight) || cfg.fitting_weight<0 ||
     !std::isfinite(cfg.obstacle_weight)||cfg.obstacle_weight<0||!cfg.gap.valid()||
     cfg.attempts<1 || cfg.attempts>8 || cfg.optimization_iterations<0 ||
     cfg.optimization_iterations>1000)return {{},"INVALID_CONFIG",0};
  for(Vec3 p:route)if(!finite(p))return {{},"INVALID_INPUT",0};
  if(!g.segment(route.front(),route.front(),unknown)){
    for(size_t i=1;i<route.size();++i){
      if(cancelled && cancelled())return {{},"CANCELLED",0};
      if(!g.segmentFrom(route[i-1],route[i],unknown))return {{},"INPUT_COLLISION",0};
    }
    if(route.size()<2)return {{},"INPUT_COLLISION",0};
    return {densify(route,cfg.sample_spacing),"RECOVERY_POLYLINE",0};
  }
  for(size_t i=1;i<route.size();++i){
    if(cancelled && cancelled())return {{},"CANCELLED",0};
    if(!g.segment(route[i-1],route[i],unknown))return {{},"INPUT_COLLISION",0};
  }
  if(!cfg.enabled || route.size()<2 || pathLength(route)<.05)
    return {densify(route,cfg.sample_spacing),"POLYLINE",0};
  std::vector<double> spacings;
  for(double spacing=cfg.max_control_spacing;spacing>cfg.control_spacing+1e-9;spacing*=.5)
    spacings.push_back(spacing);
  for(int attempt=0;attempt<cfg.attempts;++attempt)
    spacings.push_back(cfg.control_spacing*std::pow(.5,attempt));
  const auto route_arc=arcLengths(route);
  const auto optimize_deadline=std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double,std::milli>(cfg.optimization_budget_ms));
  for(double spacing:spacings){
    if(cancelled && cancelled())return {{},"CANCELLED",0};
    auto controls=densify(route,spacing);
    if(controls.size()<4)controls=densify(route,pathLength(route)/3.1);
    if(controls.size()<4 || controls.size()>3000)break;
    const auto optimized=optimizeSplineControls(controls,cfg,optimize_deadline,cancelled,&g);
    bool has_refinement=optimized.cost_after<optimized.cost_before-1e-9;
    if(has_refinement){
      // A lower control-polygon jerk alone can occasionally create a tighter
      // local turn, especially near fixed boundary controls. Do not trade away
      // corner speed for that algebraic improvement. This is a ranking check;
      // safety remains the full, continuous span certification below.
      const double seed_peak=splinePeakCurvature(CubicBSpline(controls),cancelled);
      const double new_peak=splinePeakCurvature(CubicBSpline(optimized.controls),cancelled);
      // A straight wall-hugging seed needs nonzero curvature to move away.
      // The follower limits curvature speed; never forbid all such repairs
      // merely because the unsafe preference seed has zero curvature.
      has_refinement=new_peak<=(cfg.obstacle_weight>0?std::max(2.5,seed_peak*1.05):seed_peak*1.01)+1e-8;
    }
    // A failed optimization is only a rejected candidate. Always retain the
    // original curve as the next candidate before reducing its control spacing.
    for(int candidate=0;candidate<(has_refinement?2:1);++candidate){
    const bool refined=has_refinement && candidate==0;
    CubicBSpline spline(refined?optimized.controls:controls);
    auto spans=spline.spans();
    bool valid=true;
    double deviation=0;
    std::vector<Vec3> out{route.front()};
    for(const auto&span:spans){
      if(!certifySpan(g,span,unknown,0,cancelled)){valid=false;break;}
      double len=distance(span[0],span[1])+distance(span[1],span[2])+distance(span[2],span[3]);
      int count=std::max(2,int(std::ceil(3*len/cfg.sample_spacing)));
      for(int i=1;i<=count;++i){
        if(cancelled && cancelled())return {{},"CANCELLED",0};
        Vec3 p=bezierPoint(span,double(i)/count);
        if(!g.segment(out.back(),p,unknown)){valid=false;break;}
        // This sampled bound is a route-preference constraint only. Physical
        // collision safety still comes from the complete certified span hull.
        auto projection=project(route,route_arc,p);
        deviation=std::max(deviation,projection.error);
        if(cfg.obstacle_weight>0){
          const double seed_clearance=g.clearanceAt(atArc(route,route_arc,projection.s));
          if(seed_clearance>.08 && g.clearanceAt(p)<.60*seed_clearance){valid=false;break;}
        }
        if(deviation>cfg.max_deviation+1e-9){valid=false;break;}
        if(distance(out.back(),p)>1e-7)out.push_back(p);
      }
      if(!valid)break;
    }
    if(cancelled && cancelled())return {{},"CANCELLED",0};
    if(valid)return {out,"BSPLINE",spans.size(),spacing,deviation,refined,
                     refined?optimized.iterations:0,
                     refined?optimized.cost_before:0,refined?optimized.cost_after:0};
    }
  }
  // Never publish an unverified smooth curve. The speed profile stops at sharp
  // fallback corners; the diagnostic makes this fallback visible.
  return {densify(route,cfg.sample_spacing),"POLYLINE_FALLBACK",0};
}
} // namespace fire_scout
