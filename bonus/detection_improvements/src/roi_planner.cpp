#include "detection_improvements/roi_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace ft_detector {
namespace {
// 四端点的轴对齐边界只用于搜索，PnP 仍使用原始四点。
cv::Rect2f boxOf(const ArmorCandidate& armor) {
  float x=armor.corners[0].x, y=armor.corners[0].y, right=x, bottom=y;
  for (const auto& p: armor.corners) {
    x=std::min(x,p.x); y=std::min(y,p.y);
    right=std::max(right,p.x); bottom=std::max(bottom,p.y);
  }
  return {x,y,right-x,bottom-y};
}
cv::Point2f centerOf(const cv::Rect2f& box) { return {box.x+box.width/2,box.y+box.height/2}; }
}

// 启动即检查完整参数，避免非法留白或时间设置进入逐帧处理。
void validateRoiParams(const RoiParams& params) {
  if (!std::isfinite(params.search_interval_s) || params.search_interval_s <= 0.0 ||
      !std::isfinite(params.max_history_gap_s) || params.max_history_gap_s < params.search_interval_s ||
      !std::isfinite(params.horizontal_padding) || params.horizontal_padding < 0.0 ||
      !std::isfinite(params.vertical_padding) || params.vertical_padding < 0.0 ||
      params.min_width <= 0 || params.min_height <= 0 || params.confirmation_searches < 2 || params.max_misses < 1)
    throw std::invalid_argument("Invalid ROI timing, padding or confirmation settings");
}

RoiPlanner::RoiPlanner(const RoiParams& params, const RoiPolicy& policy):params_(params),policy_(policy) {
  validateRoiParams(params_);
}

// 回退、换系、换分辨率与长间隔后不能继续沿用旧区域。
void RoiPlanner::reset() {
  regions_.clear(); pending_.clear(); last_stamp_=-1; last_search_=-1;
  frame_.clear(); size_={}; state_={}; force_search_=true;
}

// 根据最后观测、像素速度和留白生成安全区域，并裁剪到全图。
cv::Rect RoiPlanner::expanded(const Region& region) const {
  const double dt=std::max(0.0,stamp_-region.stamp);
  const auto center=centerOf(region.box)+region.velocity*static_cast<float>(dt);
  const double width=std::max<double>(params_.min_width,region.box.width*(1+2*params_.horizontal_padding));
  const double height=std::max<double>(params_.min_height,region.box.height*(1+2*params_.vertical_padding));
  const double extra_x=std::abs(region.velocity.x)*dt, extra_y=std::abs(region.velocity.y)*dt;
  const int x0=static_cast<int>(std::floor(std::max(0.0,center.x-width/2-extra_x)));
  const int y0=static_cast<int>(std::floor(std::max(0.0,center.y-height/2-extra_y)));
  const int x1=static_cast<int>(std::ceil(std::min<double>(size_.width,center.x+width/2+extra_x)));
  const int y1=static_cast<int>(std::ceil(std::min<double>(size_.height,center.y+height/2+extra_y)));
  return {x0,y0,std::max(0,x1-x0),std::max(0,y1-y0)};
}

bool RoiPlanner::compatible(const Region& region, const ArmorCandidate& armor) const {
  return region.red==armor.is_red && expanded(region).contains(armor.center);
}

// 使用图像时间安排全图刷新；合并多个区域后每帧仅调用一次检测。
RoiDebugState RoiPlanner::begin(cv::Size size, double stamp, const std::string& frame) {
  if (size.empty() || !std::isfinite(stamp) || stamp<0 || frame.empty())
    throw std::invalid_argument("ROI requires image dimensions, a finite timestamp and frame_id");
  if (size!=size_ || frame!=frame_ || (last_stamp_>=0 &&
      (stamp<=last_stamp_ || stamp-last_stamp_>params_.max_history_gap_s))) reset();
  stamp_=stamp; last_stamp_=stamp; size_=size; frame_=frame;
  state_={}; state_.enabled=params_.enabled;
  state_.crop={0,0,size.width,size.height};
  if (!params_.enabled) { state_.status="ROI off"; return state_; }
  for (const auto& region:regions_) {
    auto box=expanded(region);
    if (box.area()>0) state_.regions.push_back(box);
  }
  state_.full_search=force_search_ || state_.regions.empty() || last_search_<0 ||
                     stamp-last_search_>=params_.search_interval_s-1e-9;
  if (!state_.full_search) {
    cv::Rect united;
    for (const auto& box:state_.regions) united=united.area()>0 ? united|box : box;
    // NN 保留原 4:3 上下文；传统按实际目标留白，不额外纳入上下背景。
    if (united.area()>0 && policy_.keep_model_aspect) {
      double width=std::max<double>(united.width,united.height*4.0/3.0);
      double height=std::max<double>(united.height,united.width*3.0/4.0);
      width=std::min<double>(size.width,width);height=std::min<double>(size.height,height);
      const int w=static_cast<int>(std::ceil(width)),h=static_cast<int>(std::ceil(height));
      const int x=std::max(0,std::min(size.width-w,united.x+(united.width-w)/2));
      const int y=std::max(0,std::min(size.height-h,united.y+(united.height-h)/2));
      united={x,y,w,h};
    }
    if (united.area()>0 && united.area()<.8*size.area()) state_.crop=united;
    else state_.full_search=true;
  }
  if (state_.full_search) last_search_=stamp;
  force_search_=false;
  state_.status=state_.full_search ? "full discovery" : "local multi-target";
  return state_;
}

// 区域内候选直接输出；新区域须通过连续全图发现确认，不能一次误检就永久锁定。
std::vector<ArmorCandidate> RoiPlanner::finish(const std::vector<ArmorCandidate>& candidates) {
  if (!params_.enabled) return candidates;
  std::vector<ArmorCandidate> accepted;
  std::vector<bool> used(candidates.size(),false);
  for (auto& region:regions_) {
    int best=-1; double distance=std::numeric_limits<double>::infinity();
    const auto predicted=centerOf(region.box)+region.velocity*static_cast<float>(std::max(0.0,stamp_-region.stamp));
    for (std::size_t i=0;i<candidates.size();++i) {
      if (used[i] || !compatible(region,candidates[i])) continue;
      const double d=cv::norm(candidates[i].center-predicted);
      if (d<distance) { best=static_cast<int>(i); distance=d; }
    }
    if (best<0) { ++region.misses; continue; }
    const auto& armor=candidates[best];
    const double dt=stamp_-region.stamp;
    if (dt>1e-6) region.velocity=.5F*region.velocity+.5F*(armor.center-centerOf(region.box))/static_cast<float>(dt);
    region.box=boxOf(armor); region.stamp=stamp_; region.misses=0; used[best]=true;
  }
  // NN 沿用联合区域；传统只接纳独立区域内的候选，避免信任两车之间的空白。
  cv::Rect trusted;
  for (const auto& box:state_.regions) trusted=trusted.area()>0 ? trusted|box : box;
  for (std::size_t i=0;i<candidates.size();++i) {
    const bool inside = policy_.trust_union ? trusted.contains(candidates[i].center) :
        std::any_of(state_.regions.begin(),state_.regions.end(),[&](const cv::Rect& box) {
          return box.contains(candidates[i].center);
        });
    if (!used[i] && !inside) continue;
    accepted.push_back(candidates[i]);
    if (!used[i]) {
      Region region;region.box=boxOf(candidates[i]);region.red=candidates[i].is_red;region.stamp=stamp_;
      regions_.push_back(region);
    }
  }
  bool missed=false;
  regions_.erase(std::remove_if(regions_.begin(),regions_.end(),[&](const Region& r){
    // 两次漏检立即全图重搜，但短遮挡期间保留区域记忆，换面回来无需当成陌生车辆。
    if (r.misses>=params_.max_misses) missed=true;
    return stamp_-r.stamp>params_.max_history_gap_s;
  }),regions_.end());
  if (state_.full_search) {
    std::vector<Region> next_pending;
    std::vector<bool> pending_used(pending_.size(),false);
    for (const auto& armor:candidates) {
      if (std::any_of(accepted.begin(),accepted.end(),[&](const ArmorCandidate& a){return a.id==armor.id;})) continue;
      if (policy_.seed_filter && !policy_.seed_filter(armor)) continue;
      Region proposal; proposal.box=boxOf(armor); proposal.red=armor.is_red; proposal.stamp=stamp_;
      for (std::size_t i=0;i<pending_.size();++i) {
        if (!pending_used[i] && stamp_-pending_[i].stamp<=2.5*params_.search_interval_s && compatible(pending_[i],armor)) {
          proposal.confirmations=pending_[i].confirmations+1; pending_used[i]=true; break;
        }
      }
      if (proposal.confirmations>=params_.confirmation_searches) {
        regions_.push_back(proposal); accepted.push_back(armor);
      } else next_pending.push_back(proposal);
    }
    pending_=std::move(next_pending);
  }
  force_search_=missed || regions_.empty();
  std::sort(accepted.begin(),accepted.end(),[](const ArmorCandidate& a,const ArmorCandidate& b){
    return a.center.x!=b.center.x ? a.center.x<b.center.x : a.center.y<b.center.y;
  });
  for (std::size_t i=0;i<accepted.size();++i) accepted[i].id=i;
  return accepted;
}

// 只绘制搜索边界和运行状态，不改变输出图大小、角点或时间戳。
void drawRoiDebug(cv::Mat& image,const RoiDebugState& state) {
  if (!state.enabled) return;
  for (const auto& box:state.regions) cv::rectangle(image,box,{160,150,20},1);
  cv::rectangle(image,state.crop,{20,220,180},2);
  const std::string label="ROI: "+state.status+" | regions="+std::to_string(state.regions.size());
  cv::putText(image,label,{12,72},cv::FONT_HERSHEY_SIMPLEX,.6,{20,220,180},2,cv::LINE_AA);
}

}  // namespace ft_detector
