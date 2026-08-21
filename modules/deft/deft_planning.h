#pragma once

#include <memory>
#include <queue>
#include <set>
#include <utility>

#include "cyber/time/clock.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/on_lane_planning.h"

namespace apollo {
namespace deft {

struct PlannerCheckpoint {
  planning::PlanningStatus planning_status;

  planning::EgoInfo ego_info;
  size_t seq_num = 0;
  double start_time = 0.0;
  bool has_last_trajectory = false;
  planning::ADCTrajectory last_trajectory;

  std::set<uint32_t> frame_history_ids;
  size_t frame_history_capacity = 0;

  bool has_committed_inputs = false;
  localization::LocalizationEstimate committed_localization;
  canbus::Chassis committed_chassis;
};

class DeFTPlanning : public planning::OnLanePlanning {
 public:
  explicit DeFTPlanning(
      const std::shared_ptr<planning::DependencyInjector>& injector)
      : planning::OnLanePlanning(injector) {}

  void Checkpoint(PlannerCheckpoint* cp) {
    cp->planning_status = injector_->planning_context()->planning_status();
    cp->ego_info = *injector_->ego_info();
    cp->seq_num = seq_num_;
    cp->start_time = start_time_;

    cp->has_last_trajectory = (last_publishable_trajectory_ != nullptr);
    cp->last_trajectory.Clear();
    if (cp->has_last_trajectory) {
      last_publishable_trajectory_->PopulateTrajectoryProtobuf(
          &cp->last_trajectory);
    }

    auto* fh = injector_->frame_history();
    cp->frame_history_ids.clear();
    for (const auto& kv : fh->map_) {
      cp->frame_history_ids.insert(kv.first);
    }
    cp->frame_history_capacity = fh->capacity_;
    fh->capacity_ = 0;

    cp->has_committed_inputs = has_committed_inputs_;
    cp->committed_localization = committed_localization_;
    cp->committed_chassis = committed_chassis_;
  }

  void Trial(const planning::LocalView& local_view, double frame_time,
             planning::ADCTrajectory* out) {
    cyber::Clock::SetNowInSeconds(frame_time);
    RunOnce(local_view, out);
  }

  void Rollback(const PlannerCheckpoint& cp) {
    *injector_->planning_context()->mutable_planning_status() =
        cp.planning_status;
    *injector_->ego_info() = cp.ego_info;
    seq_num_ = cp.seq_num;
    start_time_ = cp.start_time;

    if (cp.has_last_trajectory) {
      last_publishable_trajectory_.reset(
          new planning::PublishableTrajectory(cp.last_trajectory));
    } else {
      last_publishable_trajectory_.reset();
    }

    auto* fh = injector_->frame_history();
    for (auto it = fh->map_.begin(); it != fh->map_.end();) {
      it = cp.frame_history_ids.count(it->first) ? std::next(it)
                                                 : fh->map_.erase(it);
    }

    std::queue<std::pair<uint32_t, const planning::Frame*>> kept;
    while (!fh->queue_.empty()) {
      auto entry = fh->queue_.front();
      fh->queue_.pop();
      if (cp.frame_history_ids.count(entry.first)) kept.push(entry);
    }
    fh->queue_ = std::move(kept);
    fh->capacity_ = cp.frame_history_capacity;

    if (cp.has_committed_inputs) {
      injector_->vehicle_state()->Update(cp.committed_localization,
                                         cp.committed_chassis);
    }
    has_committed_inputs_ = cp.has_committed_inputs;
    committed_localization_ = cp.committed_localization;
    committed_chassis_ = cp.committed_chassis;
  }

  void NoteCommitted(const planning::LocalView& local_view) {
    if (local_view.localization_estimate) {
      committed_localization_ = *local_view.localization_estimate;
    }
    if (local_view.chassis) {
      committed_chassis_ = *local_view.chassis;
    }
    has_committed_inputs_ =
        (local_view.localization_estimate != nullptr &&
         local_view.chassis != nullptr);
  }

  void Commit(const planning::LocalView& local_view, double frame_time,
              planning::ADCTrajectory* out) {
    cyber::Clock::SetNowInSeconds(frame_time);
    RunOnce(local_view, out);
    if (local_view.localization_estimate) {
      committed_localization_ = *local_view.localization_estimate;
    }
    if (local_view.chassis) {
      committed_chassis_ = *local_view.chassis;
    }
    has_committed_inputs_ =
        (local_view.localization_estimate != nullptr &&
         local_view.chassis != nullptr);
  }

 private:
  bool has_committed_inputs_ = false;
  localization::LocalizationEstimate committed_localization_;
  canbus::Chassis committed_chassis_;
};

}
}
