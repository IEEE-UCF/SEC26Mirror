#include "RobotManager.h"

using namespace Subsystem;

bool RobotManager::init() {
	timekeepers_.clear();
	timekeepers_.reserve(2);
	// 50 Hz (20 ms)
	timekeepers_.emplace_back(MS_20, DELAY_MS_20);
	// 500 Hz (2 ms)
	timekeepers_.emplace_back(MS_2, DELAY_MS_2);
	bool ok = true;
	for (auto* obj : setup_.objects_) {
		if (!obj) continue;
		auto& base_ref = const_cast<Classes::BaseClass&>(obj->base_);
		ok = base_ref.init() && ok;
	}
	return ok;
}

void RobotManager::update() {
	for (auto& tk : timekeepers_) {
		if (!tk.checkTimer()) continue;
		const auto cfg = tk.getConfig();
		for (auto* obj : setup_.objects_) {
			if (!obj) continue;
			if (obj->timer_config_ != cfg) continue;
			if (obj->update_fn_) {
				obj->update_fn_(&const_cast<Classes::BaseClass&>(obj->base_));
			}
		}
	}
}

void RobotManager::reset() {
	for (auto* obj : setup_.objects_) {
		if (!obj) continue;
		if (obj->reset_fn_) {
			obj->reset_fn_(&const_cast<Classes::BaseClass&>(obj->base_));
		}
	}
	timekeepers_.clear();
}

